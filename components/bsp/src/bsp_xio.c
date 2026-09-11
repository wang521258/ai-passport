// components/bsp/src/bsp_xio.c
// AW9523 扩展 IO 封装(寄存器映射与 TCA9535 一致,见下方说明)。
// 负责:板上多路电源使能(VDD_3V3 / VDDA_3V3 / VDD_2V8 / VBAT / ESP_ADC_SEL / PA)
//       + LCD 背光(pin 8,低有效) + K1/K2 按键输入(pin 0 / pin 1)。
// 直接复用 bsp_i2c 已建好的 I2C 总线,不引入 esp_io_expander 组件依赖。
//
// 【寄存器映射】本板官方板级实现(atk_dnesp32s3_box3)用的是
// esp_io_expander_tca95xx_16bit 驱动 —— 即 AW9523 在这里按 TCA9535 的寄存器布局访问:
//     0x00/0x01 输入 P0/P1   0x02/0x03 输出 P0/P1   0x06/0x07 方向 P0/P1(1=输入,0=输出)
// 之前误按"方向=0x06、模式=0x10/0x11、初值 DIR_P1=0xFF"去初始化:
//   0xFF 的含义是【P1 全部当输入】→ VDD_3V3_EN/VBAT_EN/VDDA_3V3_EN/VDD_2V8_EN/背光
//   这些输出脚从来没被驱动过 → 板子缺电 + 黑屏。现按官方驱动逐条对齐。
//
// 注意:IDF v5.x 的 I2C 新驱动要求先在总线上"添加设备"得到 dev 句柄,
//       再以 dev 句柄做收发(旧 API 直接传 bus+地址已失效)。
#include "bsp_xio.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "bsp_xio";
static bool                    s_inited;
static i2c_master_dev_handle_t s_dev;   // AW9523 设备句柄

// AW9523 / TCA9535 兼容寄存器(TCA9535 布局:16 位值 = 低 8 位 P0 + 高 8 位 P1,
// 连续两个寄存器地址,一次 3 字节写即可同时设置两个端口)。
#define XIO_REG_INPUT   0x00   // 读 2 字节 → P0, P1
#define XIO_REG_OUTPUT  0x02   // 写 2 字节 → P0, P1
#define XIO_REG_DIR     0x06   // 写 2 字节 → P0, P1 (1=输入, 0=输出)

// 16 位输出影子寄存器,便于按位改电平而不用重读硬件。
static uint16_t s_out = 0xFFFF;

static esp_err_t xio_write16(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(s_dev, buf, 3, 1000);
}

static esp_err_t xio_read16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2] = { 0, 0 };
    esp_err_t e = i2c_master_transmit_receive(s_dev, &reg, 1, buf, 2, 1000);
    if (e == ESP_OK && val) *val = (uint16_t)(buf[0] | (buf[1] << 8));
    return e;
}

static esp_err_t xio_set_pin(uint8_t pin, uint8_t level) {
    if (level) s_out |=  (uint16_t)(1u << pin);
    else       s_out &= (uint16_t)~(1u << pin);
    return xio_write16(XIO_REG_OUTPUT, s_out);
}

esp_err_t bsp_xio_init(void) {
    if (s_inited) return ESP_OK;

    esp_err_t e = bsp_i2c_init();
    if (e != ESP_OK) return e;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_LOGE(TAG, "I2C 总线为空");
        return ESP_ERR_INVALID_STATE;
    }

    // 在总线上添加 AW9523 设备(7 位地址,100kHz —— 官方驱动亦为 400k,100k 更稳)
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_XIO_ADDR,
        .scl_speed_hz    = 100000,
    };
    if ((e = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev)) != ESP_OK) {
        ESP_LOGE(TAG, "AW9523 设备添加失败 (%s)", esp_err_to_name(e));
        return e;
    }

    // ---- 与官方 tca95xx_16bit 驱动的 reset() 逐条对齐 ----
    // 1) 方向先全部置输入(复位默认态),避免方向未定期间误驱动
    if ((e = xio_write16(XIO_REG_DIR, 0xFFFF)) != ESP_OK) goto fail;
    // 2) 输出寄存器全高:电源使能为高有效,默认拉高即"上电"
    s_out = 0xFFFF;
    if ((e = xio_write16(XIO_REG_OUTPUT, s_out)) != ESP_OK) goto fail;
    // 3) 方向:P0_0/P0_1(K1/K2 按键)= 输入,其余全部输出
    if ((e = xio_write16(XIO_REG_DIR, 0x0003)) != ESP_OK) goto fail;

    // 4) 按板级定义落具体电平(与官方 InitializeIoExpander 一致):
    //    VDD_2V8_EN(14)=1  VDD_3V3_EN(11)=1  ESP_ADC_SEL(4)=1
    //    VDDA_3V3_EN(13)=1 VBAT_EN(12)=1     PA_CTRL(5)=1    LCD_BL(8)=0(低有效=点亮)
    if ((e = xio_set_pin(BSP_XIO_BL_PIN, 0)) != ESP_OK) goto fail;   // 背光先灭一下再点
    if ((e = xio_set_pin(BSP_XIO_ADC_SEL_PIN,  1)) != ESP_OK) goto fail;   // ESP_ADC_SEL
    if ((e = xio_set_pin(BSP_XIO_PA_PIN,       1)) != ESP_OK) goto fail;   // PA_CTRL 音频功放
    if ((e = xio_set_pin(BSP_XIO_VDD_3V3_PIN,  1)) != ESP_OK) goto fail;   // VDD_3V3_EN
    if ((e = xio_set_pin(BSP_XIO_VBAT_PIN,     1)) != ESP_OK) goto fail;   // VBAT_EN
    if ((e = xio_set_pin(BSP_XIO_VDDA_3V3_PIN, 1)) != ESP_OK) goto fail;   // VDDA_3V3_EN
    if ((e = xio_set_pin(BSP_XIO_VDD_2V8_PIN,  1)) != ESP_OK) goto fail;   // VDD_2V8_EN
    if ((e = xio_set_pin(BSP_XIO_BL_PIN, 1)) != ESP_OK) goto fail;   // 背光低有效 → 写 0

    // 5) 回读一次输入寄存器做连通性确认(能读到就说明 I2C 真的通了)
    uint16_t in = 0;
    if ((e = xio_read16(XIO_REG_INPUT, &in)) != ESP_OK) goto fail;

    s_inited = true;
    ESP_LOGI(TAG, "AW9523 就绪:电源使能已拉高,背光已开,输出=0x%04X,输入=0x%04X", s_out, in);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "AW9523 初始化失败 (%s) —— 检查 I2C SDA=GPIO%d SCL=GPIO%d, addr 0x%02X",
             esp_err_to_name(e), BSP_I2C_SDA, BSP_I2C_SCL, BSP_XIO_ADDR);
    return e;
}

// 背光为低有效:on=1 → 写 0 点亮
void bsp_xio_set_bl(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_BL_PIN, on ? 0 : 1);
}

bool bsp_xio_get_key(uint8_t xio_pin) {
    uint16_t v = 0;
    if (!s_inited) return false;
    if (xio_read16(XIO_REG_INPUT, &v) != ESP_OK) return false;
    return (v >> xio_pin) & 1u;
}

void bsp_xio_set_pa(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_PA_PIN, on ? 1 : 0);
}
