// components/bsp/src/bsp_xio.c
// XL9555 扩展 IO(I2C 0x20)封装。
// 负责:LCD 背光(P1_0,高有效) + 功放使能(P0_5) + 按键输入(P1_4..P1_7)。
//
// 【为什么从 AW9523 换成 XL9555】
//   本板官方 SKU = atk-dnesp32s3-box(板子启动日志自报),I/O 扩展器是 XL9555@0x20,
//   I2C 在 SDA=GPIO48 / SCL=GPIO45。之前按 box3 的 AW9523@0x59 + SDA3/SCL2 去做,
//   总线上根本没有设备应答 → 扫描 0 设备、背光不亮、黑屏。
//
// 【寄存器布局(TCA9535/XL9555 兼容)】
//   0x00/0x01 输入 P0/P1   0x02/0x03 输出 P0/P1
//   0x06/0x07 方向 P0/P1(1=输入,0=输出)
// 位定义来自正点原子官方 xl9555.h。
//
// 注意:IDF v5.x 的 I2C 新驱动要求先在总线上"添加设备"得到 dev 句柄再收发。
#include "bsp_xio.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "bsp_xio";
static bool                    s_inited;
static i2c_master_dev_handle_t s_dev;

#define XIO_REG_INPUT   0x00   // 读 2 字节 → P0, P1
#define XIO_REG_OUTPUT  0x02   // 写 2 字节 → P0, P1
#define XIO_REG_DIR     0x06   // 写 2 字节 → P0, P1 (1=输入, 0=输出)

static uint16_t s_out;         // 16 位输出影子寄存器(pin 0..7=P0, 8..15=P1)

static esp_err_t xio_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 1000);
}

static esp_err_t xio_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, 1000);
}

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
    // 只更新 pin 所在的那一个端口字节(0x02=P0 / 0x03=P1)
    return (pin < 8) ? xio_write_reg(XIO_REG_OUTPUT,     (uint8_t)(s_out & 0xFF))
                     : xio_write_reg(XIO_REG_OUTPUT + 1, (uint8_t)(s_out >> 8));
}

esp_err_t bsp_xio_init(void) {
    if (s_inited) return ESP_OK;

    esp_err_t e = bsp_i2c_init();
    if (e != ESP_OK) return e;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) { ESP_LOGE(TAG, "I2C 总线为空"); return ESP_ERR_INVALID_STATE; }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_XIO_ADDR,
        .scl_speed_hz    = 100000,
    };
    if ((e = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev)) != ESP_OK) {
        ESP_LOGE(TAG, "XL9555 设备添加失败 (%s)", esp_err_to_name(e));
        return e;
    }

    // ---- 与官方 InitializeI2c() 的 XL9555 初始化逐条对齐 ----
    // 1) 先按 0x06=0x3B / 0x07=0xFE 配方向:P0_5 暂时当【输入】,用来读 SPK_CTRL 判 codec
    if ((e = xio_write16(XIO_REG_DIR, (uint16_t)(BSP_XIO_DIR_P0 | 0x20) | (BSP_XIO_DIR_P1 << 8))) != ESP_OK) goto fail;
    // 2) 读一次 P0 输入,判板载 codec(高=ES8311,低=NS4168)
    {
        uint8_t p0 = 0;
        if (xio_read_reg(XIO_REG_INPUT, &p0) == ESP_OK) {
            ESP_LOGI(TAG, "XL9555 在线,输入 P0=0x%02X → codec 判定:%s",
                     p0, (p0 & 0x20) ? "ES8311" : "NS4168");
        }
    }
    // 3) 输出寄存器全高(电源/使能类默认拉高即"上电";背光 P1_0 也随之点亮)
    s_out = 0xFFFF;
    if ((e = xio_write16(XIO_REG_OUTPUT, s_out)) != ESP_OK) goto fail;
    // 4) 正式方向:P0=0x1B / P1=0xFE
    if ((e = xio_write16(XIO_REG_DIR, (uint16_t)(BSP_XIO_DIR_P0 | (BSP_XIO_DIR_P1 << 8)))) != ESP_OK) goto fail;
    // 5) 回读输入做连通性确认
    uint16_t in = 0;
    if ((e = xio_read16(XIO_REG_INPUT, &in)) != ESP_OK) goto fail;

    s_inited = true;
    ESP_LOGI(TAG, "XL9555 就绪:背光已开(P1_0),输出=0x%04X,输入=0x%04X", s_out, in);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "XL9555 初始化失败 (%s) —— 检查 I2C SDA=GPIO%d SCL=GPIO%d, addr 0x%02X",
             esp_err_to_name(e), BSP_I2C_SDA, BSP_I2C_SCL, BSP_XIO_ADDR);
    return e;
}

// 背光:P1_0 高有效。on=1 → 写 1
void bsp_xio_set_bl(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_BL_PIN, on ? 1 : 0);
}

bool bsp_xio_get_key(uint8_t xio_pin) {
    uint16_t v = 0;
    if (!s_inited) return false;
    if (xio_read16(XIO_REG_INPUT, &v) != ESP_OK) return false;
    return (v >> xio_pin) & 1u;
}

// 一次性读回 16 位输入(按键诊断用)
bool bsp_xio_read_all(uint16_t *in) {
    if (!s_inited || !in) return false;
    return xio_read16(XIO_REG_INPUT, in) == ESP_OK;
}

void bsp_xio_set_pa(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_SPK_PIN, on ? 1 : 0);
}
