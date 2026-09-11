// components/bsp/src/bsp_xio.c
// AW9523 I/O 扩展器(TCA95xx 兼容寄存器)封装。
// 负责:板上多路电源使能(VDD_3V3 / VDDA_3V3 / VDD_2V8 / VBAT / ESP_ADC_SEL / PA)
//       + LCD 背光(P0_8,低有效) + K1/K2 按键输入(P0_0 / P0_1)。
// 直接复用 bsp_i2c 已建好的 I2C 总线,不引入 esp_io_expander 组件依赖。
#include "bsp_xio.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "bsp_xio";
static bool s_inited;

// AW9523 寄存器(TCA95xx 兼容;16 位方向/输出分两个 8 位端口 P0/P1)
#define XIO_REG_INPUT_P0   0x00
#define XIO_REG_INPUT_P1   0x01
#define XIO_REG_OUTPUT_P0  0x02
#define XIO_REG_OUTPUT_P1  0x03
#define XIO_REG_DIR_P0     0x06   // 1 = 输入, 0 = 输出
#define XIO_REG_DIR_P1     0x07
#define XIO_REG_MODE_P0    0x10   // 0 = GPIO 模式(非 LED 渐变)
#define XIO_REG_MODE_P1    0x11

static esp_err_t xio_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(bsp_i2c_bus(), BSP_XIO_ADDR, buf, 2, 50);
}
static esp_err_t xio_read(uint8_t reg, uint8_t *val) {
    return i2c_master_receive(bsp_i2c_bus(), BSP_XIO_ADDR, val, 1, 50);
}

static uint8_t s_out_p0 = 0;
static uint8_t s_out_p1 = 0;

static void xio_set_pin(uint8_t pin, uint8_t level) {
    if (pin < 8) {
        if (level) s_out_p0 |= (1u << pin); else s_out_p0 &= ~(1u << pin);
        xio_write(XIO_REG_OUTPUT_P0, s_out_p0);
    } else {
        uint8_t p = pin - 8;
        if (level) s_out_p1 |= (1u << p); else s_out_p1 &= ~(1u << p);
        xio_write(XIO_REG_OUTPUT_P1, s_out_p1);
    }
}

esp_err_t bsp_xio_init(void) {
    if (s_inited) return ESP_OK;
    if (!bsp_i2c_bus()) {
        ESP_LOGE(TAG, "请先成功调用 bsp_i2c_init()");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t e;
    // 先切 GPIO 模式(关 LED 渐变),再设方向、再设输出电平 —— 顺序不能反。
    if ((e = xio_write(XIO_REG_MODE_P0, 0x00)) != ESP_OK) goto fail;
    if ((e = xio_write(XIO_REG_MODE_P1, 0x00)) != ESP_OK) goto fail;
    // 方向:P0_0/P0_1 输入(键),其余输出;P1 全输出。
    if ((e = xio_write(XIO_REG_DIR_P0, 0x03)) != ESP_OK) goto fail;
    if ((e = xio_write(XIO_REG_DIR_P1, 0xFF)) != ESP_OK) goto fail;

    // 输出初值:全部先清 0,再按需拉高。
    s_out_p0 = 0; s_out_p1 = 0;
    xio_set_pin(4, 1);   // P0_4 ESP_ADC_SEL
    xio_set_pin(5, 1);   // P0_5 音频功放 PA
    xio_set_pin(8 + 3, 1); // P1_3 VDD_3V3_EN
    xio_set_pin(8 + 4, 1); // P1_4 VBAT_EN
    xio_set_pin(8 + 5, 1); // P1_5 VDDA_3V3_EN(音频电源)
    xio_set_pin(8 + 6, 1); // P1_6 VDD_2V8_EN
    xio_set_pin(8 + 0, 0); // P1_0 LCD_BL 低有效 → 开背光

    s_inited = true;
    ESP_LOGI(TAG, "AW9523 就绪(背光已开,电源使能已拉高)");
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "AW9523 初始化失败 (%s) —— 检查 I2C SDA=GPIO%d SCL=GPIO%d, addr 0x%02X",
             esp_err_to_name(e), BSP_I2C_SDA, BSP_I2C_SCL, BSP_XIO_ADDR);
    return e;
}

void bsp_xio_set_bl(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_BL_PIN, on ? 0 : 1);   // 低有效
}

bool bsp_xio_get_key(uint8_t xio_pin) {
    uint8_t v = 0;
    if (xio_read(XIO_REG_INPUT_P0, &v) != ESP_OK) return false;
    return (v >> xio_pin) & 1u;
}

void bsp_xio_set_pa(uint8_t on) {
    if (!s_inited) return;
    xio_set_pin(BSP_XIO_PA_PIN, on ? 1 : 0);
}
