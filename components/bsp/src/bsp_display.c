// components/bsp/src/bsp_display.c
// ATK-DNESP32S3-BOX 显示:ST7789 320x240,8bit 8080 并口(i80,ESP32-S3 的 LCD_CAM 外设)。
//
// 与官方板级实现(main/boards/alientek/atk-dnesp32s3-box)逐条对齐:
//   总线 8bit / DC=GPIO2 WR=GPIO42 / CS=GPIO1 / RD=GPIO41 需配成"常高输出"
//   数据 D0..D7 = 40,39,38,12,11,10,9,46
//   面板:invert_color(1) / 0x36=0x00 / 0x3A=0x65 / swap_xy=1 / mirror(1,0)
//   背光:XL9555 P1_0 高有效(bsp_xio 负责),MCU 不直驱
#include "bsp_display.h"
#include "bsp_pins.h"
#include "bsp_xio.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

static const char *TAG = "bsp_disp";

static esp_lcd_panel_handle_t    s_panel;
static esp_lcd_panel_io_handle_t s_io;

esp_err_t bsp_display_init(void) {
    if (s_panel) return ESP_OK;

    // ---- RD(GPIO41)配成输出并拉高:本驱动只写不读,拉高即"常读无效" ----
    gpio_config_t rd = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = 1ull << BSP_LCD_RD,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&rd);
    gpio_set_level(BSP_LCD_RD, 1);

    // ---- i80 总线 ----
    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num = BSP_LCD_DC,
        .wr_gpio_num = BSP_LCD_WR,
        .clk_src     = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {
            BSP_LCD_D0, BSP_LCD_D1, BSP_LCD_D2, BSP_LCD_D3,
            BSP_LCD_D4, BSP_LCD_D5, BSP_LCD_D6, BSP_LCD_D7,
        },
        .bus_width = 8,
        .max_transfer_bytes = BSP_LCD_PANEL_W * BSP_LCD_PANEL_H * sizeof(uint16_t),
        .dma_burst_size = 64,
    };
    esp_err_t e = esp_lcd_new_i80_bus(&bus_cfg, &bus);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "i80 总线创建失败 (%s) —— 检查 DC=GPIO%d WR=GPIO%d 与数据线 GPIO%d..%d 是否冲突",
                 esp_err_to_name(e), BSP_LCD_DC, BSP_LCD_WR, BSP_LCD_D0, BSP_LCD_D7);
        return e;
    }

    esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num        = BSP_LCD_CS,
        .pclk_hz            = BSP_LCD_PCLK_HZ,
        .trans_queue_depth  = 10,
        .on_color_trans_done = NULL,
        .user_ctx           = NULL,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 8,
        .dc_levels = {
            .dc_idle_level  = 0,
            .dc_cmd_level   = 0,
            .dc_dummy_level = 0,
            .dc_data_level  = 1,
        },
        // 8 位并口按字节送,不需要 SPI 那种高低字节交换
        .flags = { .swap_color_bytes = 0 },
    };
    e = esp_lcd_new_panel_io_i80(bus, &io_cfg, &s_io);
    if (e != ESP_OK) { ESP_LOGE(TAG, "panel_io(i80) 创建失败: %s", esp_err_to_name(e)); return e; }

    esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = BSP_LCD_RST,          // -1 → SWRESET 软复位
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    e = esp_lcd_new_panel_st7789(s_io, &dev, &s_panel);
    if (e != ESP_OK) { ESP_LOGE(TAG, "面板创建失败: %s", esp_err_to_name(e)); return e; }

    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);

    // 官方在 panel_init 之后覆写这两条(init 里按 bpp 写的是 0x55)
    uint8_t d36 = 0x00;   // MADCTL 基准,后续 swap/mirror 会在其上合成
    uint8_t d3a = 0x65;   // COLMOD:16bit
    esp_lcd_panel_io_tx_param(s_io, 0x36, &d36, 1);
    esp_lcd_panel_io_tx_param(s_io, 0x3A, &d3a, 1);

    esp_lcd_panel_invert_color(s_panel, BSP_LCD_INVERT_COLOR);
    esp_lcd_panel_swap_xy(s_panel, BSP_LCD_SWAP_XY);                 // 0x36 MV 位
    esp_lcd_panel_mirror(s_panel, BSP_LCD_MIRROR_X, BSP_LCD_MIRROR_Y);
    esp_lcd_panel_set_gap(s_panel, 0, 0);
    esp_lcd_panel_disp_on_off(s_panel, true);

    bsp_xio_set_bl(1);
    ESP_LOGI(TAG, "显示就绪 %dx%d 竖屏(面板原生 %dx%d,8080 并口 8bit)",
             BSP_LCD_W, BSP_LCD_H, BSP_LCD_PANEL_W, BSP_LCD_PANEL_H);
    return ESP_OK;
}

esp_lcd_panel_handle_t bsp_display_panel(void) { return s_panel; }

esp_lcd_panel_io_handle_t bsp_display_io(void) { return s_io; }

void bsp_display_backlight(uint8_t percent) {
    // 背光只有开/关(XL9555 P1_0,高有效),无亮度调节。
    bsp_xio_set_bl(percent > 0 ? 1 : 0);
}
