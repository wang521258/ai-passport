// components/bsp/src/bsp_display_lvgl.c
// LVGL 接入单独成文件:不用 LVGL 的开发者删掉本文件 + idf_component.yml 里的两条依赖即可。
#include "bsp_display.h"
#include "bsp_pins.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

static const char *TAG = "bsp_lvgl";

static lv_display_t *s_disp;

lv_display_t *bsp_lvgl_init(void) {
    if (s_disp) return s_disp;
    if (!bsp_display_panel()) {
        ESP_LOGE(TAG, "请先成功调用 bsp_display_init()");
        return NULL;
    }

    const lvgl_port_cfg_t pc = ESP_LVGL_PORT_INIT_CONFIG();
    if (lvgl_port_init(&pc) != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init 失败");
        return NULL;
    }

    const lvgl_port_display_cfg_t dc = {
        .panel_handle = bsp_display_panel(),
        .io_handle    = bsp_display_io(),
        // S3 有 8MB octal PSRAM:用 40 行双缓冲放 PSRAM,刷新快且稳。
        .buffer_size   = (uint32_t)BSP_LCD_W * 40,
        .double_buffer = true,
        // 画布 = 240x320 竖屏。旋转已经在 bsp_display.c 里用面板级
        // esp_lcd_panel_swap_xy()/mirror() 做掉了(软件坐标空间随之变成 240x320),
        // 所以这里 rotation 全关、分辨率直接给竖屏值,两层不要重复旋转。
        .hres = BSP_LCD_W, .vres = BSP_LCD_H,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        // 8080 并口按 16bit 走,不需要 SPI 那种高低字节交换(官方 swap_color_bytes=0)。
        .flags = { .buff_spiram = true, .swap_bytes = false },
    };
    s_disp = lvgl_port_add_disp(&dc);
    if (!s_disp) { ESP_LOGE(TAG, "lvgl_port_add_disp 失败"); return NULL; }

    ESP_LOGI(TAG, "LVGL 就绪");
    return s_disp;
}

bool bsp_lvgl_lock(int timeout_ms) { return lvgl_port_lock(timeout_ms); }
void bsp_lvgl_unlock(void)         { lvgl_port_unlock(); }
