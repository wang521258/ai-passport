// components/bsp/include/bsp_display.h
// ST7789 320x240 显示(8bit 8080 并口/i80):面板初始化 + 背光(XL9555 P1_0)。
#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"
#include <stdbool.h>
#include <stdint.h>

// 初始化 i80 总线、面板、背光。成功后屏幕已上电但内容未定。
esp_err_t bsp_display_init(void);

// 取底层面板句柄。想直接 esp_lcd_panel_draw_bitmap 画,或接 LVGL 以外的 GUI 时用。
// 未初始化返回 NULL。
esp_lcd_panel_handle_t bsp_display_panel(void);

// 取底层 panel io 句柄(LVGL 接入需要)。未初始化返回 NULL。
esp_lcd_panel_io_handle_t bsp_display_io(void);

// 背光亮度 0..100(%)。无亮度调节,0=灭 / >0=亮(XL9555 P1_0)。
void bsp_display_backlight(uint8_t percent);

// ---------------------------------------------------------------------------
// LVGL 接入(可选层)。必须先 bsp_display_init() 成功后再调。
// 不想用 LVGL 的开发者可忽略本段,直接用 bsp_display_panel() 自己画。
// ---------------------------------------------------------------------------
// 前向声明:LVGL 里 lv_display_t 是 `typedef struct _lv_display_t lv_display_t;`,
// 故此处用 struct 形式即可,避免本头文件强行 include lvgl.h。
struct _lv_display_t;

// 启动 LVGL 与其渲染任务,返回 lv_display_t*。失败返回 NULL。
struct _lv_display_t *bsp_lvgl_init(void);

// LVGL 非线程安全:在【非 LVGL 任务】里操作任何 lv_* 对象前后必须加解锁。
bool bsp_lvgl_lock(int timeout_ms);
void bsp_lvgl_unlock(void);
