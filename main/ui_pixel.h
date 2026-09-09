#pragma once

#include "lvgl.h"

#define UI_SKY        0x1689E8
#define UI_SKY_DARK   0x0872C9
#define UI_INK        0x17202A
#define UI_PAPER      0xF4F4EA
#define UI_GRASS      0x82BE2D
#define UI_GRASS_DARK 0x55951D
#define UI_YELLOW     0xFFD928
#define UI_ORANGE     0xFFB23E
#define UI_RED        0xE43B2F
#define UI_MUTED      0xD9E7EC

lv_obj_t *ui_pixel_screen_create(const char *title);

/* 基础色块：宠物玩法（demo_pet.c / ui_pet.c）需要用 block() 绘制背景与
   精灵球，因此这里去掉 static 并对外导出。
   注意：每个 block 都是一个独立 LVGL 对象，ESP32-C3 只有约 80 KB 动态
   RAM，请勿用它做逐像素或大循环绘制。 */
lv_obj_t *block(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color);
lv_obj_t *ui_pixel_panel_create(lv_obj_t *parent, int x, int y, int w, int h,
                                uint32_t color);
lv_obj_t *ui_pixel_label(lv_obj_t *parent, const char *text,
                         const lv_font_t *font, uint32_t color);
lv_obj_t *ui_pixel_mascot_create(lv_obj_t *parent, int x, int y);
void ui_pixel_mascot_jump(lv_obj_t *mascot);
void ui_pixel_set_selected(lv_obj_t *panel, bool selected, bool enabled);
