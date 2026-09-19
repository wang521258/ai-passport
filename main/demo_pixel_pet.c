// main/demo_pixel_pet.c —— 电子宠物：v43 本尊离线渲染 16 帧（144x144 RGB565 RLE）。
// 形象 = 「3D 萌宠实验室」宠物在 PC 上逐帧"拍照"烤成的点阵，非手绘。
//
// 交互（语义遵循仓库约定，长按确定返回菜单由 main.c 统一拦截）:
//   上   短按 = 喂食（饱食 +30）
//   下   短按 = 玩耍（开心 +20，精力 -15，精力不足会拒绝）
//   确定 短按 = 睡觉/唤醒（睡觉时精力自动恢复，屏幕调暗省电）
//
// 帧数据 pet_real.h 为机器生成，勿手改。
// 生成器: D:/workbuddy2/esp32_pixel_pet/_bake_real.py（输入 work/pet_full/*.png）
#include "demo.h"
#include "bsp_display.h"
#include "bsp_pins.h"      // BSP_LCD_W/H（PET_X 要用）
#include "ui_pixel.h"
#include "pet_real.h"
#include "lvgl.h"

// LV_COLOR_DEPTH=16 → lv_color_t 就是 5/6/5 位域结构，**没有 .full 成员**
//（那是别的库的写法）。这里给两个小工具，把 RGB565 的裸值和 lv_color_t 互相转换。
static inline lv_color_t pet_rgb565(uint16_t v)
{
    lv_color_t c;
    c.red   = (uint16_t)((v >> 11) & 0x1F);
    c.green = (uint16_t)((v >> 5) & 0x3F);
    c.blue  = (uint16_t)(v & 0x1F);
    return c;
}
static inline int pet_is_same(lv_color_t a, lv_color_t b)
{
    return a.red == b.red && a.green == b.green && a.blue == b.blue;
}

#define ANIM_MS      220                              // 动画帧间隔（≈原始抓帧节奏）
#define CANVAS_PX    PET_REAL_SIZE                    // 144
#define PET_X        ((BSP_LCD_W - CANVAS_PX) / 2)
#define PET_Y        92
#define DECAY_TICKS  273                              // 220ms × 273 ≈ 60s 扣一次状态
#define SLEEP_REGEN  7                                // 睡觉每 7 tick(1.5s)回 1 精力

typedef enum { PET_IDLE, PET_EAT, PET_PLAY, PET_SLEEP } pet_state_t;

static lv_obj_t *s_scr;
static lv_obj_t *s_canvas;
static lv_obj_t *s_bars[3];          // 0饱食 1开心 2精力
static lv_obj_t *s_lvl_label;
static lv_obj_t *s_bubble;           // 状态气泡文字
static lv_timer_t *s_tick;
static int s_state_ticks;

// LV_COLOR_DEPTH=16 → 144×144×2 ≈ 40.5KB 静态缓冲,不占 LVGL 堆(仅24KB)
static uint8_t s_canvas_buf[CANVAS_PX * CANVAS_PX * sizeof(lv_color_t)];

static pet_state_t s_state = PET_IDLE;
static uint8_t s_seq_pos;
static int s_hunger = 70, s_fun = 60, s_energy = 80;
static int s_exp;

static const char *bubble_text(void) {
    switch (s_state) {
    case PET_EAT:   return "YUM!";
    case PET_PLAY:  return "WHEE!";
    case PET_SLEEP: return "Zzz...";
    default:        return "";
    }
}

// RLE 解码一帧到画布。透明 run 填背景色。
static void draw_frame(uint16_t fi) {
    static uint16_t last = 0xFFFF;
    if (fi == last) return;
    last = fi;
    const uint16_t *p = PET_REAL + PET_REAL_OFF[fi];
    const uint16_t *end = PET_REAL + PET_REAL_OFF[fi + 1];
    lv_color_t *buf = (lv_color_t *)s_canvas_buf;
    lv_color_t bg = lv_color_hex(UI_SKY);            // 透明处透出天空底色
    lv_color_t *o = buf;
    const int total = CANVAS_PX * CANVAS_PX;
    int written = 0;
    while (p < end && written < total) {
        uint16_t v = *p++;
        int n = v >> 1;
        if (v & 1) {                                  // 实色 run（RGB565 直通）
            lv_color_t c = pet_rgb565(v);
            for (int i = 0; i < n; i++) o[i] = c;
        } else {                                      // 透明 run
            for (int i = 0; i < n; i++) o[i] = bg;
        }
        o += n;
        written += n;
    }
    // 贴地椭圆影（程序画，替代 3D 场景阴影）：UI_SKY 70% + 深棕 30% 预混
    lv_color_t sh = lv_color_hex(0x2C75B4);
    const int cx = CANVAS_PX / 2, cy = PET_REAL_BASE_Y + 4;
    for (int dy = -7; dy <= 7; dy++) {
        for (int dx = -36; dx <= 36; dx++) {
            if (dx * dx * 49 + dy * dy * 1296 <= 36 * 36 * 49) {
                lv_color_t *q = buf + (cy + dy) * CANVAS_PX + (cx + dx);
                if (pet_is_same(q[0], bg)) q[0] = sh;   // 只画在背景上,不盖宠物
            }
        }
    }
    lv_obj_invalidate(s_canvas);
}

static void bars_refresh(void) {
    lv_bar_set_value(s_bars[0], s_hunger, LV_ANIM_OFF);
    lv_bar_set_value(s_bars[1], s_fun, LV_ANIM_OFF);
    lv_bar_set_value(s_bars[2], s_energy, LV_ANIM_OFF);
    lv_label_set_text_fmt(s_lvl_label, "Lv.%d", s_exp / 8 + 1);
    const char *b = bubble_text();
    lv_label_set_text(s_bubble, b);
    if (b[0]) lv_obj_remove_flag(s_bubble, LV_OBJ_FLAG_HIDDEN);
    else      lv_obj_add_flag(s_bubble, LV_OBJ_FLAG_HIDDEN);
}

static void set_state(pet_state_t st, int ticks) {
    s_state = st;
    s_seq_pos = 0;
    s_state_ticks = ticks;
}

// ---------- idle 行为调度器：像活物而不是循环 GIF ----------
// 帧语义（来自 _bake_real.py 抓帧分析）:
//   0-2 睁眼站立微动  3-5 闭眼(眨)  6-9 睁开过渡  10-13 慢闭眼(犯困)  14-15 睁开
typedef enum { BEH_REST, BEH_BREATHE, BEH_BLINK, BEH_DROWSY } beh_t;
static const uint8_t BR_SEQ[]   = {0, 1, 2, 1};              // 慢呼吸 ping-pong
static const uint8_t BLINK_SEQ[] = {2, 3, 4, 5, 6, 7};       // 眨一下眼（快）
static const uint8_t DROW_SEQ[]  = {9, 10, 11, 12, 11, 10, 9};
static beh_t s_beh = BEH_REST;
static int s_beh_ticks;              // 本行为剩余 tick
static int s_step, s_sub;            // 行为内帧步进 / 分频计数

static void next_behavior(void) {
    int r = rand() % 100;
    s_step = 0; s_sub = 0;
    if (r < 55) {                                   // 安静站立 4.4~7s
        s_beh = BEH_REST;    s_beh_ticks = 20 + rand() % 14;
    } else if (r < 85) {                            // 慢呼吸 1~1.5 轮
        s_beh = BEH_BREATHE; s_beh_ticks = 16 + (rand() % 2) * 8;
    } else if (r < 93) {                            // 眨一下眼
        s_beh = BEH_BLINK;   s_beh_ticks = (int)sizeof(BLINK_SEQ);
    } else {                                        // 犯困眯眼再睁开
        s_beh = BEH_DROWSY;  s_beh_ticks = (int)sizeof(DROW_SEQ) * 2;
    }
}

// 220ms 一拍:推进动画 + 状态衰减/恢复
static void tick_cb(lv_timer_t *t) {
    (void)t;
    static int decay_cnt;
    static int regen_cnt;

    if (s_state == PET_IDLE) {
        switch (s_beh) {
        case BEH_REST:
            draw_frame(0);                                   // 静止帧
            break;
        case BEH_BREATHE:                                    // 2 tick 一步 ≈ 440ms 慢呼吸
            if (++s_sub >= 2) {
                s_sub = 0;
                draw_frame(BR_SEQ[s_step & 3]);
                s_step++;
            }
            break;
        case BEH_BLINK:                                      // 1 tick 一步，快速眨眼
            draw_frame(BLINK_SEQ[s_step]);
            s_step++;
            break;
        case BEH_DROWSY:                                     // 2 tick 一步犯困
            if (++s_sub >= 2) {
                s_sub = 0;
                draw_frame(DROW_SEQ[s_step % (sizeof(DROW_SEQ) - 1)]);
                s_step++;
            }
            break;
        }
        if (--s_beh_ticks <= 0) next_behavior();
    } else if (s_state == PET_SLEEP) {
        draw_frame(11 + ((s_seq_pos >> 3) & 1));             // 睡觉:极慢闭眼微动
        s_seq_pos++;
    } else {
        if (++s_sub >= 2) {                                  // 吃/玩:慢速过完整动作 8帧半/3.5s
            s_sub = 0;
            draw_frame(s_seq_pos % PET_REAL_FRAMES);
            s_seq_pos++;
        }
    }

    if (s_state != PET_SLEEP && s_state_ticks > 0 && --s_state_ticks == 0) {
        set_state(PET_IDLE, 0);
    }
    if (s_state == PET_SLEEP) {
        if (++regen_cnt >= SLEEP_REGEN) {
            regen_cnt = 0;
            if (s_energy < 100) s_energy++;
            else {                                  // 睡满自动醒
                set_state(PET_IDLE, 0);
                bsp_display_backlight(100);
            }
        }
        if (s_hunger > 0) s_hunger--;
    } else if (++decay_cnt >= DECAY_TICKS) {       // 每 60s 衰减一次
        decay_cnt = 0;
        if (s_hunger > 0) s_hunger--;
        if (s_fun > 0) s_fun--;
        if (s_energy > 0) s_energy--;
    }
    bars_refresh();
}

static lv_obj_t *stat_panel(int x, const char *name, uint32_t bar_color) {
    lv_obj_t *panel = ui_pixel_panel_create(s_scr, x, 46, 70, 44, UI_PAPER);
    lv_obj_t *label = ui_pixel_label(panel, name, &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(label, 0, 0);
    lv_obj_t *bar = lv_bar_create(panel);
    lv_obj_set_size(bar, lv_pct(100), 8);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xD3D1C7), 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(bar_color), LV_PART_INDICATOR);
    return bar;
}

static lv_obj_t *action_btn(int x, const char *name) {
    lv_obj_t *panel = ui_pixel_panel_create(s_scr, x, 238, 68, 40, UI_YELLOW);
    lv_obj_t *label = ui_pixel_label(panel, name, &lv_font_montserrat_14, UI_INK);
    lv_obj_center(label);
    return panel;
}

void demo_pixel_pet_enter(void) {
    s_hunger = 70; s_fun = 60; s_energy = 80; s_exp = 0;
    set_state(PET_IDLE, 0);

    s_scr = ui_pixel_screen_create("CUTE PET");

    s_bars[0] = stat_panel(8,  "FOOD", UI_RED);
    s_bars[1] = stat_panel(85, "FUN",  UI_ORANGE);
    s_bars[2] = stat_panel(162, "ENE", UI_GRASS);

    s_canvas = lv_canvas_create(s_scr);
    lv_canvas_set_buffer(s_canvas, s_canvas_buf, CANVAS_PX, CANVAS_PX, LV_COLOR_FORMAT_NATIVE);
    lv_obj_set_pos(s_canvas, PET_X, PET_Y);

    // 等级牌压在画布左上角
    lv_obj_t *badge = ui_pixel_panel_create(s_scr, 8, 92, 54, 24, UI_PAPER);
    s_lvl_label = ui_pixel_label(badge, "Lv.1", &lv_font_montserrat_14, UI_INK);
    lv_obj_center(s_lvl_label);

    // 状态气泡（画布右上角）
    s_bubble = ui_pixel_label(s_scr, "", &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(s_bubble, PET_X + CANVAS_PX - 60, PET_Y + 4);
    lv_obj_add_flag(s_bubble, LV_OBJ_FLAG_HIDDEN);

    action_btn(8, "FOOD");
    action_btn(86, "PLAY");
    action_btn(164, "SLEEP");

    s_seq_pos = 0;
    draw_frame(0);
    bars_refresh();
    lv_screen_load(s_scr);

    s_tick = lv_timer_create(tick_cb, ANIM_MS, NULL);
}

void demo_pixel_pet_exit(void) {
    // 规范:删屏前先停定时器,恢复背光避免菜单看不清
    if (s_tick) { lv_timer_delete(s_tick); s_tick = NULL; }
    bsp_display_backlight(100);
    if (s_scr) { lv_obj_delete(s_scr); s_scr = NULL; s_canvas = NULL; s_lvl_label = NULL; s_bubble = NULL; }
}

void demo_pixel_pet_key(bsp_btn_t btn, bsp_btn_ev_t ev) {
    if (ev != BSP_BTN_CLICK) return;
    if (btn == BSP_BTN_UP) {                       // 喂食
        if (s_state == PET_SLEEP) return;
        s_hunger = (s_hunger + 30 > 100) ? 100 : s_hunger + 30;
        s_exp++;
        set_state(PET_EAT, 16);                    // 气泡显示约 3.5s
    } else if (btn == BSP_BTN_DOWN) {              // 玩耍
        if (s_state == PET_SLEEP || s_energy < 15) return;
        s_fun = (s_fun + 20 > 100) ? 100 : s_fun + 20;
        s_energy -= 15;
        s_exp++;
        set_state(PET_PLAY, 16);
    } else if (btn == BSP_BTN_OK) {                // 睡觉/唤醒
        if (s_state == PET_SLEEP) {
            set_state(PET_IDLE, 0);
            bsp_display_backlight(100);
        } else {
            set_state(PET_SLEEP, 0);
            bsp_display_backlight(30);             // 睡觉调暗省电
        }
    }
    bars_refresh();
}
