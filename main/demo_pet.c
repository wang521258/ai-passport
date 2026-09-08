// main/demo_pet.c —— 电子宠物演示页。
// 精灵球破壳 → 随机宝可梦 GIF 动画；四项状态随时间衰减；
// 3 按键映射 5 动作：上=喂食 下=玩耍 中=睡觉 上长按=清洁 下长按=训练；
// 训练获得经验，满经验升级。
#include "demo.h"
#include "bsp_display.h"
#include "ui_pixel.h"
#include "ui_pet.h"
#include "gif_player.h"
#include "pokemon_sprites.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <string.h>
#include <stdlib.h>

/* ---------- 宠物状态 ---------- */
typedef struct {
    uint8_t hunger;   /* 饱食 0~100 */
    uint8_t happy;    /* 心情 0~100 */
    uint8_t energy;   /* 精力 0~100 */
    uint8_t clean;    /* 清洁 0~100 */
    uint8_t lv;       /* 等级 */
    uint16_t exp;     /* 经验 0~99 */
} pet_stat_t;

static pet_stat_t s_stat = { 80, 80, 80, 80, 1, 0 };
static lv_obj_t  *s_scr;
static lv_obj_t  *s_gif;       /* 宝可梦 GIF canvas */
static lv_obj_t  *s_ball;
static lv_obj_t  *s_namelabel;
static lv_obj_t  *s_bars[4];
static lv_obj_t  *s_vals[4];
static lv_obj_t  *s_lvlabel;
static lv_obj_t  *s_flash;     /* 进化闪光全屏矩形 */
static lv_obj_t  *s_sleepmask; /* 睡觉半透明遮罩 */
static esp_timer_handle_t s_timer;
static bool s_active;
static bool s_hatched;
static bool s_sleeping;
static uint8_t s_hatch_clicks;
static uint8_t s_cur_poke_idx;  /* 当前宝可梦在 pokemon_gifs[] 中的索引 */

/* 进化等级阈值（stage 0→1 需要 LV16，1→2 需要 LV32）*/
#define EVO_LV_STAGE1 16
#define EVO_LV_STAGE2 32

#define CLAMP(v) ((v) < 0 ? 0 : ((v) > 100 ? 100 : (v)))
#define EXP_PER_LEVEL 100

/* 刷新状态条 + 等级 */
static void refresh(void)
{
    static const uint32_t BAR_COLORS[4] = { 0xFF8A3D, 0xFFD928, 0x82BE2D, 0x4FC3F7 };
    uint8_t vals[4] = { s_stat.hunger, s_stat.happy, s_stat.energy, s_stat.clean };
    for (int i = 0; i < 4; i++) {
        lv_obj_set_width(s_bars[i], (int)(vals[i] * 80 / 100));
        lv_obj_set_style_bg_color(s_bars[i], lv_color_hex(BAR_COLORS[i]), 0);
        lv_label_set_text_fmt(s_vals[i], "%d", vals[i]);
    }
    if (s_lvlabel) lv_label_set_text_fmt(s_lvlabel, "Lv%d", s_stat.lv);
}

/* 进化闪光淡出回调 */
static void flash_fade_cb(lv_timer_t *tm)
{
    lv_obj_t *mask = (lv_obj_t *)tm->user_data;
    if (mask) lv_obj_add_flag(mask, LV_OBJ_FLAG_HIDDEN);
    lv_timer_delete(tm);
}

/* 经验增加，满则升级；到达阈值触发进化 */
static void try_evolve(void)
{
    const pokemon_gif_t *cur = &pokemon_gifs[s_cur_poke_idx];
    /* 检查是否有同 family 的下一阶段 */
    int next_idx = -1;
    for (int i = 0; i < pokemon_gif_count; i++) {
        if (i == s_cur_poke_idx) continue;
        if (pokemon_gifs[i].evo_family == cur->evo_family &&
            pokemon_gifs[i].evo_stage == cur->evo_stage + 1) {
            next_idx = i;
            break;
        }
    }
    if (next_idx < 0) return;  /* 已是最终形态 */

    /* 进化条件：等级达到阈值 */
    bool can_evolve = false;
    if (cur->evo_stage == 0 && s_stat.lv >= EVO_LV_STAGE1) can_evolve = true;
    if (cur->evo_stage == 1 && s_stat.lv >= EVO_LV_STAGE2) can_evolve = true;
    if (!can_evolve) return;

    /* 触发进化：全屏闪光 + 切换 GIF */
    s_cur_poke_idx = next_idx;
    if (s_flash) {
        lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(s_flash, LV_OPA_90, 0);
        lv_obj_remove_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
        lv_timer_create(flash_fade_cb, 400, s_flash);
    }
    /* 重新播放新形态 GIF */
    if (s_gif) gif_player_stop(s_gif);
    s_gif = gif_player_create(s_scr, 88, 156, 64, 64);
    gif_player_play(s_gif, pokemon_gifs[s_cur_poke_idx].data, pokemon_gifs[s_cur_poke_idx].len);
    if (s_namelabel) lv_label_set_text(s_namelabel, pokemon_gifs[s_cur_poke_idx].name);
}

/* 经验增加，满则升级 */
static void add_exp(uint16_t amount)
{
    s_stat.exp += amount;
    while (s_stat.exp >= EXP_PER_LEVEL) {
        s_stat.exp -= EXP_PER_LEVEL;
        s_stat.lv++;
        try_evolve();  /* 每次升级检查进化 */
    }
}

/* 每秒衰减 */
static void tick_cb(void *arg)
{
    (void)arg;
    if (!s_active || !s_hatched) return;
    s_stat.hunger = CLAMP(s_stat.hunger - 1);
    s_stat.happy  = CLAMP(s_stat.happy  - 1);
    s_stat.energy = CLAMP(s_stat.energy - 1);
    s_stat.clean  = CLAMP(s_stat.clean  - 1);
    /* 睡觉时能量 +2/秒，饥饿 -1/秒 */
    if (s_sleeping) {
        s_stat.energy = CLAMP(s_stat.energy + 2);
        s_stat.hunger = CLAMP(s_stat.hunger - 1);
    }
    if (bsp_lvgl_lock(100)) {
        refresh();
        bsp_lvgl_unlock();
    }
}

/* 破壳：随机选一只基础形态宝可梦，播放 GIF */
static void do_hatch(void)
{
    if (!s_ball) return;
    ui_pixel_ball_open(s_ball);
    s_ball = NULL;
    s_hatched = true;

    /* 随机选一只基础形态（stage=0）宝可梦 */
    int tries = 0;
    do {
        s_cur_poke_idx = rand() % pokemon_gif_count;
        tries++;
    } while (pokemon_gifs[s_cur_poke_idx].evo_stage != 0 && tries < 20);
    const pokemon_gif_t *pg = &pokemon_gifs[s_cur_poke_idx];

    /* 创建 GIF 播放器（64×64，居中）*/
    s_gif = gif_player_create(s_scr, 88, 156, 64, 64);
    gif_player_play(s_gif, pg->data, pg->len);

    /* 显示名字 */
    if (s_namelabel) lv_label_set_text(s_namelabel, pg->name);

    refresh();
}

/* ---------- demo 接口 ---------- */
void demo_pet_enter(void)
{
    srand(esp_random());
    s_stat.hunger = 80; s_stat.happy = 80;
    s_stat.energy = 80; s_stat.clean = 80;
    s_stat.lv = 1; s_stat.exp = 0;
    s_hatched = false; s_hatch_clicks = 0;
    s_active = true;

    s_scr = ui_pixel_screen_create("PET");

    /* 四条状态条 + 标签 */
    static const char *LABELS[4] = { "FOOD", "MOOD", "NRG", "CLEAN" };
    for (int i = 0; i < 4; i++) {
        int y = 56 + i * 22;
        lv_obj_t *lab = ui_pixel_label(s_scr, LABELS[i], &lv_font_montserrat_14, UI_INK);
        lv_obj_set_pos(lab, 12, y);
        lv_obj_t *bg = lv_obj_create(s_scr);
        lv_obj_remove_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(bg, 62, y + 2);
        lv_obj_set_size(bg, 84, 12);
        lv_obj_set_style_radius(bg, 0, 0);
        lv_obj_set_style_border_width(bg, 1, 0);
        lv_obj_set_style_border_color(bg, lv_color_hex(UI_INK), 0);
        lv_obj_set_style_pad_all(bg, 0, 0);
        lv_obj_set_style_bg_color(bg, lv_color_hex(UI_PAPER), 0);
        s_bars[i] = lv_obj_create(s_scr);
        lv_obj_remove_flag(s_bars[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(s_bars[i], 63, y + 3);
        lv_obj_set_size(s_bars[i], 80, 10);
        lv_obj_set_style_radius(s_bars[i], 0, 0);
        lv_obj_set_style_border_width(s_bars[i], 0, 0);
        lv_obj_set_style_pad_all(s_bars[i], 0, 0);
        s_vals[i] = ui_pixel_label(s_scr, "80", &lv_font_montserrat_14, UI_INK);
        lv_obj_set_pos(s_vals[i], 152, y);
    }

    /* 等级标签 */
    s_lvlabel = ui_pixel_label(s_scr, "Lv1", &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(s_lvlabel, 190, 56);

    /* 精灵球（屏幕居中），点击 3 次破壳 */
    s_ball = ui_pixel_ball_create(s_scr, 108, 148);

    /* 宠物名字标签（破壳后显示） */
    s_namelabel = ui_pixel_label(s_scr, "", &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(s_namelabel, 100, 250);

    /* 进化闪光层（全屏白色矩形，初始隐藏）*/
    s_flash = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_flash, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_flash, 0, 0);
    lv_obj_set_size(s_flash, 240, 320);
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_0, 0);
    lv_obj_set_style_border_width(s_flash, 0, 0);
    lv_obj_set_style_pad_all(s_flash, 0, 0);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);

    /* 睡觉半透明遮罩（覆盖 GIF 区域，初始隐藏）*/
    s_sleepmask = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_sleepmask, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_sleepmask, 88, 156);
    lv_obj_set_size(s_sleepmask, 64, 64);
    lv_obj_set_style_bg_color(s_sleepmask, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_sleepmask, LV_OPA_40, 0);
    lv_obj_set_style_border_width(s_sleepmask, 0, 0);
    lv_obj_set_style_pad_all(s_sleepmask, 0, 0);
    lv_obj_add_flag(s_sleepmask, LV_OBJ_FLAG_HIDDEN);

    /* 操作提示 */
    lv_obj_t *tip = ui_pixel_label(s_scr,
        "UP:FEED  DOWN:PLAY  OK:SLEEP",
        &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(tip, 16, 300);
    lv_obj_t *tip2 = ui_pixel_label(s_scr,
        "LONG UP:WASH  LONG DOWN:TRAIN",
        &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(tip2, 16, 316);

    lv_screen_load(s_scr);

    esp_timer_create_args_t args = {
        .callback = tick_cb,
        .name = "pet_tick",
    };
    esp_timer_create(&args, &s_timer);
    esp_timer_start_periodic(s_timer, 1000000);
}

void demo_pet_exit(void)
{
    s_active = false;
    s_sleeping = false;
    if (s_timer) { esp_timer_stop(s_timer); esp_timer_delete(s_timer); s_timer = NULL; }
    if (s_scr)  { lv_obj_delete(s_scr); s_scr = NULL; s_gif = NULL; s_ball = NULL; s_flash = NULL; s_sleepmask = NULL; }
}

void demo_pet_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    /* 未破壳：任意按键累计破壳进度 */
    if (!s_hatched) {
        if (ev != BSP_BTN_CLICK) return;
        s_hatch_clicks++;
        if (s_ball) ui_pixel_ball_shake(s_ball, s_hatch_clicks <= 3 ? s_hatch_clicks : 3);
        if (s_hatch_clicks >= 3) do_hatch();
        return;
    }

    if (ev == BSP_BTN_CLICK) {
        if (btn == BSP_BTN_UP) {
            /* 喂食 */
            s_stat.hunger = CLAMP(s_stat.hunger + 25);
            s_stat.clean  = CLAMP(s_stat.clean  - 5);
        } else if (btn == BSP_BTN_DOWN) {
            /* 玩耍 */
            s_stat.happy  = CLAMP(s_stat.happy  + 20);
            s_stat.energy = CLAMP(s_stat.energy - 12);
            s_stat.hunger = CLAMP(s_stat.hunger - 5);
        } else if (btn == BSP_BTN_OK) {
            /* 睡觉/唤醒切换 */
            s_sleeping = !s_sleeping;
            if (s_sleeping) {
                s_stat.energy = CLAMP(s_stat.energy + 30);
                s_stat.hunger = CLAMP(s_stat.hunger - 3);
                if (s_sleepmask) lv_obj_remove_flag(s_sleepmask, LV_OBJ_FLAG_HIDDEN);
            } else {
                if (s_sleepmask) lv_obj_add_flag(s_sleepmask, LV_OBJ_FLAG_HIDDEN);
            }
        }
    } else if (ev == BSP_BTN_LONG) {
        if (btn == BSP_BTN_UP) {
            /* 清洁 */
            s_stat.clean  = CLAMP(s_stat.clean  + 30);
            s_stat.energy = CLAMP(s_stat.energy - 3);
        } else if (btn == BSP_BTN_DOWN) {
            /* 训练：得经验 */
            s_stat.energy = CLAMP(s_stat.energy - 15);
            s_stat.hunger = CLAMP(s_stat.hunger - 8);
            add_exp(25);
        }
    }
    refresh();
}
