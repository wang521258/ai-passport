// main/demo_pet.c —— 电子宠物演示页（UI 优化版 + 英语训练）。
// 精灵球破壳 → 随机宝可梦 GIF 动画；四项状态随时间衰减；
// 3 按键映射 5 动作：上=喂食 下=玩耍 中=睡觉 上长按=清洁 下长按=训练；
// 训练 = 英语单词选词（反向学习）：显示英文，选中文释义。
#include "demo.h"
#include "bsp_display.h"
#include "ui_pixel.h"
#include "ui_pet.h"
#include "gif_player.h"
#include "pokemon_sprites.h"
#include "word_pool.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
static lv_obj_t  *s_stat_bg;    /* 顶部状态条半透明背景 */
static lv_obj_t  *s_stat_icons[4]; /* 状态图标块 */
static lv_obj_t  *s_stat_vals[4];   /* 状态数字 */
static lv_obj_t  *s_lvlabel;
static lv_obj_t  *s_flash;     /* 进化闪光全屏矩形 */
static lv_obj_t  *s_sleepmask; /* 睡觉半透明遮罩 */
static esp_timer_handle_t s_timer;
static bool s_active;
static bool s_hatched;
static bool s_sleeping;
static uint8_t s_hatch_clicks;
static uint8_t s_cur_poke_idx;  /* 当前宝可梦在 pokemon_gifs[] 中的索引 */

/* ---------- 训练模式（英语单词选词） ---------- */
static bool s_training;          /* 是否在训练模式 */
static int  s_train_word_idx;    /* 当前题目的正确单词索引 */
static int  s_train_option_idx;  /* 当前选中的选项（0-3） */
static int  s_train_correct;     /* 正确选项的位置 */
static lv_obj_t *s_train_panel;  /* 训练面板 */
static lv_obj_t *s_train_word;   /* 英文单词标签 */
static lv_obj_t *s_train_opts[4]; /* 4 个中文选项 */
static lv_obj_t *s_train_cursor;  /* 选中指示器 */

/* 进化等级阈值（stage 0→1 需要 LV16，1→2 需要 LV32）*/
#define EVO_LV_STAGE1 16
#define EVO_LV_STAGE2 32

#define CLAMP(v) ((v) < 0 ? 0 : ((v) > 100 ? 100 : (v)))
#define EXP_PER_LEVEL 100

/* ---------- 背景绘制：天空 + 草地 ----------
 * 注意：block() 每调用一次就创建一个 LVGL 对象。ESP32-C3 只有约 80 KB 动态 RAM，
 * 早期版本用 240 个小色块拼背景（天空 50 + 草地 30 + 草点 40 + 小山 120），
 * 进页面时一次性创建近 300 个对象导致内存耗尽、空指针崩溃重启。
 * 现改为 13 个大色块，保留渐变层次与山体轮廓，玩法逻辑完全不变。
 */
static void draw_background(lv_obj_t *parent)
{
    /* 天空：3 段渐变（原 50 个 4px 条带 → 3 块） */
    block(parent, 0, 0,   240, 80, 0x1689E8);   /* 顶部深蓝 */
    block(parent, 0, 80,  240, 60, 0x4FA0E8);   /* 中部蓝 */
    block(parent, 0, 140, 240, 60, 0x7BB8F0);   /* 近地平线浅蓝 */

    /* 远处小山：3 块近似起伏轮廓（原 120 个 2px 竖条 → 3 块） */
    block(parent, 0,   148, 80, 52, 0x5A9A4A);
    block(parent, 80,  138, 80, 62, 0x5A9A4A);
    block(parent, 160, 148, 80, 52, 0x5A9A4A);

    /* 草地：2 段渐变（原 30 个 4px 条带 → 2 块） */
    block(parent, 0, 200, 240, 80, 0x82BE2D);   /* 近处亮绿 */
    block(parent, 0, 280, 240, 40, 0x5A9020);   /* 底部暗绿 */

    /* 草地纹理：5 簇小草（原 40 个随机点 → 5 个固定点） */
    const int grass[5][2] = { {20, 248}, {70, 272}, {130, 258}, {186, 288}, {214, 238} };
    for (int i = 0; i < 5; i++) {
        block(parent, grass[i][0], grass[i][1], 3, 4, 0x4A8A18);
    }
}

/* ---------- 顶部状态条（GBA 风格图标 + 数字） ---------- */
static const uint32_t STAT_COLORS[4] = { 0xFF8A3D, 0xFFD928, 0x82BE2D, 0x4FC3F7 };
static const char *STAT_LABELS[4] = { "FOOD", "MOOD", "NRG", "WASH" };

static void build_status_bar(void)
{
    /* 半透明背景条 */
    s_stat_bg = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_stat_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_stat_bg, 0, 0);
    lv_obj_set_size(s_stat_bg, 240, 24);
    lv_obj_set_style_bg_color(s_stat_bg, lv_color_hex(0x17202A), 0);
    lv_obj_set_style_bg_opa(s_stat_bg, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_stat_bg, 0, 0);
    lv_obj_set_style_pad_all(s_stat_bg, 0, 0);
    lv_obj_set_style_radius(s_stat_bg, 0, 0);

    /* 4 个状态块：图标 + 数字，水平排列 */
    for (int i = 0; i < 4; i++) {
        int x = 4 + i * 56;
        /* 颜色块（图标占位） */
        s_stat_icons[i] = lv_obj_create(s_stat_bg);
        lv_obj_remove_flag(s_stat_icons[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(s_stat_icons[i], x, 4);
        lv_obj_set_size(s_stat_icons[i], 16, 16);
        lv_obj_set_style_bg_color(s_stat_icons[i], lv_color_hex(STAT_COLORS[i]), 0);
        lv_obj_set_style_bg_opa(s_stat_icons[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_stat_icons[i], 1, 0);
        lv_obj_set_style_border_color(s_stat_icons[i], lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_pad_all(s_stat_icons[i], 0, 0);
        lv_obj_set_style_radius(s_stat_icons[i], 0, 0);
        /* 数字 */
        s_stat_vals[i] = ui_pixel_label(s_stat_bg, "80", &lv_font_montserrat_14, 0xFFFFFF);
        lv_obj_set_pos(s_stat_vals[i], x + 20, 4);
    }
    /* 等级标签（右上角） */
    s_lvlabel = ui_pixel_label(s_stat_bg, "Lv1", &lv_font_montserrat_14, 0xFFD928);
    lv_obj_set_pos(s_lvlabel, 200, 4);
}

/* 刷新状态条 + 等级 */
static void refresh(void)
{
    uint8_t vals[4] = { s_stat.hunger, s_stat.happy, s_stat.energy, s_stat.clean };
    for (int i = 0; i < 4; i++) {
        lv_label_set_text_fmt(s_stat_vals[i], "%d", vals[i]);
    }
    if (s_lvlabel) lv_label_set_text_fmt(s_lvlabel, "Lv%d", s_stat.lv);
}

/* 进化闪光淡出回调 */
static void flash_fade_cb(lv_timer_t *tm)
{
    lv_obj_t *mask = (lv_obj_t *)lv_timer_get_user_data(tm);
    if (mask) lv_obj_add_flag(mask, LV_OBJ_FLAG_HIDDEN);
    lv_timer_delete(tm);
}

/* 经验增加，满则升级；到达阈值触发进化 */
static void try_evolve(void)
{
    const pokemon_gif_t *cur = &pokemon_gifs[s_cur_poke_idx];
    int next_idx = -1;
    for (int i = 0; i < pokemon_gif_count; i++) {
        if (i == s_cur_poke_idx) continue;
        if (pokemon_gifs[i].evo_family == cur->evo_family &&
            pokemon_gifs[i].evo_stage == cur->evo_stage + 1) {
            next_idx = i;
            break;
        }
    }
    if (next_idx < 0) return;

    bool can_evolve = false;
    if (cur->evo_stage == 0 && s_stat.lv >= EVO_LV_STAGE1) can_evolve = true;
    if (cur->evo_stage == 1 && s_stat.lv >= EVO_LV_STAGE2) can_evolve = true;
    if (!can_evolve) return;

    s_cur_poke_idx = next_idx;
    if (s_flash) {
        lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(s_flash, LV_OPA_90, 0);
        lv_obj_remove_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
        lv_timer_create(flash_fade_cb, 400, s_flash);
    }
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
        try_evolve();
    }
}

/* 每秒衰减 */
static void tick_cb(void *arg)
{
    (void)arg;
    if (!s_active || !s_hatched || s_training) return;
    s_stat.hunger = CLAMP(s_stat.hunger - 1);
    s_stat.happy  = CLAMP(s_stat.happy  - 1);
    s_stat.energy = CLAMP(s_stat.energy - 1);
    s_stat.clean  = CLAMP(s_stat.clean  - 1);
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
    ESP_LOGI("PET", "hatch: free=%d", (int)esp_get_free_heap_size());
    if (!s_ball) return;
    ui_pixel_ball_open(s_ball);
    ESP_LOGI("PET", "ball_open ok free=%d", (int)esp_get_free_heap_size());
    s_ball = NULL;
    s_hatched = true;

    int tries = 0;
    do {
        s_cur_poke_idx = rand() % pokemon_gif_count;
        tries++;
    } while (pokemon_gifs[s_cur_poke_idx].evo_stage != 0 && tries < 20);
    const pokemon_gif_t *pg = &pokemon_gifs[s_cur_poke_idx];
    ESP_LOGI("PET", "pick=%s len=%d free=%d", pg->name, pg->len, (int)esp_get_free_heap_size());

    s_gif = gif_player_create(s_scr, 88, 156, 64, 64);
    ESP_LOGI("PET", "gif_create=%p free=%d", s_gif, (int)esp_get_free_heap_size());
    gif_player_play(s_gif, pg->data, pg->len);
    ESP_LOGI("PET", "gif_play done free=%d", (int)esp_get_free_heap_size());
    if (s_namelabel) lv_label_set_text(s_namelabel, pg->name);
    ESP_LOGI("PET", "hatch ok free=%d", (int)esp_get_free_heap_size());

    refresh();
}

/* ---------- 训练模式：英语单词选词 ---------- */
static void train_start(void);
static void train_show_question(void);
static void train_answer(int option);
static void train_exit(void);

static void train_start(void)
{
    s_training = true;
    /* 隐藏 GIF 和名字 */
    if (s_gif) lv_obj_add_flag(s_gif, LV_OBJ_FLAG_HIDDEN);
    if (s_namelabel) lv_obj_add_flag(s_namelabel, LV_OBJ_FLAG_HIDDEN);

    /* 创建训练面板 */
    s_train_panel = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_train_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_train_panel, 20, 100);
    lv_obj_set_size(s_train_panel, 200, 180);
    lv_obj_set_style_bg_color(s_train_panel, lv_color_hex(0xF4F4EA), 0);
    lv_obj_set_style_bg_opa(s_train_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(s_train_panel, 2, 0);
    lv_obj_set_style_border_color(s_train_panel, lv_color_hex(0x17202A), 0);
    lv_obj_set_style_pad_all(s_train_panel, 8, 0);
    lv_obj_set_style_radius(s_train_panel, 0, 0);

    /* 英文单词标签 */
    s_train_word = ui_pixel_label(s_train_panel, "", &lv_font_montserrat_20, 0x17202A);
    lv_obj_set_pos(s_train_word, 10, 10);

    /* 4 个中文选项 */
    for (int i = 0; i < 4; i++) {
        s_train_opts[i] = lv_obj_create(s_train_panel);
        lv_obj_remove_flag(s_train_opts[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(s_train_opts[i], 10, 40 + i * 28);
        lv_obj_set_size(s_train_opts[i], 170, 24);
        lv_obj_set_style_bg_color(s_train_opts[i], lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(s_train_opts[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_train_opts[i], 1, 0);
        lv_obj_set_style_border_color(s_train_opts[i], lv_color_hex(0x888888), 0);
        lv_obj_set_style_pad_all(s_train_opts[i], 2, 0);
        lv_obj_set_style_radius(s_train_opts[i], 0, 0);
    }

    /* 选中指示器（>符号） */
    s_train_cursor = ui_pixel_label(s_train_panel, ">", &lv_font_montserrat_14, 0xFF0000);
    lv_obj_set_pos(s_train_cursor, 0, 44);

    train_show_question();
}

static void train_show_question(void)
{
    /* 随机选一个单词作为正确答案 */
    s_train_word_idx = rand() % WORD_POOL_SIZE;
    /* 随机选 3 个干扰项 */
    int wrong_indices[3];
    for (int i = 0; i < 3; i++) {
        int idx;
        do {
            idx = rand() % WORD_POOL_SIZE;
        } while (idx == s_train_word_idx);
        wrong_indices[i] = idx;
    }

    /* 随机决定正确答案的位置（0-3） */
    s_train_correct = rand() % 4;
    s_train_option_idx = 0;

    /* 设置英文单词 */
    lv_label_set_text(s_train_word, word_pool[s_train_word_idx].en);

    /* 设置 4 个选项 */
    int wrong_pos = 0;
    for (int i = 0; i < 4; i++) {
        if (i == s_train_correct) {
            lv_label_set_text(s_train_opts[i], word_pool[s_train_word_idx].cn);
        } else {
            lv_label_set_text(s_train_opts[i], word_pool[wrong_indices[wrong_pos]].cn);
            wrong_pos++;
        }
    }

    /* 移动光标到第一个选项 */
    lv_obj_set_pos(s_train_cursor, 0, 44);
}

static void train_answer(int option)
{
    if (option == s_train_correct) {
        /* 答对：经验+25，心情+10 */
        add_exp(25);
        s_stat.happy = CLAMP(s_stat.happy + 10);
        /* 显示下一题 */
        train_show_question();
    } else {
        /* 答错：经验+5，心情-5 */
        add_exp(5);
        s_stat.happy = CLAMP((int)s_stat.happy - 5);
        /* 标记错误选项（红色边框） */
        lv_obj_set_style_border_color(s_train_opts[option], lv_color_hex(0xFF0000), 0);
        /* 0.5 秒后恢复并出下一题 */
        /* 简化：直接出下一题 */
        train_show_question();
    }
}

static void train_exit(void)
{
    s_training = false;
    if (s_train_panel) {
        lv_obj_delete(s_train_panel);
        s_train_panel = NULL;
    }
    /* 恢复显示 GIF 和名字 */
    if (s_gif) lv_obj_remove_flag(s_gif, LV_OBJ_FLAG_HIDDEN);
    if (s_namelabel) lv_obj_remove_flag(s_namelabel, LV_OBJ_FLAG_HIDDEN);
    /* 训练消耗精力 + 饥饿 */
    s_stat.energy = CLAMP((int)s_stat.energy - 15);
    s_stat.hunger = CLAMP((int)s_stat.hunger - 8);
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
    s_training = false;

    s_scr = ui_pixel_screen_create("PET");

    /* 背景：天空 + 草地 */
    draw_background(s_scr);

    /* 顶部状态条 */
    build_status_bar();

    /* 精灵球（屏幕居中），点击 3 次破壳 */
    s_ball = ui_pixel_ball_create(s_scr, 108, 148);

    /* 宠物名字标签（破壳后显示） */
    s_namelabel = ui_pixel_label(s_scr, "", &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(s_namelabel, 100, 230);
    lv_obj_set_style_bg_color(s_namelabel, lv_color_hex(0xF4F4EA), 0);
    lv_obj_set_style_bg_opa(s_namelabel, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(s_namelabel, 2, 0);

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

    /* 操作提示（底部简化） */
    lv_obj_t *tip = ui_pixel_label(s_scr,
        "UP:FEED  DOWN:PLAY  OK:SLEEP",
        &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(tip, 16, 268);
    lv_obj_t *tip2 = ui_pixel_label(s_scr,
        "LONG UP:WASH  LONG DOWN:TRAIN",
        &lv_font_montserrat_14, UI_INK);
    lv_obj_set_pos(tip2, 10, 286);

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
    s_training = false;
    if (s_timer) { esp_timer_stop(s_timer); esp_timer_delete(s_timer); s_timer = NULL; }
    gif_player_destroy();     /* 释放单例解码器（约 24KB），否则退出后一直占着堆 */
    if (s_scr)  { lv_obj_delete(s_scr); s_scr = NULL; s_gif = NULL; s_ball = NULL; s_flash = NULL; s_sleepmask = NULL; s_train_panel = NULL; }
}

void demo_pet_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    /* 训练模式下：上下选选项，OK 确认，长按 OK 退出 */
    if (s_training) {
        if (ev == BSP_BTN_CLICK) {
            if (btn == BSP_BTN_UP) {
                s_train_option_idx = (s_train_option_idx - 1 + 4) % 4;
                lv_obj_set_pos(s_train_cursor, 0, 44 + s_train_option_idx * 28);
            } else if (btn == BSP_BTN_DOWN) {
                s_train_option_idx = (s_train_option_idx + 1) % 4;
                lv_obj_set_pos(s_train_cursor, 0, 44 + s_train_option_idx * 28);
            } else if (btn == BSP_BTN_OK) {
                train_answer(s_train_option_idx);
            }
        } else if (ev == BSP_BTN_LONG && btn == BSP_BTN_OK) {
            train_exit();
        }
        return;
    }

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
            /* 训练：进入英语单词选词模式 */
            train_start();
        }
    }
    refresh();
}
