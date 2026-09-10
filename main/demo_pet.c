// main/demo_pet.c —— 宝可梦电子宠物（v12 定版）
//
// 布局（240x320）：
//   y=0..28     顶部栏：4 按钮 [训练][睡觉][温习][重修] + 右侧 攻+词数框
//   y=28..320   主舞台：蛋 / 宠物 / 全屏训练面板（绿豆色）
//
// v12 定版特性（与 HTML 预览 pet_preview.html 一致）：
//   - 词库：入门档 335 词（GitHub 小学高频+PEP1-2）+ PEP 3-6 年级 8 册 817 词 = 1152 词
//     按学习进度加权滑动出题，新词在最前面（简单词先学）
//   - 题型：英选汉 / 汉选英 各 50% 随机
//   - 等级 = 学会的词数（右上角 攻+数字），进化阈值 210 / 915 词
//   - 进化：严格同家族逐档升级，绝不跨种越级
//   - 遗忘：记忆度每小时 -1（约 4 天忘光），归 0 掉进错题本，攻数值跟着掉并闪红
//   - 破壳：只从"有进化链的基础形态"随机（15 只）
//   - 背景：纯色绿豆 #C8DFA0，无分段
//   - 睡觉：黑幕从顶部三段拉下（噔噔噔）→ 全黑+Zzz；醒来黑幕向下撤、上方先亮
//   - 睡觉锁键：只有 OK 能唤醒，其余键全吞
//   - 重修 = 回蛋（学习记录保留）

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
#include "esp_log.h"
#include "esp_random.h"
#include <string.h>
#include <stdlib.h>

/* 中文字库：由 tools/gen_font.py 从系统字体子集化生成（1142 汉字 + ASCII，约 88KB）。
   LVGL 9.5 已移除 lv_font_simsun_16_cjk，内置的思源黑体子集又只含 1187 个 CJK 字符
   （本项目要 1142 汉字，实测缺 722 个），所以自己生成一份 100% 覆盖的。 */
extern lv_font_t cn_16;

#define TAG "PET"
#define LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)

#define CLAMP(v) ((v) < 0 ? 0 : ((v) > 100 ? 100 : (v)))

#define PET_SIZE 112
#define PET_X    ((240 - PET_SIZE) / 2)   /* 64 */
#define PET_Y    118
#define EGG_X    ((240 - 72) / 2)         /* 84 */
#define EGG_Y    129

#define MAX_WRONG    120                /* 错题本容量（遗忘会灌入，比 v4 大） */
#define MEM_FULL     100                /* 学会时记忆度满分 */
#define FORGET_SEC   3600               /* 每 3600 秒记忆度 -1 → 约 4 天忘光 */
#define EVO_KNOWN_1  210                /* stage0→1：学会 210 词（1152 词库等比自 150）*/
#define EVO_KNOWN_2  915                /* stage1→2：学会 915 词（1152 词库等比自 650）*/

#define C_BG       0xC8DFA0             /* 绿豆色（场景 + 面板） */
#define C_BG565    0xC6F4               /* 同色的 RGB565 */
#define C_ROW      0xEDF5DC             /* 选项行底 */
#define C_ROWBRD   0xA5C97A             /* 选项行描边 */
#define C_SELBRD   0x33691E             /* 选中描边 */
#define C_INK      0x1B3A0F             /* 面板文字 */
#define C_ATK      0xFFB74D             /* 攻图标色 */
#define C_ATKNUM   0xFFCC80             /* 攻数字色 */

typedef enum { MODE_EGG, MODE_HOME, MODE_TRAIN, MODE_REVIEW } pet_mode_t;
typedef enum { MENU_TRAIN = 0, MENU_SLEEP, MENU_REVIEW, MENU_RESET } pet_menu_t;

static struct {
    uint8_t hunger, happy, energy;
} s_stat = { 80, 80, 80 };

static pet_mode_t s_mode = MODE_EGG;
static pet_menu_t s_menu = MENU_TRAIN;
static uint8_t  s_opt;                    /* 训练/温习选项 0..4（4=返回） */
static uint8_t  s_hatch_clicks;
static uint8_t  s_cur_poke_idx;
static uint8_t  s_tick_div;
static bool     s_sleeping;
static bool     s_active;
static int      s_wrong_book[MAX_WRONG];
static int      s_wrong_n;

/* 学习状态：每词记忆度 0..100，>0 即"已学会" */
static uint8_t  s_mem[WORD_POOL_SIZE];

/* 破壳候选：有进化链的基础形态下标 */
static int      s_bases[64];
static int      s_base_n;

/* 当前题 */
static int  s_qWord = -1;
static int  s_qOpts[4];
static int  s_qCorrect;
static uint8_t s_qDir;                    /* 0=英选汉 1=汉选英 */

/* 攻数值告警（遗忘掉词时闪红几次） */
static int  s_atk_alert_blinks;

/* LVGL 对象 */
static lv_obj_t *s_scr;
static lv_obj_t *s_egg;
static lv_obj_t *s_gif;
static lv_obj_t *s_topbar_bg;
static lv_obj_t *s_menu_btns[4];
static lv_obj_t *s_atkbox;
static lv_obj_t *s_atk_num;
static lv_obj_t *s_night;                 /* 睡觉黑幕 */
static lv_obj_t *s_zzz;
static lv_obj_t *s_flash;
static lv_obj_t *s_panel;
static lv_obj_t *s_p_word;
static lv_obj_t *s_p_opts[5];             /* 行容器 */
static lv_obj_t *s_p_txt[5];              /* 行内文字 */
static lv_obj_t *s_p_cursor[5];

static lv_timer_t *s_blink_t;
static lv_timer_t *s_curtain_t;           /* 黑幕动画定时器（非 NULL 时表示动画中） */
static esp_timer_handle_t s_tick_timer;
static uint32_t s_forget_cnt;

static void try_evolve(void);             /* 前向声明（answer 先于定义调用） */

/* ============================================================
 *  工具
 * ============================================================ */
static int known_count(void)
{
    int n = 0;
    for (int i = 0; i < WORD_POOL_SIZE; i++) if (s_mem[i] > 0) n++;
    return n;
}

static uint16_t bg_color_at(int x, int y)
{
    (void)x; (void)y;
    return C_BG565;
}

/* ============================================================
 *  背景：纯色绿豆，一色到底
 * ============================================================ */
static void draw_background(lv_obj_t *parent)
{
    block(parent, 0, 0, 240, 320, C_BG);
}

/* ============================================================
 *  顶部按钮栏（4 按钮 + 攻+词数框）
 * ============================================================ */
static void build_topbar(void)
{
    s_topbar_bg = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_topbar_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_topbar_bg, 0, 0);
    lv_obj_set_size(s_topbar_bg, 240, 28);
    lv_obj_set_style_bg_color(s_topbar_bg, lv_color_hex(0x17202A), 0);
    lv_obj_set_style_bg_opa(s_topbar_bg, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_topbar_bg, 0, 0);
    lv_obj_set_style_pad_all(s_topbar_bg, 0, 0);
    lv_obj_set_style_radius(s_topbar_bg, 0, 0);

    static const char *LBL[4] = { "训练", "睡觉", "温习", "重修" };
    for (int i = 0; i < 4; i++) {
        lv_obj_t *b = lv_obj_create(s_topbar_bg);
        lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(b, 1 + i * 47, 1);
        lv_obj_set_size(b, 46, 26);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x37474F), 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(0x546E7A), 0);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_radius(b, 0, 0);
        lv_obj_set_style_pad_all(b, 0, 0);

        lv_obj_t *lb = lv_label_create(b);
        lv_label_set_text(lb, LBL[i]);
        lv_obj_set_style_text_font(lb, &cn_16, 0);
        lv_obj_set_style_text_color(lb, lv_color_hex(0xECEFF1), 0);
        lv_obj_center(lb);
        s_menu_btns[i] = b;
    }

    /* 温习按钮只显示"温习"，不挂数量角标（角标会把按钮撑变形） */

    /* 攻框：攻 图标 + 学会词数（右半部分） */
    s_atkbox = lv_obj_create(s_topbar_bg);
    lv_obj_remove_flag(s_atkbox, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_atkbox, 189, 1);
    lv_obj_set_size(s_atkbox, 50, 26);
    lv_obj_set_style_bg_color(s_atkbox, lv_color_hex(0x17202A), 0);
    lv_obj_set_style_bg_opa(s_atkbox, LV_OPA_90, 0);
    lv_obj_set_style_border_color(s_atkbox, lv_color_hex(C_ATK), 0);
    lv_obj_set_style_border_width(s_atkbox, 1, 0);
    lv_obj_set_style_radius(s_atkbox, 0, 0);
    lv_obj_set_style_pad_all(s_atkbox, 0, 0);

    lv_obj_t *ic = lv_label_create(s_atkbox);
    lv_label_set_text(ic, "攻");
    lv_obj_set_style_text_font(ic, &cn_16, 0);
    lv_obj_set_style_text_color(ic, lv_color_hex(C_ATK), 0);
    lv_obj_set_pos(ic, 2, 4);

    s_atk_num = lv_label_create(s_atkbox);
    lv_label_set_text(s_atk_num, "0");
    lv_obj_set_style_text_font(s_atk_num, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_atk_num, lv_color_hex(C_ATKNUM), 0);
    lv_obj_set_pos(s_atk_num, 20, 6);
}

/* 闪烁定时器：按钮配色 / 告警 / 角标 / 攻数值 集中刷新 */
static void blink_timer_cb(lv_timer_t *t)
{
    (void)t;
    static bool on = false;
    on = !on;

    if (s_mode != MODE_HOME) {
        for (int i = 0; i < 4; i++) {
            lv_obj_set_style_bg_color(s_menu_btns[i], lv_color_hex(0x37474F), 0);
            lv_obj_set_style_border_color(s_menu_btns[i], lv_color_hex(0x546E7A), 0);
        }
        return;
    }

    int warn_idx = -1;
    if      (s_stat.hunger < 30)  warn_idx = MENU_TRAIN;
    else if (s_stat.energy < 30)  warn_idx = MENU_SLEEP;
    else if (s_wrong_n > 0)       warn_idx = MENU_REVIEW;

    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_bg_color(s_menu_btns[i], lv_color_hex(0x37474F), 0);
        lv_obj_set_style_border_color(s_menu_btns[i], lv_color_hex(0x546E7A), 0);
        lv_obj_set_style_text_color(s_menu_btns[i], lv_color_hex(0xECEFF1), 0);
    }
    lv_obj_set_style_bg_color(s_menu_btns[s_menu], lv_color_hex(0xFFD928), 0);
    lv_obj_set_style_border_color(s_menu_btns[s_menu], lv_color_hex(0x17202A), 0);
    lv_obj_set_style_text_color(s_menu_btns[s_menu], lv_color_hex(0x17202A), 0);

    if (warn_idx >= 0 && warn_idx != (int)s_menu) {
        if (on) {
            lv_obj_set_style_bg_color(s_menu_btns[warn_idx], lv_color_hex(0xE53935), 0);
            lv_obj_set_style_border_color(s_menu_btns[warn_idx], lv_color_hex(0xB71C1C), 0);
            lv_obj_set_style_text_color(s_menu_btns[warn_idx], lv_color_hex(0xFFFFFF), 0);
        } else {
            lv_obj_set_style_bg_color(s_menu_btns[warn_idx], lv_color_hex(0x8E0000), 0);
            lv_obj_set_style_border_color(s_menu_btns[warn_idx], lv_color_hex(0x8E0000), 0);
            lv_obj_set_style_text_color(s_menu_btns[warn_idx], lv_color_hex(0xFFB0B0), 0);
        }
    }

    /* 有词待温习时由 blink 定时器把"温习"按钮整体闪红，不再显示数量 */

    /* 攻数值 + 掉词告警闪红 */
    if (s_atk_num) lv_label_set_text_fmt(s_atk_num, "%d", known_count());
    if (s_atkbox) {
        if (s_atk_alert_blinks > 0) {
            s_atk_alert_blinks--;
            lv_obj_set_style_bg_color(s_atkbox, lv_color_hex(on ? 0xB71C1C : 0x17202A), 0);
            lv_obj_set_style_bg_opa(s_atkbox, LV_OPA_90, 0);
        } else {
            lv_obj_set_style_bg_color(s_atkbox, lv_color_hex(0x17202A), 0);
            lv_obj_set_style_bg_opa(s_atkbox, LV_OPA_90, 0);
        }
    }
}

/* ============================================================
 *  训练面板（全屏绿豆，无标题无提示文字）
 * ============================================================ */
static void render_panel(void)
{
    /* 题干：e2c 显英文 / c2e 显中文；音标不显示（Montserrat 无 IPA 字形会出方块） */
    lv_label_set_text(s_p_word, s_qDir ? word_pool[s_qWord].cn : word_pool[s_qWord].en);
    lv_obj_set_style_text_font(s_p_word,
        s_qDir ? &cn_16 : &lv_font_montserrat_20, 0);
    /* 中文题干最长 11 字（"舞者，舞蹈演员，舞蹈家"），16px CJK 在 240 宽内可一行放下 */
    lv_label_set_long_mode(s_p_word, LV_LABEL_LONG_WRAP);  /* 极端情况下允许换行 */
    lv_obj_set_height(s_p_word, 32);                       /* 压缩题区给选项让位 */

    for (int i = 0; i < 5; i++) {
        const char *text;
        if (i < 4) {
            text = s_qDir ? word_pool[s_qOpts[i]].en : word_pool[s_qOpts[i]].cn;
            lv_obj_set_style_text_font(s_p_txt[i],
                s_qDir ? &lv_font_montserrat_20 : &cn_16, 0);
        } else {
            text = (s_mode == MODE_REVIEW) ? "结束温习，返回" : "结束训练，返回";
            lv_obj_set_style_text_font(s_p_txt[i], &cn_16, 0);
        }
        lv_label_set_text(s_p_txt[i], text);

        if (i == (int)s_opt) {
            lv_obj_set_style_border_color(s_p_opts[i], lv_color_hex(C_SELBRD), 0);
            lv_obj_set_style_border_width(s_p_opts[i], 3, 0);
            lv_obj_set_style_bg_color(s_p_opts[i], lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_opa(s_p_cursor[i], LV_OPA_COVER, 0);
        } else {
            lv_obj_set_style_border_color(s_p_opts[i], lv_color_hex(C_ROWBRD), 0);
            lv_obj_set_style_border_width(s_p_opts[i], 2, 0);
            lv_obj_set_style_bg_color(s_p_opts[i], lv_color_hex(C_ROW), 0);
            lv_obj_set_style_opa(s_p_cursor[i], LV_OPA_TRANSP, 0);
        }
    }
}

/* ============================================================
 *  出题：v7 权重滑动随机（起点三年级，随进度滑向高年级）
 * ============================================================ */
static float focus_book(void)
{
    return (float)known_count() / (float)WORD_POOL_SIZE * 8.0f;   /* 0~8：入门档→六下 */
}
static float book_weight(int book, float focus)
{
    float d = (float)book - focus;
    if (d < 0) d = -d;
    return 1.0f / (1.0f + d * 0.9f);
}
static float mem_weight(int idx)
{
    uint8_t m = s_mem[idx];
    if (m == 0)  return 1.0f;    /* 没学过：正常权重 */
    if (m <= 40) return 1.8f;    /* 快忘了：加重复习 */
    return 0.35f;                /* 记得牢：少出 */
}

static int pick_train_word(void)
{
    float focus = focus_book();
    float bw[9], tot = 0;
    for (int b = 0; b < 9; b++) { bw[b] = book_weight(b, focus); tot += bw[b]; }
    float r = (float)(rand() % 10000) / 10000.0f * tot;
    int book = 8;
    for (int b = 0; b < 9; b++) { r -= bw[b]; if (r <= 0) { book = b; break; } }

    /* 在该册内按记忆度权重抽词（先数该册词数） */
    float wtot = 0;
    for (int i = 0; i < WORD_POOL_SIZE; i++)
        if (word_pool[i].book == book) wtot += mem_weight(i);
    if (wtot <= 0) return rand() % WORD_POOL_SIZE;
    float r2 = (float)(rand() % 10000) / 10000.0f * wtot;
    for (int i = 0; i < WORD_POOL_SIZE; i++) {
        if (word_pool[i].book != book) continue;
        r2 -= mem_weight(i);
        if (r2 <= 0) return i;
    }
    return rand() % WORD_POOL_SIZE;
}

/* 义项分隔符：ASCII 逗号/分号 + 全角"，；、"（UTF-8 各占 3 字节） */
static int sep_len(const char *p)
{
    unsigned char c = (unsigned char)p[0];
    if (c == ',' || c == ';') return 1;
    if (c == 0xEF && (unsigned char)p[1] == 0xBC) {
        if ((unsigned char)p[2] == 0x8C) return 3;   /* ， */
        if ((unsigned char)p[2] == 0x9B) return 3;   /* ； */
    }
    if (c == 0xE3 && (unsigned char)p[1] == 0x80 && (unsigned char)p[2] == 0x81) return 3; /* 、 */
    return 0;
}

/* 中文释义是否有交集：earth="地球，世界" 与 world="世界" 在孩子眼里是一个意思，
   只做 strcmp 全等比较会漏掉，必须用义项级判断 */
static int cn_overlap(const char *a, const char *b)
{
    char buf[32];
    const char *p = a;
    while (*p) {
        const char *q = p;
        while (*q && !sep_len(q)) q++;
        size_t len = (size_t)(q - p);
        if (len > 0 && len < sizeof(buf)) {
            memcpy(buf, p, len);
            buf[len] = 0;
            if (strstr(b, buf)) return 1;
        }
        if (!*q) break;
        p = q + sep_len(q);
    }
    return 0;
}

static void build_question(void)
{
    if (s_mode == MODE_REVIEW && s_wrong_n > 0) {
        s_qWord = s_wrong_book[rand() % s_wrong_n];
    } else {
        s_qWord = pick_train_word();
    }
    s_qDir = rand() & 1;          /* 英选汉 / 汉选英 各半 */

    float focus = focus_book();
    s_qOpts[0] = s_qWord;
    int filled = 1;
    for (int tries = 0; tries < 200 && filled < 4; tries++) {
        int idx = rand() % WORD_POOL_SIZE;
        int dup = 0;
        for (int j = 0; j < filled; j++) if (s_qOpts[j] == idx) dup++;
        if (dup) continue;
        float dd = (float)word_pool[idx].book - focus;
        if (dd < 0) dd = -dd;
        if (dd > 3.0f) continue;
        /* 干扰项不能与正确答案同义：两种题型都要判 en 和 cn 义项，
           否则题干"帽子"、选项 cap/hat 两个都对 */
        if (strcmp(word_pool[idx].en, word_pool[s_qWord].en) == 0) continue;
        if (cn_overlap(word_pool[idx].cn, word_pool[s_qWord].cn)) continue;
        s_qOpts[filled++] = idx;
    }
    while (filled < 4) {          /* 极端兜底：顺序补不重复的 */
        int idx = rand() % WORD_POOL_SIZE;
        int dup = 0;
        for (int j = 0; j < filled; j++) if (s_qOpts[j] == idx) dup++;
        if (!dup) s_qOpts[filled++] = idx;
    }
    for (int i = 3; i > 0; i--) { /* shuffle */
        int j = rand() % (i + 1);
        int t = s_qOpts[i]; s_qOpts[i] = s_qOpts[j]; s_qOpts[j] = t;
    }
    for (int i = 0; i < 4; i++) if (s_qOpts[i] == s_qWord) { s_qCorrect = i; break; }
    s_opt = 0;
}

static bool in_wrong_book(int idx)
{
    for (int i = 0; i < s_wrong_n; i++) if (s_wrong_book[i] == idx) return true;
    return false;
}
static void add_wrong(int idx)
{
    if (s_wrong_n >= MAX_WRONG) return;
    if (in_wrong_book(idx)) return;
    s_wrong_book[s_wrong_n++] = idx;
}
static void remove_wrong(int idx)
{
    for (int i = 0; i < s_wrong_n; i++) {
        if (s_wrong_book[i] == idx) {
            for (int k = i; k < s_wrong_n - 1; k++) s_wrong_book[k] = s_wrong_book[k + 1];
            s_wrong_n--;
            return;
        }
    }
}

static void start_train(void)
{
    s_mode = MODE_TRAIN;
    s_opt = 0;
    build_question();
    if (s_panel) lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    render_panel();
}
static void start_review(void)
{
    if (s_wrong_n == 0) return;
    s_mode = MODE_REVIEW;
    s_opt = 0;
    build_question();
    if (s_panel) lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    render_panel();
}
static void exit_qa(void)
{
    s_mode = MODE_HOME;
    if (s_panel) lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    s_stat.energy = CLAMP((int)s_stat.energy - 4);
}

static void answer(int option)
{
    bool right = (option == s_qCorrect);
    if (right) {
        s_mem[s_qWord] = MEM_FULL;            /* 学会 / 记忆刷新到满分 */
        if (s_mode == MODE_REVIEW) remove_wrong(s_qWord);
        s_stat.hunger = CLAMP(s_stat.hunger + 25);
        s_stat.happy  = CLAMP(s_stat.happy  + 10);
        try_evolve();                          /* 词数变了，检查进化 */
        if (s_mode == MODE_REVIEW && s_wrong_n == 0) {
            exit_qa();                         /* 错题本清空 → 自动回主页 */
            return;
        }
    } else {
        s_stat.happy = CLAMP((int)s_stat.happy - 3);
        add_wrong(s_qWord);
    }
    s_stat.energy = CLAMP((int)s_stat.energy - 2);
    build_question();
    render_panel();
}

/* 闪光淡出回调（一次性，手动 delete） */
static void fade_flash_cb(lv_timer_t *tm)
{
    if (s_flash) lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
    lv_timer_delete(tm);
}

/* ============================================================
 *  进化：严格同家族、逐档，缺档不越级
 * ============================================================ */
static void try_evolve(void)
{
    static const int TH[3] = { 0, EVO_KNOWN_1, EVO_KNOWN_2 };
    const pokemon_gif_t *cur = &pokemon_gifs[s_cur_poke_idx];
    int target = cur->evo_stage + 1;
    if (target > 2) return;
    if (known_count() < TH[target]) return;

    int next_idx = -1;
    for (int i = 0; i < pokemon_gif_count; i++) {
        if (pokemon_gifs[i].evo_family == cur->evo_family &&
            pokemon_gifs[i].evo_stage == target) {
            next_idx = i; break;
        }
    }
    if (next_idx < 0) return;
    s_cur_poke_idx = next_idx;
    LOGI("evolve -> %s (known=%d)", pokemon_gifs[next_idx].name, known_count());

    if (s_flash) {
        lv_obj_move_foreground(s_flash);
        lv_obj_set_style_bg_opa(s_flash, LV_OPA_90, 0);
        lv_obj_remove_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
        lv_timer_create(fade_flash_cb, 350, NULL);
    }

    if (s_gif) {
        gif_player_stop(s_gif);
        lv_obj_delete(s_gif);
        s_gif = NULL;
    }
    s_gif = gif_player_create(s_scr, PET_X, PET_Y, PET_SIZE, PET_SIZE);
    if (s_gif) {
        gif_player_set_bob(s_gif, true);
        gif_player_play(s_gif, pokemon_gifs[s_cur_poke_idx].data,
                        pokemon_gifs[s_cur_poke_idx].len);
    }
}

/* ============================================================
 *  破壳 / 重开蛋
 * ============================================================ */
static void do_hatch(void)
{
    if (s_egg) {
        ui_pixel_egg_destroy(s_egg);
        s_egg = NULL;
    }
    s_cur_poke_idx = (uint8_t)s_bases[rand() % s_base_n];   /* 只出有进化链的基础形态 */
    LOGI("hatch -> %s", pokemon_gifs[s_cur_poke_idx].name);
    s_mode = MODE_HOME;
    s_gif = gif_player_create(s_scr, PET_X, PET_Y, PET_SIZE, PET_SIZE);
    if (s_gif) {
        gif_player_set_bob(s_gif, true);
        gif_player_play(s_gif, pokemon_gifs[s_cur_poke_idx].data,
                        pokemon_gifs[s_cur_poke_idx].len);
    }
}

static void reset_egg(void)
{
    if (s_gif) {
        gif_player_stop(s_gif);
        lv_obj_delete(s_gif);
        s_gif = NULL;
    }
    if (s_egg) {
        ui_pixel_egg_destroy(s_egg);
        s_egg = NULL;
    }
    s_stat.hunger = 80; s_stat.happy = 80; s_stat.energy = 80;
    s_hatch_clicks = 0;
    s_sleeping = false;
    /* 学习记录（s_mem / 错题本）保留 —— 重修是重养宠物，不是清空学习 */
    s_mode = MODE_EGG;
    s_menu = MENU_TRAIN;
    s_opt = 0;
    s_tick_div = 0;
    if (s_night) lv_obj_add_flag(s_night, LV_OBJ_FLAG_HIDDEN);
    if (s_topbar_bg) lv_obj_add_flag(s_topbar_bg, LV_OBJ_FLAG_HIDDEN);
    s_egg = ui_pixel_egg_create(s_scr, EGG_X, EGG_Y);
}

/* ============================================================
 *  睡觉：黑幕三段拉下 / 向上撤出
 * ============================================================ */
typedef struct { int step; bool leaving; } curtain_ctx_t;

static void curtain_cb(lv_timer_t *t)
{
    curtain_ctx_t *c = (curtain_ctx_t *)lv_timer_get_user_data(t);
    static const int H_IN[3]  = { 107, 213, 320 };   /* 入睡：顶部往下拉（噔噔噔） */
    static const int H_OUT[3] = { 213, 107, 0 };     /* 醒来：底部锚定收缩，上方先亮 */
    int h = c->leaving ? H_OUT[c->step] : H_IN[c->step];
    if (c->leaving) lv_obj_set_pos(s_night, 0, 320 - h);
    else            lv_obj_set_pos(s_night, 0, 0);
    lv_obj_set_size(s_night, 240, h);
    c->step++;
    if (c->step >= 3) {
        if (c->leaving) {
            lv_obj_add_flag(s_night, LV_OBJ_FLAG_HIDDEN);
            s_sleeping = false;                       /* 动画走完才真正解锁 */
        } else {
            lv_obj_remove_flag(s_zzz, LV_OBJ_FLAG_HIDDEN);
        }
        lv_timer_delete(t);
        free(c);
        s_curtain_t = NULL;
    }
}

static void curtain_start(bool leaving)
{
    curtain_ctx_t *c = (curtain_ctx_t *)malloc(sizeof(curtain_ctx_t));
    if (!c) {                             /* 内存不足兜底：直接到位 */
        if (leaving) {
            lv_obj_add_flag(s_night, LV_OBJ_FLAG_HIDDEN);
            s_sleeping = false;
        } else {
            lv_obj_set_pos(s_night, 0, 0);
            lv_obj_set_size(s_night, 240, 320);
            lv_obj_remove_flag(s_zzz, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    c->step = 0;
    c->leaving = leaving;
    s_curtain_t = lv_timer_create(curtain_cb, 130, c);
}

static void start_sleep(void)
{
    s_sleeping = true;
    s_stat.energy = CLAMP(s_stat.energy + 30);
    s_stat.hunger = CLAMP((int)s_stat.hunger - 3);
    lv_obj_add_flag(s_zzz, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_night, 0, 0);
    lv_obj_set_size(s_night, 240, 0);
    lv_obj_remove_flag(s_night, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_night);        /* 黑幕压宠物 */
    curtain_start(false);
}

static void start_wake(void)
{
    lv_obj_add_flag(s_zzz, LV_OBJ_FLAG_HIDDEN);
    curtain_start(true);                    /* s_sleeping 在动画结束时清 */
}

/* ============================================================
 *  遗忘：记忆度每小时 -1，归 0 掉进错题本，攻数值跟着掉
 * ============================================================ */
static void forget_tick(void)
{
    bool dropped = false;
    for (int i = 0; i < WORD_POOL_SIZE; i++) {
        if (s_mem[i] == 0) continue;
        s_mem[i]--;
        if (s_mem[i] == 0) {
            add_wrong(i);
            dropped = true;
        }
    }
    if (dropped) s_atk_alert_blinks = 6;    /* 攻框闪红约 2 秒 */
}

/* ============================================================
 *  状态衰减 tick（1 秒）
 * ============================================================ */
static void tick_cb(void *arg)
{
    (void)arg;
    if (!s_active || s_mode == MODE_EGG) return;

    if (++s_tick_div >= 3) {               /* 每 3 秒掉 1 点 */
        s_tick_div = 0;
        s_stat.hunger = CLAMP((int)s_stat.hunger - 1);
        s_stat.happy  = CLAMP((int)s_stat.happy  - 1);
        s_stat.energy = CLAMP((int)s_stat.energy - 1);
        if (s_sleeping) {
            s_stat.energy = CLAMP(s_stat.energy + 1);
            s_stat.hunger = CLAMP((int)s_stat.hunger - 1);
        }
    }

    if (++s_forget_cnt >= FORGET_SEC) {    /* 遗忘时钟 */
        s_forget_cnt = 0;
        forget_tick();
    }
}

/* ============================================================
 *  demo 接口
 * ============================================================ */
void demo_pet_enter(void)
{
    LOGI("enter free=%d", (int)esp_get_free_heap_size());
    srand(esp_random());
    s_stat.hunger = 80; s_stat.happy = 80; s_stat.energy = 80;
    s_hatch_clicks = 0;
    s_sleeping = false;
    s_tick_div = 0;
    s_forget_cnt = 0;
    s_mode = MODE_EGG;
    s_menu = MENU_TRAIN;
    s_opt = 0;
    s_cur_poke_idx = 0;
    s_wrong_n = 0;
    s_atk_alert_blinks = 0;
    s_active = true;
    memset(s_mem, 0, sizeof(s_mem));

    /* 对象清零（防止上次退出残留） */
    s_scr = NULL; s_egg = NULL; s_gif = NULL;
    s_topbar_bg = NULL; s_atkbox = NULL; s_atk_num = NULL;
    s_night = NULL; s_zzz = NULL; s_flash = NULL;
    s_panel = NULL; s_p_word = NULL;
    for (int i = 0; i < 4; i++) s_menu_btns[i] = NULL;
    for (int i = 0; i < 5; i++) { s_p_opts[i] = NULL; s_p_txt[i] = NULL; s_p_cursor[i] = NULL; }
    s_blink_t = NULL; s_curtain_t = NULL;

    /* 破壳候选：stage0 且家族内有 stage1（有进化链才入候选） */
    s_base_n = 0;
    for (int i = 0; i < pokemon_gif_count; i++) {
        if (pokemon_gifs[i].evo_stage != 0) continue;
        for (int j = 0; j < pokemon_gif_count; j++) {
            if (pokemon_gifs[j].evo_family == pokemon_gifs[i].evo_family &&
                pokemon_gifs[j].evo_stage == 1) {
                if (s_base_n < (int)(sizeof(s_bases) / sizeof(s_bases[0])))
                    s_bases[s_base_n++] = i;
                break;
            }
        }
    }
    LOGI("hatch bases=%d", s_base_n);

    gif_player_set_bg_fn(bg_color_at);

    s_scr = ui_pixel_screen_create("PET");
    draw_background(s_scr);

    build_topbar();
    lv_obj_add_flag(s_topbar_bg, LV_OBJ_FLAG_HIDDEN);

    s_egg = ui_pixel_egg_create(s_scr, EGG_X, EGG_Y);

    /* 睡觉黑幕（初始隐藏；Zzz 为子对象，拉幕到位才显示） */
    s_night = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_night, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_night, 0, 0);
    lv_obj_set_size(s_night, 240, 320);
    lv_obj_set_style_bg_color(s_night, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_night, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_night, 0, 0);
    lv_obj_set_style_pad_all(s_night, 0, 0);
    lv_obj_set_style_radius(s_night, 0, 0);
    lv_obj_add_flag(s_night, LV_OBJ_FLAG_HIDDEN);
    s_zzz = lv_label_create(s_night);
    lv_label_set_text(s_zzz, "Zzz");
    lv_obj_set_style_text_font(s_zzz, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_zzz, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(s_zzz);
    lv_obj_add_flag(s_zzz, LV_OBJ_FLAG_HIDDEN);

    /* 进化闪光 */
    s_flash = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_flash, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_flash, 0, 0);
    lv_obj_set_size(s_flash, 240, 320);
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_0, 0);
    lv_obj_set_style_border_width(s_flash, 0, 0);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);

    /* 训练 / 温习面板：全屏绿豆覆盖，无任何标题提示文字 */
    s_panel = lv_obj_create(s_scr);
    lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(s_panel, 0, 0);
    lv_obj_set_size(s_panel, 240, 320);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_panel, 0, 0);
    lv_obj_set_style_pad_all(s_panel, 0, 0);
    lv_obj_set_style_radius(s_panel, 0, 0);
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);

    s_p_word = lv_label_create(s_panel);
    lv_obj_set_pos(s_p_word, 0, 16);
    lv_obj_set_size(s_p_word, 240, 34);
    lv_obj_set_style_text_color(s_p_word, lv_color_hex(C_INK), 0);
    lv_obj_set_style_text_align(s_p_word, LV_TEXT_ALIGN_CENTER, 0);

    for (int i = 0; i < 5; i++) {
        int w = (i < 4) ? 220 : 154;      /* 返回行变窄 */
        lv_obj_t *row = lv_obj_create(s_panel);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(row, (240 - w) / 2, 58 + i * 48);
        lv_obj_set_size(row, w, 42);
        lv_obj_set_style_bg_color(row, lv_color_hex(C_ROW), 0);
        lv_obj_set_style_border_width(row, 2, 0);
        lv_obj_set_style_border_color(row, lv_color_hex(C_ROWBRD), 0);
        lv_obj_set_style_radius(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        s_p_opts[i] = row;

        s_p_cursor[i] = lv_label_create(row);
        lv_label_set_text(s_p_cursor[i], ">");
        lv_obj_set_style_text_font(s_p_cursor[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_p_cursor[i], lv_color_hex(0xE53935), 0);
        lv_obj_set_pos(s_p_cursor[i], 8, 12);
        lv_obj_set_style_bg_opa(s_p_cursor[i], LV_OPA_TRANSP, 0);

        s_p_txt[i] = lv_label_create(row);
        lv_obj_set_style_text_color(s_p_txt[i], lv_color_hex(C_INK), 0);
        lv_obj_set_pos(s_p_txt[i], 26, 10);
        lv_obj_set_size(s_p_txt[i], w - 34, 24);
    }

    lv_screen_load(s_scr);

    s_blink_t = lv_timer_create(blink_timer_cb, 350, NULL);

    esp_timer_create_args_t args = { .callback = tick_cb, .name = "pet_tick" };
    esp_timer_create(&args, &s_tick_timer);
    esp_timer_start_periodic(s_tick_timer, 1000000);
}

void demo_pet_exit(void)
{
    LOGI("exit free=%d", (int)esp_get_free_heap_size());
    s_active = false;
    s_sleeping = false;
    if (s_tick_timer) { esp_timer_stop(s_tick_timer); esp_timer_delete(s_tick_timer); s_tick_timer = NULL; }
    if (s_blink_t) { lv_timer_delete(s_blink_t); s_blink_t = NULL; }
    /* s_curtain_t 若还在跑，其 ctx 由 cb 自释放；屏幕整体删除后定时器不再触发安全路径，
       这里主动停掉防悬空 */
    if (s_curtain_t) {
        void *c = lv_timer_get_user_data(s_curtain_t);
        lv_timer_delete(s_curtain_t);
        if (c) free(c);
        s_curtain_t = NULL;
    }
    gif_player_destroy();
    if (s_scr) lv_obj_delete(s_scr);
    s_scr = NULL;
    s_egg = NULL; s_gif = NULL;
    s_topbar_bg = NULL; s_atkbox = NULL; s_atk_num = NULL;
    s_night = NULL; s_zzz = NULL; s_flash = NULL;
    s_panel = NULL; s_p_word = NULL;
    for (int i = 0; i < 4; i++) s_menu_btns[i] = NULL;
    for (int i = 0; i < 5; i++) { s_p_opts[i] = NULL; s_p_txt[i] = NULL; s_p_cursor[i] = NULL; }
    LOGI("exit done free=%d", (int)esp_get_free_heap_size());
}

void demo_pet_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    /* 睡觉锁键：只有 OK 能唤醒，其余键全吞（黑幕动画中也吞） */
    if (s_sleeping) {
        if (ev == BSP_BTN_CLICK && btn == BSP_BTN_OK && s_curtain_t == NULL) {
            start_wake();
        }
        return;
    }

    /* 训练 / 温习：上下选项，OK 确认 */
    if (s_mode == MODE_TRAIN || s_mode == MODE_REVIEW) {
        if (ev == BSP_BTN_CLICK) {
            if (btn == BSP_BTN_UP) {
                s_opt = (s_opt + 4) % 5;
                render_panel();
            } else if (btn == BSP_BTN_DOWN) {
                s_opt = (s_opt + 1) % 5;
                render_panel();
            } else if (btn == BSP_BTN_OK) {
                if (s_opt == 4) exit_qa();
                else            answer(s_opt);
            }
        }
        return;
    }

    /* 蛋状态：只响应 OK */
    if (s_mode == MODE_EGG) {
        if (ev == BSP_BTN_CLICK && btn == BSP_BTN_OK) {
            s_hatch_clicks++;
            if (s_egg) ui_pixel_egg_shake(s_egg, s_hatch_clicks <= 3 ? s_hatch_clicks : 3);
            if (s_hatch_clicks >= 3) {
                do_hatch();
                if (s_topbar_bg) lv_obj_remove_flag(s_topbar_bg, LV_OBJ_FLAG_HIDDEN);
            }
        }
        return;
    }

    /* HOME */
    if (ev == BSP_BTN_CLICK) {
        if (btn == BSP_BTN_UP) {
            s_menu = (s_menu + 3) % 4;
            blink_timer_cb(NULL);
        } else if (btn == BSP_BTN_DOWN) {
            s_menu = (s_menu + 1) % 4;
            blink_timer_cb(NULL);
        } else if (btn == BSP_BTN_OK) {
            switch ((int)s_menu) {
                case MENU_TRAIN:  start_train(); break;
                case MENU_SLEEP:  start_sleep(); break;
                case MENU_REVIEW: start_review(); break;   /* 错题空时内部直接返回 */
                case MENU_RESET:  reset_egg(); break;
            }
        }
    } else if (ev == BSP_BTN_LONG && btn == BSP_BTN_UP) {
        reset_egg();
    }
}
