/**
 * @file ui_pet.c
 * @brief 像素风宠物 UI：粉色宠物（占位）+ 精灵蛋
 *
 * 精灵蛋绘制算法（与 HTML 预览同源）：
 *   上半椭圆 + 下半椭圆拼接，宽度按 sqrt 收缩；
 *   斑点 6 个（归一化坐标），底部阴影，高光，描边。
 *   用 2×2 像素块填色（GBA 像素风），共 36×45=1620 次填色，启动瞬间完成。
 *
 * 蛋缓冲区：12.9KB（72×90 RGB565），lv_canvas_set_buffer 把指针写到
 * obj 的 user_data，销毁时取出并 free。
 */
#include "ui_pet.h"
#include "lvgl.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ====================== 颜色常量（RGB565） ====================== */
#define C_SHELL    0xFFF7  /* 蛋壳米白 (#FFF8E7) */
#define C_SHADE    0xDE5A  /* 底部阴影 (#E3D3AE) */
#define C_SPOT_L   0x9E69  /* 浅绿斑 (#7CB342) */
#define C_SPOT_D   0x7385  /* 深绿斑 (#5A9A2E) */
#define C_LINE     0x4A48  /* 描边深棕 (#3E2723) */
#define C_HILITE   0xFFFF  /* 高光白 */

/* ====================== 像素宠物（粉色，占位） ====================== */
lv_obj_t *ui_pixel_pet_create(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *o = lv_obj_create(parent);
    if (!o) return NULL;
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, 60, 60);
    lv_obj_set_style_radius(o, 30, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(0xFFB6C1), 0);
    lv_obj_set_style_border_color(o, lv_color_hex(0x17202A), 0);
    lv_obj_set_style_border_width(o, 3, 0);
    return o;
}

void ui_pixel_pet_jump(lv_obj_t *pet) { (void)pet; }
void ui_pixel_pet_set_face(lv_obj_t *pet, pet_face_t face) { (void)pet; (void)face; }

/* ====================== 精灵蛋 ====================== */

typedef struct { float sx, sy, rx, ry; } egg_spot_t;
static const egg_spot_t EGG_SPOTS[] = {
    { -0.30f, -0.08f, 0.13f, 0.075f },
    {  0.26f,  0.06f, 0.11f, 0.065f },
    { -0.08f,  0.28f, 0.095f, 0.055f },
    {  0.32f,  0.40f, 0.08f,  0.05f  },
    { -0.36f,  0.44f, 0.07f,  0.045f },
    {  0.04f, -0.34f, 0.085f, 0.05f  },
};
#define EGG_SPOT_N (sizeof(EGG_SPOTS)/sizeof(EGG_SPOTS[0]))

/** v12：场景统一纯色绿豆 #C8DFA0 → RGB565 0xC6F4（蛋"长在场景里"） */
#define PET_BG565 0xC6F4
static uint16_t bg_rgb565_at(int x, int y)
{
    (void)x; (void)y;
    return PET_BG565;
}

static void egg_paint(uint16_t *buf)
{
    const int W = EGG_W, H = EGG_H;
    const int cx = W / 2, cy = H / 2;
    const float A = (float)W * 0.47f;
    const float splitY = (float)H * 0.42f;
    const int PX = 2;

    /* 1. 底色：场景取色（无白块） */
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            buf[y * W + x] = bg_rgb565_at(x, y);
        }
    }

    /* 2. 行宽表 */
    int rows_y[H / PX + 1];
    int rows_w[H / PX + 1];
    int rc = 0;
    for (int y = 0; y < H; y += PX) {
        float rw;
        if (y < (int)splitY) {
            float t = (splitY - y) / splitY;
            rw = A * (1.0f - 0.30f * t * t) * sqrtf(fmaxf(0.0f, 1.0f - t * t));
        } else {
            float t = ((float)y - splitY) / ((float)H - splitY);
            rw = A * (1.0f - 0.10f * t * t) * sqrtf(fmaxf(0.0f, 1.0f - t * t));
        }
        /* v12 封口：椭圆顶端/底端 t→1 时 rw→0 会开口，加最小宽度下限 */
        rw = fmaxf(rw, (float)PX * 0.7f);
        rows_y[rc] = y; rows_w[rc] = (int)rw; rc++;
    }

    /* 3. 逐块填色 */
    for (int ri = 0; ri < rc; ri++) {
        int y = rows_y[ri], rw = rows_w[ri];
        for (int x = 0; x < W; x += PX) {
            float d = fabsf((float)(x + PX / 2) - (float)cx);
            if (d > rw) continue;
            uint16_t col = C_SHELL;
            if (d > rw - PX * 1.1f) col = C_LINE;          /* 描边 */
            else if (y > (int)(H * 0.72f) && d > rw * 0.55f) col = C_SHADE;
            for (int s = 0; s < (int)EGG_SPOT_N; s++) {
                const egg_spot_t *sp = &EGG_SPOTS[s];
                float px = (float)cx + sp->sx * (float)W;
                float py = (float)cy + sp->sy * (float)H;
                float dx = ((float)(x + PX / 2) - px) / (sp->rx * (float)W);
                float dy = ((float)(y + PX / 2) - py) / (sp->ry * (float)H);
                if (dx * dx + dy * dy < 1.0f) {
                    col = (y > (int)(H * 0.60f)) ? C_SPOT_D : C_SPOT_L;
                }
            }
            if ((x + PX / 2) < cx - 8 && y < (int)(H * 0.30f) && d < rw - PX * 2.5f) col = C_HILITE;
            buf[y * W + x] = col;
            if (x + 1 < W) buf[y * W + x + 1] = col;
            if (y + 1 < H) buf[(y + 1) * W + x] = col;
            if (x + 1 < W && y + 1 < H) buf[(y + 1) * W + x + 1] = col;
        }
    }
}

lv_obj_t *ui_pixel_egg_create(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *canvas = lv_canvas_create(parent);
    if (!canvas) return NULL;
    lv_obj_remove_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(canvas, x, y);
    lv_obj_set_size(canvas, EGG_W, EGG_H);

    uint16_t *buf = (uint16_t *)malloc(EGG_W * EGG_H * sizeof(uint16_t));
    if (!buf) {
        lv_obj_delete(canvas);
        return NULL;
    }
    lv_canvas_set_buffer(canvas, buf, EGG_W, EGG_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_user_data(canvas, buf);   /* 销毁时取出并 free */
    egg_paint(buf);
    return canvas;
}

typedef struct {
    lv_obj_t *obj;
    int base_x;
    int level;
    int frame;
} shake_ctx_t;

static const int SHAKE_PAT[8] = { -5, -7, 5, 7, -3, 3, -1, 0 };

static void shake_timer_cb(lv_timer_t *t)
{
    shake_ctx_t *ctx = (shake_ctx_t *)lv_timer_get_user_data(t);
    if (!ctx || !ctx->obj) {
        lv_timer_delete(t);
        return;
    }
    int off = SHAKE_PAT[ctx->frame] * ctx->level;
    lv_obj_set_x(ctx->obj, ctx->base_x + off);
    ctx->frame++;
    if (ctx->frame >= 8) {
        lv_obj_set_x(ctx->obj, ctx->base_x);
        lv_timer_delete(t);
        free(ctx);
    }
}

void ui_pixel_egg_shake(lv_obj_t *egg, int level)
{
    if (!egg || level < 1) return;
    if (level > 3) level = 3;
    shake_ctx_t *ctx = (shake_ctx_t *)malloc(sizeof(shake_ctx_t));
    if (!ctx) return;
    ctx->obj = egg;
    ctx->base_x = lv_obj_get_x(egg);
    ctx->level = level;
    ctx->frame = 0;
    lv_timer_create(shake_timer_cb, 40, ctx);
}

void ui_pixel_egg_destroy(lv_obj_t *egg)
{
    if (!egg) return;
    void *p = lv_obj_get_user_data(egg);
    if (p) free(p);
    lv_obj_delete(egg);
}