/**
 * @file gif_player.c
 * @brief 基于 AnimatedGIF + LVGL canvas 的 GIF 播放器
 */
#include "gif_player.h"
#include "AnimatedGIF.h"
#include "ui_pixel.h"
#include "lvgl.h"
#include "esp_timer.h"
#include <string.h>

/* gif.inl 包含纯 C 解码核心（GIF_openRAM / GIF_playFrame 等） */
/* ESP-IDF 下没有 memcpy_P 宏，手动定义 */
#ifndef memcpy_P
#define memcpy_P memcpy
#endif
#include "gif.inl"

#define MAX_GIF_W 96
#define MAX_GIF_H 96

/* canvas 背景色（与屏幕纸色一致，RGB565） */
/* UI_PAPER=0xF4F4EA → RGB565: R5=30 G6=61 B5=29 = 0xF7DD */
#define BG_RGB565  ((uint16_t)0xF7DD)

typedef struct {
    GIFIMAGE gif;
    lv_obj_t *canvas;
    uint16_t *buf;
    int cw, ch;          /* canvas 尺寸 */
    int gw, gh;          /* GIF 画布尺寸 */
    lv_timer_t *timer;
    bool playing;
} gif_player_t;

/* 当前播放中的播放器（供 draw callback 访问） */
static gif_player_t *s_cur = NULL;

/* GIF draw callback：把解码行写入 canvas buffer */
static void gif_draw_cb(GIFDRAW *pDraw)
{
    if (!s_cur || !s_cur->buf) return;
    int cw = s_cur->cw;
    int ch = s_cur->ch;
    int gw = s_cur->gw;
    int gh = s_cur->gh;
    uint16_t *pal = pDraw->pPalette;
    uint8_t *s = pDraw->pPixels;
    int x0 = pDraw->iX;
    int w = pDraw->iWidth;

    /* GIF 原始 y → canvas y（最近邻缩放）*/
    int gy = pDraw->iY + pDraw->y;
    int cy = (gh > 0 && ch > 0) ? (gy * ch / gh) : gy;
    if (cy < 0 || cy >= ch) return;

    if (pDraw->ucHasTransparency) {
        uint8_t trans = pDraw->ucTransparent;
        for (int x = 0; x < w; x++) {
            uint8_t c = s[x];
            if (c != trans) {
                int gx = x0 + x;
                int cx = (gw > 0 && cw > 0) ? (gx * cw / gw) : gx;
                if (cx >= 0 && cx < cw) {
                    s_cur->buf[cy * cw + cx] = pal[c];
                }
            }
        }
    } else {
        for (int x = 0; x < w; x++) {
            int gx = x0 + x;
            int cx = (gw > 0 && cw > 0) ? (gx * cw / gw) : gx;
            if (cx >= 0 && cx < cw) {
                s_cur->buf[cy * cw + cx] = pal[s[x]];
            }
        }
    }
}

static void frame_timer_cb(lv_timer_t *t)
{
    gif_player_t *p = lv_timer_get_user_data(t);
    if (!p || !p->playing) return;
    int delay = 0;
    /* 每帧前清空 canvas，避免 disposal method 导致的残影 */
    uint16_t bg = BG_RGB565;
    for (int i = 0; i < p->cw * p->ch; i++) p->buf[i] = bg;
    s_cur = p;
    int res = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    if (res == 0) {
        /* 播完一遍，重新循环 */
        GIF_reset(&p->gif);
        GIF_playFrame(&p->gif, &delay, NULL);
    }
    if (delay < 20) delay = 80; /* 有些 GIF 帧延迟为 0，给个默认值 */
    lv_timer_set_period(p->timer, delay);
    /* 通知 LVGL 刷新 canvas */
    lv_obj_invalidate(p->canvas);
}

lv_obj_t *gif_player_create(lv_obj_t *parent, int x, int y, int w, int h)
{
    gif_player_t *p = calloc(1, sizeof(gif_player_t));
    p->cw = w;
    p->ch = h;
    p->buf = calloc(w * h, sizeof(uint16_t));
    uint16_t bg = BG_RGB565;
    for (int i = 0; i < w * h; i++) p->buf[i] = bg;

    p->canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(p->canvas, p->buf, w, h, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(p->canvas, x, y);
    lv_obj_set_user_data(p->canvas, p);
    return p->canvas;
}

void gif_player_play(lv_obj_t *canvas, const uint8_t *data, int len)
{
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (!p) return;
    gif_player_stop(canvas);

    memset(p->buf, 0, p->cw * p->ch * 2);
    uint16_t bg = BG_RGB565;
    for (int i = 0; i < p->cw * p->ch; i++) p->buf[i] = bg;
    memset(&p->gif, 0, sizeof(GIFIMAGE));
    GIF_openRAM(&p->gif, (uint8_t *)data, len, gif_draw_cb);
    p->gw = GIF_getCanvasWidth(&p->gif);
    p->gh = GIF_getCanvasHeight(&p->gif);
    p->playing = true;

    int delay = 0;
    s_cur = p;
    GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    if (delay < 20) delay = 80;

    p->timer = lv_timer_create(frame_timer_cb, delay, p);
}

void gif_player_stop(lv_obj_t *canvas)
{
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (!p) return;
    if (p->timer) { lv_timer_delete(p->timer); p->timer = NULL; }
    p->playing = false;
    GIF_close(&p->gif);
}
