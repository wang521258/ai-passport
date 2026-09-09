/**
 * @file gif_player.c
 * @brief 基于 AnimatedGIF + LVGL canvas 的 GIF 播放器
 */
#include "gif_player.h"
#include "AnimatedGIF.h"
#include "ui_pixel.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>

static const char *GP = "GIFP";
#define GIF_LOG(fmt, ...) ESP_LOGI(GP, fmt, ##__VA_ARGS__)

/* gif.inl 的函数由 components/AnimatedGIF/src/gif_impl.c 编译为独立目标文件 */
/* gif_player.c 只需包含 AnimatedGIF.h 获取函数声明 */

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
    bool opened;         /* GIF_openRAM 是否成功过,GIF_close 前必查 */
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
    if (!p || !p->playing || !p->buf || !p->opened) return;
    int delay = 0;
    /* 每帧前清空 canvas，避免 disposal method 导致的残影 */
    uint16_t bg = BG_RGB565;
    for (int i = 0; i < p->cw * p->ch; i++) p->buf[i] = bg;
    s_cur = p;
    int res = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    if (res < 0) {
        /* 解码出错：停掉动画，保留最后一帧画面，别让设备重启 */
        GIF_LOG("playFrame err=%d, stop anim", GIF_getLastError(&p->gif));
        p->playing = false;
        lv_timer_pause(t);
        return;
    }
    if (res == 0) {
        /* 播完一遍，从头循环 */
        GIF_reset(&p->gif);
        uint16_t bg2 = BG_RGB565;
        for (int i = 0; i < p->cw * p->ch; i++) p->buf[i] = bg2;
        s_cur = p;
        GIF_playFrame(&p->gif, &delay, NULL);
        s_cur = NULL;
    }
    if (delay < 20) delay = 80; /* 有些 GIF 帧延迟为 0，给个默认值 */
    lv_timer_set_period(p->timer, delay);
    /* 通知 LVGL 刷新 canvas */
    lv_obj_invalidate(p->canvas);
}

/**
 * 单例播放器。
 *
 * GIFIMAGE 内含 LZW 工作区（usGIFTable 8KB + ucGIFPixels 8KB），单个结构体约 24KB；
 * ESP32-C3 可用动态堆仅 ~81KB，若每次换宠物都新建播放器，破壳一次 + 进化一次
 * 就会吃掉 60KB+，必然 OOM 重启。因此全局只保留一个播放器实例，
 * 换宠物/进化时复用同一块内存，只重建 LVGL canvas。
 */
static gif_player_t *s_player = NULL;

lv_obj_t *gif_player_create(lv_obj_t *parent, int x, int y, int w, int h)
{
    gif_player_t *p;

    if (s_player) {
        /* 复用：停掉旧动画并释放旧画布，避免再次申请 GIFIMAGE */
        p = s_player;
        gif_player_stop(p->canvas);
        if (p->canvas) { lv_obj_delete(p->canvas); p->canvas = NULL; }
        if (p->buf)    { free(p->buf);             p->buf = NULL; }
    } else {
        p = calloc(1, sizeof(gif_player_t));
        GIF_LOG("calloc gif_player_t(%uB) -> %p, free=%d",
                 (unsigned)sizeof(gif_player_t), p, (int)esp_get_free_heap_size());
        if (!p) return NULL;          /* 内存不足：不播动画，但不崩 */
        s_player = p;
    }

    p->cw = w;
    p->ch = h;
    p->buf = calloc((size_t)w * (size_t)h, sizeof(uint16_t));
    GIF_LOG("calloc canvas buf(%dB) -> %p, free=%d",
             (int)((size_t)w * (size_t)h * 2), p->buf, (int)esp_get_free_heap_size());
    if (!p->buf) {
        p->cw = p->ch = 0;
        return NULL;                  /* 内存不足：优雅降级 */
    }
    uint16_t bg = BG_RGB565;
    for (int i = 0; i < w * h; i++) p->buf[i] = bg;

    p->canvas = lv_canvas_create(parent);
    GIF_LOG("canvas -> %p, free=%d", p->canvas, (int)esp_get_free_heap_size());
    if (!p->canvas) {
        free(p->buf); p->buf = NULL;
        return NULL;
    }
    lv_canvas_set_buffer(p->canvas, p->buf, w, h, LV_COLOR_FORMAT_RGB565);
    GIF_LOG("set_buffer done, free=%d", (int)esp_get_free_heap_size());
    lv_obj_set_pos(p->canvas, x, y);
    lv_obj_set_user_data(p->canvas, p);
    return p->canvas;
}

void gif_player_destroy(void)
{
    gif_player_t *p = s_player;
    if (!p) return;
    if (p->timer) { lv_timer_delete(p->timer); p->timer = NULL; }
    p->playing = false;
    if (p->opened) { GIF_close(&p->gif); p->opened = false; }
    if (p->canvas) { lv_obj_delete(p->canvas); p->canvas = NULL; }
    if (p->buf)    { free(p->buf);             p->buf = NULL; }
    free(p);
    s_player = NULL;
    GIF_LOG("destroyed free=%d", (int)esp_get_free_heap_size());
}

int gif_player_play(lv_obj_t *canvas, const uint8_t *data, int len)
{
    if (!canvas || !data || len <= 0) return 0;
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (!p || !p->buf) return 0;
    gif_player_stop(canvas);

    /* === 初始化 GIFIMAGE（等价于 C++ 版 begin()，但 C API 没做完）===
     * GIF_begin 会 memset 整个结构并设定 ucPaletteType。之后必须手动补两件
     * C API 不会做的事：
     *   1) ucDrawType = GIF_DRAW_RAW —— 我们要的是 8bit 索引 + RGB565 调色板回调
     *   2) pLineBufAligned 指向 16 字节对齐的行缓冲
     * 第 2 条就是之前一调 GIF_playFrame 就重启的根因：C++ 的 begin() 会算
     * 这个指针，C 路径下永远为 NULL，于是 GIFMakePels() 往 NULL+偏移 写像素
     * → LoadStoreError → 重启 → 只能跳过 playFrame → 画面全白。 */
    GIF_begin(&p->gif, GIF_PALETTE_RGB565_LE);
    p->gif.ucDrawType = GIF_DRAW_RAW;
    {
        uint8_t  *lb  = p->gif.ucLineBuf;
        uintptr_t mis = (uintptr_t)lb & 15u;
        if (mis) lb += (16u - mis);
        p->gif.pLineBufAligned = lb;
    }

    /* sprite 数据留在 Flash(.rodata)，不再拷贝到 RAM：
     * AnimatedGIF 只在 readMem() 里用 memmove 从 pData 取数据，不对其做
     * 32bit 直接读取，因此没有对齐要求；而最大的 venusaur GIF 有 125KB，
     * 拷进 RAM 会直接把 ESP32-C3 的堆吃光。 */
    int ok = GIF_openRAM(&p->gif, (uint8_t *)data, len, gif_draw_cb);
    GIF_LOG("openRAM ok=%d canvas=%ux%u linebuf=%p free=%d", ok,
             (unsigned)GIF_getCanvasWidth(&p->gif),
             (unsigned)GIF_getCanvasHeight(&p->gif),
             p->gif.pLineBufAligned,
             (int)esp_get_free_heap_size());
    if (!ok) {
        p->opened = false;
        GIF_LOG("openRAM failed err=%d", GIF_getLastError(&p->gif));
        return 0;
    }
    p->opened = true;
    p->gw = GIF_getCanvasWidth(&p->gif);
    p->gh = GIF_getCanvasHeight(&p->gif);

    /* 解第一帧：能跑到这里就说明解码器可用 */
    int delay = 0;
    s_cur = p;
    int rc = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    GIF_LOG("frame1 rc=%d err=%d delay=%d free=%d", rc,
            GIF_getLastError(&p->gif), delay, (int)esp_get_free_heap_size());
    if (rc < 0) {
        /* 解码失败：保留白底但不启动定时器，至少不崩 */
        return 0;
    }
    if (delay < 20) delay = 80;

    p->playing = true;
    if (!p->timer) {
        p->timer = lv_timer_create(frame_timer_cb, (uint32_t)delay, p);
    } else {
        lv_timer_set_period(p->timer, (uint32_t)delay);
        lv_timer_resume(p->timer);
    }
    lv_obj_invalidate(p->canvas);
    return 1;
}

void gif_player_stop(lv_obj_t *canvas)
{
    if (!canvas) return;
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (!p) return;
    if (p->timer) { lv_timer_delete(p->timer); p->timer = NULL; }
    p->playing = false;
    if (p->opened) { GIF_close(&p->gif); p->opened = false; }
}
