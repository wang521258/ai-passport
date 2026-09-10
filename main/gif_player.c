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

/* GIF 原始帧延迟(通常 50~100ms)在小屏上看着像抽搐。
 * 统一放慢到 GIF_SLOWDOWN 倍，并设 GIF_MIN_DELAY 下限，动作更柔和。 */
#define GIF_SLOWDOWN  3
#define GIF_MIN_DELAY 120

/* 待机动效：上下浮动 2px，8 帧一个周期（约 1 秒一次呼吸） */
static const int8_t BOB_TABLE[8] = { 0, -1, -1, -2, -2, -1, -1, 0 };

typedef struct {
    GIFIMAGE gif;
    lv_obj_t *canvas;
    uint16_t *buf;
    int cw, ch;          /* canvas 尺寸 */
    int gw, gh;          /* GIF 画布尺寸 */
    int ox, oy;          /* canvas 在屏幕上的左上角（背景取色用） */
    int bob;             /* 当前上下浮动偏移 */
    int frame_no;        /* 已播帧数（驱动 bob） */
    bool bob_on;         /* 是否启用浮动 */
    lv_timer_t *timer;
    bool playing;
    bool opened;         /* GIF_openRAM 是否成功过,GIF_close 前必查 */
} gif_player_t;

/* 场景背景取色回调：由 demo_pet 注册，用于消除 canvas 白底方块 */
static uint16_t (*s_bg_fn)(int x, int y) = NULL;

void gif_player_set_bg_fn(uint16_t (*fn)(int x, int y)) { s_bg_fn = fn; }

void gif_player_set_bob(lv_obj_t *canvas, bool enable)
{
    if (!canvas) return;
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (p) p->bob_on = enable;
}

void gif_player_set_pos(lv_obj_t *canvas, int x, int y)
{
    if (!canvas) return;
    gif_player_t *p = lv_obj_get_user_data(canvas);
    if (!p) return;
    p->ox = x; p->oy = y;
    lv_obj_set_pos(canvas, x, y);
}

/* 用场景背景色铺满画布：无 alpha 通道时消除白方块的唯一办法 */
static void fill_background(gif_player_t *p)
{
    if (s_bg_fn) {
        for (int r = 0; r < p->ch; r++) {
            uint16_t *row = &p->buf[r * p->cw];
            int sy = p->oy + r;
            for (int c = 0; c < p->cw; c++) row[c] = s_bg_fn(p->ox + c, sy);
        }
    } else {
        uint16_t bg = BG_RGB565;
        for (int i = 0; i < p->cw * p->ch; i++) p->buf[i] = bg;
    }
}

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

    /* GIF 原始 y → canvas y（最近邻放大，区间填充）
     *
     * 注意：GIF 只有 37x38，canvas 是 64x64。若按单点映射（cy = gy*ch/gh），
     * 64 行里只有 38 行会被写到，剩下 26 行留背景色 → 满屏横向条纹；
     * 列方向同理会有竖向空隙。所以这里按 [cy0, cy1) / [cx0, cx1) 区间整块
     * 填充，放大后是干净的色块，不会出现斑马纹。 */
    int gy = pDraw->iY + pDraw->y;
    if (gy < 0 || gy >= gh) return;
    int cy0 = (gh > 0 && ch > 0) ? (gy * ch / gh) : gy;
    int cy1 = (gh > 0 && ch > 0) ? ((gy + 1) * ch / gh) : (gy + 1);
    if (cy1 <= cy0) cy1 = cy0 + 1;
    /* 待机上下浮动：只偏移宠物本体，背景已在 fill_background 里画好不动 */
    cy0 += s_cur->bob;
    cy1 += s_cur->bob;
    if (cy1 <= 0 || cy0 >= ch) return;      /* 整行被浮出画布 */
    if (cy0 < 0)  cy0 = 0;
    if (cy1 > ch) cy1 = ch;

    if (pDraw->ucHasTransparency) {
        uint8_t trans = pDraw->ucTransparent;
        for (int x = 0; x < w; x++) {
            uint8_t c = s[x];
            if (c != trans) {
                int gx = x0 + x;
                int cx0 = (gw > 0 && cw > 0) ? (gx * cw / gw) : gx;
                int cx1 = (gw > 0 && cw > 0) ? ((gx + 1) * cw / gw) : (gx + 1);
                if (cx1 <= cx0) cx1 = cx0 + 1;
                if (cx0 < 0)  cx0 = 0;
                if (cx1 > cw) cx1 = cw;
                uint16_t v = pal[c];
                for (int cy = cy0; cy < cy1; cy++) {
                    uint16_t *row = &s_cur->buf[cy * cw];
                    for (int cx = cx0; cx < cx1; cx++) row[cx] = v;
                }
            }
        }
    } else {
        for (int x = 0; x < w; x++) {
            int gx = x0 + x;
            int cx0 = (gw > 0 && cw > 0) ? (gx * cw / gw) : gx;
            int cx1 = (gw > 0 && cw > 0) ? ((gx + 1) * cw / gw) : (gx + 1);
            if (cx1 <= cx0) cx1 = cx0 + 1;
            if (cx0 < 0)  cx0 = 0;
            if (cx1 > cw) cx1 = cw;
            uint16_t v = pal[s[x]];
            for (int cy = cy0; cy < cy1; cy++) {
                uint16_t *row = &s_cur->buf[cy * cw];
                for (int cx = cx0; cx < cx1; cx++) row[cx] = v;
            }
        }
    }
}

static void frame_timer_cb(lv_timer_t *t)
{
    gif_player_t *p = lv_timer_get_user_data(t);
    if (!p || !p->playing || !p->buf || !p->opened) return;
    int delay = 0;
    /* 注意：不要每帧重铺背景！宝可梦 GIF 大量帧是增量帧（只编码变化区域），
     * 铺底会把上一帧该保留的内容擦掉 → 画面残缺、整块闪烁。
     * 背景只在首次播放和循环回绕时铺一次，透明像素自然透出历史帧。 */
    p->frame_no++;
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
        fill_background(p);
        s_cur = p;
        GIF_playFrame(&p->gif, &delay, NULL);
        s_cur = NULL;
    }
    /* 放慢播放：原始帧延迟 × GIF_SLOWDOWN，并给下限 */
    delay *= GIF_SLOWDOWN;
    if (delay < GIF_MIN_DELAY) delay = GIF_MIN_DELAY;
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
    p->ox = x;
    p->oy = y;
    p->bob = 0;
    p->frame_no = 0;
    p->bob_on = true;
    p->buf = calloc((size_t)w * (size_t)h, sizeof(uint16_t));
    GIF_LOG("calloc canvas buf(%dB) -> %p, free=%d",
             (int)((size_t)w * (size_t)h * 2), p->buf, (int)esp_get_free_heap_size());
    if (!p->buf) {
        p->cw = p->ch = 0;
        return NULL;                  /* 内存不足：优雅降级 */
    }
    fill_background(p);

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
    fill_background(p);
    p->bob = p->bob_on ? BOB_TABLE[p->frame_no & 7] : 0;
    p->frame_no++;
    s_cur = p;
    int rc = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    GIF_LOG("frame1 rc=%d err=%d delay=%d free=%d", rc,
            GIF_getLastError(&p->gif), delay, (int)esp_get_free_heap_size());
    if (rc < 0) {
        /* 解码失败：保留白底但不启动定时器，至少不崩 */
        return 0;
    }
    delay *= GIF_SLOWDOWN;
    if (delay < GIF_MIN_DELAY) delay = GIF_MIN_DELAY;

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
