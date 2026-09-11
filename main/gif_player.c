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
#include <limits.h>

static const char *GP = "GIFP";
#define GIF_LOG(fmt, ...) ESP_LOGI(GP, fmt, ##__VA_ARGS__)

/* gif.inl 的函数由 components/AnimatedGIF/src/gif_impl.c 编译为独立目标文件 */
/* gif_player.c 只需包含 AnimatedGIF.h 获取函数声明 */

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

    /* --- 等比缩放参数(居中对齐,防止非正方形 GIF 被拉成正方形) --- */
    int sc_w, sc_h;      /* GIF 等比缩放后的像素尺寸 */
    int off_x, off_y;    /* 在 canvas 内的居中偏移 */

    /* --- 帧处置(disposal)跟踪 ---
     * 库在 RAW(GIF_DRAW_RAW) 模式下不替我们做处置,必须自己做。
     * 实测 42 只宠物共 1212 帧里 1110 帧(91.6%) 处置方法=2
     * (dispose to background),即画下一帧前必须把本帧矩形擦回背景。 */
    bool    prev_valid;  /* 上一帧是否留下了有效矩形 */
    uint8_t prev_disp;   /* 上一帧的处置方法 */
    int     prev_x0, prev_y0, prev_x1, prev_y1;  /* 上一帧矩形(canvas 坐标) */
    int     cur_x0,  cur_y0,  cur_x1,  cur_y1;   /* 本帧矩形(canvas 坐标) */
    uint8_t cur_disp;    /* 本帧的处置方法 */
    bool    cur_dirty;   /* 本帧回调是否真的输出过像素 */
    bool    wrap_pending;/* 末帧已展示满一个周期,下一 tick 才回绕 */
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

/* 把 canvas 上的一块矩形擦回场景背景色（= 场景背景取色，无回调时用纸色）。
 * 用途：GIF 处置方法 2（dispose to background）—— 上一帧的墨迹必须在画新帧前清掉。 */
static void clear_rect(gif_player_t *p, int x0, int y0, int x1, int y1)
{
    if (x0 < 0)      x0 = 0;
    if (y0 < 0)      y0 = 0;
    if (x1 > p->cw)  x1 = p->cw;
    if (y1 > p->ch)  y1 = p->ch;
    if (x1 <= x0 || y1 <= y0) return;
    for (int r = y0; r < y1; r++) {
        uint16_t *row = &p->buf[r * p->cw];
        int sy = p->oy + r;
        if (s_bg_fn) {
            for (int c = x0; c < x1; c++) row[c] = s_bg_fn(p->ox + c, sy);
        } else {
            for (int c = x0; c < x1; c++) row[c] = BG_RGB565;
        }
    }
}

/* 记录本帧矩形作为"上一帧"，供下一次解码前处置 */
static void commit_frame(gif_player_t *p)
{
    if (!p->cur_dirty) return;
    p->prev_x0 = p->cur_x0; p->prev_y0 = p->cur_y0;
    p->prev_x1 = p->cur_x1; p->prev_y1 = p->cur_y1;
    p->prev_disp = p->cur_disp;
    p->prev_valid = true;
}

/* 当前播放中的播放器（供 draw callback 访问） */
static gif_player_t *s_cur = NULL;

/* GIF draw callback：把解码行按等比缩放写入 canvas buffer */
static void gif_draw_cb(GIFDRAW *pDraw)
{
    gif_player_t *p = s_cur;
    if (!p || !p->buf) return;
    int cw = p->cw, ch = p->ch;
    int gw = p->gw, gh = p->gh;
    if (gw <= 0 || gh <= 0) return;
    uint16_t *pal = pDraw->pPalette;
    uint8_t  *src = pDraw->pPixels;
    int w = pDraw->iWidth;

    /* --- 记录本帧矩形(canvas 坐标)：disposal=2 要按此区域清背景。
     *     用帧矩形 iX/iY/iWidth/iHeight —— 这是 GIF 规范里处置的作用域。 --- */
    {
        int oyf = p->off_y + p->bob;
        int fx0 = p->off_x + pDraw->iX * p->sc_w / gw;
        int fx1 = p->off_x + (pDraw->iX + pDraw->iWidth)  * p->sc_w / gw;
        int fy0 = oyf      + pDraw->iY * p->sc_h / gh;
        int fy1 = oyf      + (pDraw->iY + pDraw->iHeight) * p->sc_h / gh;
        if (fx1 <= fx0) fx1 = fx0 + 1;
        if (fy1 <= fy0) fy1 = fy0 + 1;
        p->cur_x0 = fx0; p->cur_y0 = fy0;
        p->cur_x1 = fx1; p->cur_y1 = fy1;
        p->cur_disp = pDraw->ucDisposalMethod;
        p->cur_dirty = true;
    }

    /* GIF 原始 y → canvas y（等比缩放 + 居中 + 最近邻放大，区间整块填充）
     *
     * 注意：若按单点映射（cy = gy*sc_h/gh），放大后会有空行 → 满屏横向条纹；
     * 列方向同理。所以按 [cy0, cy1) / [cx0, cx1) 区间填充，放大后是干净色块。 */
    int gy = pDraw->iY + pDraw->y;
    int oy = p->off_y + p->bob;         /* 待机上下浮动：只偏移宠物本体 */
    int cy0 = oy + gy * p->sc_h / gh;
    int cy1 = oy + (gy + 1) * p->sc_h / gh;
    if (cy1 <= cy0) cy1 = cy0 + 1;
    if (cy1 <= 0 || cy0 >= ch) return;  /* 整行被浮出画布 */
    if (cy0 < 0)  cy0 = 0;
    if (cy1 > ch) cy1 = ch;

    int x0 = pDraw->iX;
    if (pDraw->ucHasTransparency) {
        uint8_t trans = pDraw->ucTransparent;
        for (int x = 0; x < w; x++) {
            uint8_t c = src[x];
            if (c == trans) continue;           /* 透明像素：保留背景/历史帧 */
            int gx = x0 + x;
            int cx0 = p->off_x + gx * p->sc_w / gw;
            int cx1 = p->off_x + (gx + 1) * p->sc_w / gw;
            if (cx1 <= cx0) cx1 = cx0 + 1;
            if (cx0 < 0)  cx0 = 0;
            if (cx1 > cw) cx1 = cw;
            uint16_t v = pal[c];
            for (int cy = cy0; cy < cy1; cy++) {
                uint16_t *row = &p->buf[cy * cw];
                for (int cx = cx0; cx < cx1; cx++) row[cx] = v;
            }
        }
    } else {
        for (int x = 0; x < w; x++) {
            int gx = x0 + x;
            int cx0 = p->off_x + gx * p->sc_w / gw;
            int cx1 = p->off_x + (gx + 1) * p->sc_w / gw;
            if (cx1 <= cx0) cx1 = cx0 + 1;
            if (cx0 < 0)  cx0 = 0;
            if (cx1 > cw) cx1 = cw;
            uint16_t v = pal[src[x]];
            for (int cy = cy0; cy < cy1; cy++) {
                uint16_t *row = &p->buf[cy * cw];
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

    if (p->wrap_pending) {
        /* 上一 tick 画的是**末帧** —— 现在才回绕，保证末帧也展示满一个周期。
         * (库把"最后一帧"也返回 0，若见 0 就立刻 reset，末帧会被瞬间跳过) */
        p->wrap_pending = false;
        GIF_reset(&p->gif);
        p->prev_valid = false;
        fill_background(p);
    } else if (p->prev_valid && p->prev_disp == 2) {
        /* 处置方法 2(dispose to background)：画新帧前把上一帧矩形擦回场景背景。
         * 宝可梦 GIF 里 91.6% 的帧都是这种 —— 不擦就是重影。 */
        clear_rect(p, p->prev_x0, p->prev_y0, p->prev_x1, p->prev_y1);
    }
    p->prev_valid = false;
    p->cur_dirty  = false;

    p->frame_no++;
    p->bob = p->bob_on ? BOB_TABLE[p->frame_no & 7] : 0;

    s_cur = p;
    int res = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;

    if (!p->cur_dirty) {
        /* 本帧没有任何像素输出：多半是解码/解析出错，停掉动画保留最后一帧 */
        int e = GIF_getLastError(&p->gif);
        if (e != GIF_SUCCESS) {
            GIF_LOG("playFrame err=%d, stop anim", e);
            p->playing = false;
            lv_timer_pause(t);
            return;
        }
    } else {
        commit_frame(p);
    }

    if (res == 0) {
        /* 末帧：本 tick 只登记"待回绕"，下一 tick 才 reset */
        p->wrap_pending = true;
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
    p->prev_valid = false;
    p->cur_dirty  = false;
    p->cur_disp   = 0;
    p->wrap_pending = false;
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

    /* 等比缩放：防止非正方形 GIF 被拉成正方形（动作会明显失真）。
     * 取宽/高比例中较小者，整数化后居中放置，多余区域露出场景背景。 */
    {
        int gw = p->gw, gh = p->gh;
        int s = 1000;
        if (gw > 0 && gh > 0) {
            int sx = p->cw * 1000 / gw;
            int sy = p->ch * 1000 / gh;
            s = (sx < sy) ? sx : sy;
        }
        if (s <= 0) s = 1000;
        p->sc_w = gw * s / 1000;
        p->sc_h = gh * s / 1000;
        if (p->sc_w < 1) p->sc_w = 1;
        if (p->sc_h < 1) p->sc_h = 1;
        if (p->sc_w > p->cw) p->sc_w = p->cw;
        if (p->sc_h > p->ch) p->sc_h = p->ch;
        p->off_x = (p->cw - p->sc_w) / 2;
        p->off_y = (p->ch - p->sc_h) / 2;
        GIF_LOG("fit %dx%d -> %dx%d @(%d,%d) in %dx%d",
                gw, gh, p->sc_w, p->sc_h, p->off_x, p->off_y, p->cw, p->ch);
    }

    /* 处置跟踪复位 */
    p->prev_valid   = false;
    p->cur_dirty    = false;
    p->cur_disp     = 0;
    p->wrap_pending = false;

    /* 解第一帧：能跑到这里就说明解码器可用 */
    int delay = 0;
    fill_background(p);
    p->bob = p->bob_on ? BOB_TABLE[p->frame_no & 7] : 0;
    p->frame_no++;
    p->cur_dirty = false;
    s_cur = p;
    int rc = GIF_playFrame(&p->gif, &delay, NULL);
    s_cur = NULL;
    GIF_LOG("frame1 rc=%d err=%d delay=%d free=%d", rc,
            GIF_getLastError(&p->gif), delay, (int)esp_get_free_heap_size());
    if (rc < 0) {
        /* 解码失败：保留白底但不启动定时器，至少不崩 */
        return 0;
    }
    commit_frame(p);            /* 首帧登记为"上一帧"，第二帧前才会被处置 */
    if (rc == 0) p->wrap_pending = true;
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
