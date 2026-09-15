// main/ui_sound.c —— UI 提示音播放（马林巴 / 三角铁音色）。
//
// 设计要点：
//   * PCM 是【单声道 16bit】的静态表（main/ui_sound_pcm.h，由 gen_sound.py 生成），
//     播放时统一走 bsp_audio_write_mono()，由 BSP 负责展开成硬件要求的双声道槽。
//   * 播放放在独立任务里：bsp_audio_write 写 DMA 会阻塞几十毫秒，
//     绝不能放在按键回调（那里持着 LVGL 锁）里做。
//   * 队列用 xQueueReset + 单条投递：连按按键时丢掉积压、立即响应最新一次，
//     避免"按十下响十下"的延迟堆积。
#include "ui_sound.h"
#include "bsp_audio.h"
#include "ui_sound_pcm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

static const char *TAG = "ui_sound";

// 音量：PCM 已归一化到 0.42~0.52 满量程，这里再压一档 → 约 3/8。
// 提示音宁小勿大（小喇叭破音比听不见更糟）。
#define SND_GAIN_NUM   3
#define SND_GAIN_DEN   8

typedef struct {
    const int16_t *pcm;
    int            n;
} clip_t;

static const clip_t CLIPS[UI_SND_COUNT] = {
    [UI_SND_SWITCH]  = { SND_SWITCH_PCM,  (int)(sizeof(SND_SWITCH_PCM)  / sizeof(int16_t)) },
    [UI_SND_CORRECT] = { SND_CORRECT_PCM, (int)(sizeof(SND_CORRECT_PCM) / sizeof(int16_t)) },
    [UI_SND_WRONG]   = { SND_WRONG_PCM,   (int)(sizeof(SND_WRONG_PCM)   / sizeof(int16_t)) },
};

static QueueHandle_t s_q;
static TaskHandle_t  s_task;
static bool          s_ready;
static int16_t      *s_buf;        // 混音/增益缓冲(堆上分配,不占内部 RAM 的 .bss)
static size_t        s_buf_cap;    // 容量(样本数)
static volatile bool  s_bgm_enabled;
static volatile bool  s_bgm_suspended;
static uint8_t        s_bgm_step;

/* 轻快的三角波旋律：无需额外音频文件，适合小喇叭且不会破音。 */
static void write_tone(int period, int samples, int amplitude)
{
    if (!s_buf || samples <= 0) return;
    if ((size_t)samples > s_buf_cap) samples = (int)s_buf_cap;
    int half = period / 2;
    for (int i = 0; i < samples; i++) {
        int x = i % period;
        int v = (x < half) ? (-amplitude + (2 * amplitude * x) / half)
                           : ( amplitude - (2 * amplitude * (x - half)) / half);
        s_buf[i] = (int16_t)v;
    }
    bsp_audio_write_mono(s_buf, (size_t)samples);
}

static void play_wrong_notice(void)
{
    /* 两个清晰的下行短音，比原来的闷响更容易分辨，音量也更高。 */
    write_tone(80, 1280, 22000);   /* 200Hz */
    write_tone(106, 1760, 21000);  /* 151Hz */
}

static void play_bgm_step(void)
{
    /* C大调五声音阶小循环：轻快、不刺耳。period 越小音越高。 */
    static const uint8_t periods[] = { 61, 68, 76, 68, 61, 76, 91, 76, 68, 61, 68, 76, 61, 91, 76, 68 };
    write_tone(periods[s_bgm_step % (sizeof(periods) / sizeof(periods[0]))], 1180, 5200);
    s_bgm_step++;
}

static void sound_task(void *arg)
{
    (void)arg;
    uint8_t id;
    for (;;) {
        if (xQueueReceive(s_q, &id, pdMS_TO_TICKS(12)) == pdTRUE) {
            if (id >= UI_SND_COUNT) continue;
            if (id == UI_SND_WRONG) { play_wrong_notice(); continue; }
            const clip_t *c = &CLIPS[id];
            if (!c->pcm || c->n <= 0) continue;
            int n = c->n;
            if ((size_t)n > s_buf_cap) n = (int)s_buf_cap;
            int gain = (id == UI_SND_CORRECT) ? 5 : 4;
            for (int i = 0; i < n; i++)
                s_buf[i] = (int16_t)(((int)c->pcm[i] * gain) / 8);
            bsp_audio_write_mono(s_buf, (size_t)n);
        } else if (s_bgm_enabled && !s_bgm_suspended) {
            play_bgm_step();
        }
    }
}

void ui_sound_init(void)
{
    if (s_task) return;

    // 音频必须先就绪；这里只声明格式，失败就静默降级（不阻断 UI）。
    if (bsp_audio_set_format(SND_PCM_RATE, 16, 1) != ESP_OK) {
        ESP_LOGW(TAG, "音频不可用 → 提示音已禁用（宠物功能不受影响）");
        return;
    }

    // 最长的那条 PCM 决定缓冲大小
    size_t need = 0;
    for (int i = 0; i < UI_SND_COUNT; i++)
        if ((size_t)CLIPS[i].n > need) need = (size_t)CLIPS[i].n;
    s_buf = malloc(need * sizeof(int16_t));
    if (!s_buf) { ESP_LOGW(TAG, "缓冲分配失败 → 提示音已禁用"); return; }
    s_buf_cap = need;

    s_q = xQueueCreate(4, sizeof(uint8_t));
    if (!s_q) { ESP_LOGW(TAG, "队列创建失败 → 提示音已禁用"); free(s_buf); s_buf = NULL; return; }

    if (xTaskCreate(sound_task, "ui_sound", 4096, NULL, 3, &s_task) != pdPASS) {
        ESP_LOGW(TAG, "任务创建失败 → 提示音已禁用");
        vQueueDelete(s_q); s_q = NULL; free(s_buf); s_buf = NULL; s_task = NULL;
        return;
    }
    s_ready = true;
    ESP_LOGI(TAG, "提示音就绪(%dHz, 切换/答对/答错 三音)", SND_PCM_RATE);
}

bool ui_sound_ready(void) { return s_ready; }

void ui_sound_play(ui_sound_t id)
{
    if (!s_ready || !s_q) return;
    uint8_t v = (uint8_t)id;
    xQueueReset(s_q);                 // 丢积压：保证手感跟手
    xQueueSend(s_q, &v, 0);
}

void ui_sound_bgm(bool enabled)
{
    s_bgm_enabled = enabled;
    if (!enabled) s_bgm_step = 0;
}

void ui_sound_bgm_suspend(bool suspended)
{
    s_bgm_suspended = suspended;
}
