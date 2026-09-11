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

static void sound_task(void *arg)
{
    (void)arg;
    uint8_t id;
    for (;;) {
        if (xQueueReceive(s_q, &id, portMAX_DELAY) != pdTRUE) continue;
        if (id >= UI_SND_COUNT) continue;
        const clip_t *c = &CLIPS[id];
        if (!c->pcm || c->n <= 0) continue;

        // 软件增益逐样本缩放后直接推给 BSP（BSP 内部展开成双声道槽）
        int n = c->n;
        if ((size_t)n > s_buf_cap) n = (int)s_buf_cap;
        for (int i = 0; i < n; i++)
            s_buf[i] = (int16_t)(((int)c->pcm[i] * SND_GAIN_NUM) / SND_GAIN_DEN);
        bsp_audio_write_mono(s_buf, (size_t)n);
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
