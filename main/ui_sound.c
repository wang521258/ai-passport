// main/ui_sound.c —— 音频混音器：8-bit 背景乐 + UI 提示音。
//
// 设计要点：
//   * 【单写入者】I2S 只有一路，绝不能让两个任务同时写（会互相踩 DMA 缓冲）。
//     本任务同时负责 BGM 与提示音：先把 BGM 渲染进缓冲，再把提示音的样本
//     叠加上去，然后一次性推给 I2S。BGM 不会因为按了键就卡顿。
//   * 【固定块长】每次生成 CHUNK(=240) 个样本 = 15ms，正好是 BSP 的 DMA 帧长。
//     块太大 → 按键音延迟明显；块太小 → I2S 写入调用过频、白费 CPU。
//   * 【静默期让路】背景乐关着、也没有提示音在播时，本任务阻塞等待，
//     一个字节都不往 I2S 写：省 CPU，也把输出通道让给 Audio 演示页
//     （那边要用 I2S 做录放测试，两个写入者会互相踩）。
//   * 【丢积压】连按按键时 xQueueReset + 排干队列，永远只放最新那一声，
//     手感跟手，不会"按十下响十下"越积越迟。
//   * PCM 表是【单声道 16bit】（main/ui_sound_pcm.h，由 gen_sound.py 生成），
//     由 BSP 负责展开成硬件要求的双声道槽。
#include "ui_sound.h"
#include "bgm.h"
#include "bsp_audio.h"
#include "ui_sound_pcm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdlib.h>

#if BGM_RATE != SND_PCM_RATE
#error "BGM_RATE 必须与 SND_PCM_RATE 相同，否则背景乐会变调"
#endif

static const char *TAG = "ui_sound";

// 音量：PCM 已归一化到 0.42~0.52 满量程，这里再压一档 → 约 3/8。
// 提示音宁小勿大（小喇叭破音比听不见更糟）。
#define SND_GAIN_NUM   3
#define SND_GAIN_DEN   8

#define CHUNK  240          /* 15ms @16kHz，与 BSP dma_frame_num 一致 */

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
static int16_t      *s_mix;        // 混音缓冲（堆上分配，不占 .bss）

static void sound_task(void *arg)
{
    (void)arg;
    uint8_t id;
    const clip_t *cur = NULL;      // 正在播的提示音
    int           cur_i = 0;

    for (;;) {
        // 1) 取请求。
        //    静默期（没放背景乐、也没提示音在播）→ 阻塞等待，一个字节都不写：
        //      既省 CPU，也把 I2S 让给 Audio 演示页（那边要独占通道做录放）。
        //      timeout 用来兜住"阻塞期间背景乐被打开"，50ms 的启动延迟听不出来。
        //    背景乐在跑时 → 必须持续供数，只能非阻塞排干队列（永远响应最新那声）。
        if (!bgm_is_on() && !cur) {
            if (xQueueReceive(s_q, &id, pdMS_TO_TICKS(50)) != pdTRUE) continue;
            if (id < UI_SND_COUNT && CLIPS[id].pcm && CLIPS[id].n > 0) {
                cur = &CLIPS[id]; cur_i = 0;
            }
            if (!cur) continue;
        } else {
            while (xQueueReceive(s_q, &id, 0) == pdTRUE) {
                if (id < UI_SND_COUNT && CLIPS[id].pcm && CLIPS[id].n > 0) {
                    cur = &CLIPS[id]; cur_i = 0;
                }
            }
        }

        // 2) 背景乐打底
        bgm_render(s_mix, CHUNK);

        // 3) 提示音叠加（带增益，并做限幅防破音）
        if (cur) {
            for (int i = 0; i < CHUNK && cur_i < cur->n; i++) {
                int32_t v = s_mix[i] +
                            ((int32_t)cur->pcm[cur_i++] * SND_GAIN_NUM) / SND_GAIN_DEN;
                s_mix[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            }
            if (cur_i >= cur->n) cur = NULL;
        }

        // 4) 推给 I2S（阻塞式 → 天然按实时速率节流，无需自己 delay）
        bsp_audio_write_mono(s_mix, CHUNK);
    }
}

void ui_sound_init(void)
{
    if (s_task) return;

    // 音频必须先就绪；这里只声明格式，失败就静默降级（不阻断 UI）。
    if (bsp_audio_set_format(SND_PCM_RATE, 16, 1) != ESP_OK) {
        ESP_LOGW(TAG, "音频不可用 → 声音已禁用（宠物功能不受影响）");
        return;
    }

    s_mix = malloc(CHUNK * sizeof(int16_t));
    if (!s_mix) { ESP_LOGW(TAG, "混音缓冲分配失败 → 声音已禁用"); return; }

    s_q = xQueueCreate(4, sizeof(uint8_t));
    if (!s_q) { ESP_LOGW(TAG, "队列创建失败 → 声音已禁用"); free(s_mix); s_mix = NULL; return; }

    bgm_init();

    if (xTaskCreate(sound_task, "ui_sound", 4096, NULL, 3, &s_task) != pdPASS) {
        ESP_LOGW(TAG, "任务创建失败 → 声音已禁用");
        vQueueDelete(s_q); s_q = NULL; free(s_mix); s_mix = NULL; s_task = NULL;
        return;
    }
    s_ready = true;
    ESP_LOGI(TAG, "声音就绪(%dHz, 切换/答对/答错 + 8bit 背景乐)", SND_PCM_RATE);
}

bool ui_sound_ready(void) { return s_ready; }

void ui_sound_play(ui_sound_t id)
{
    if (!s_ready || !s_q) return;
    uint8_t v = (uint8_t)id;
    xQueueReset(s_q);                 // 丢积压：保证手感跟手
    xQueueSend(s_q, &v, 0);
}

void ui_sound_bgm(bool on)
{
    if (!s_ready) return;
    if (on) {
        /* Audio 演示页可能把采样率改成了别的值；这里重新声明 16kHz。
           （BSP 内部同值会直接返回，等于只在需要时才重配 I2S 时钟。） */
        bsp_audio_set_format(SND_PCM_RATE, 16, 1);
    }
    bgm_set_on(on);
}
