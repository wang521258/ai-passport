// main/ui_sound.h —— 音频出口：UI 提示音（按键切换 / 答对 / 答错）+ 8bit 背景乐。
//
// 播放是「投递式」的：调用方只把音效编号塞进队列就返回，真正的 PCM 推送
// 由独立任务完成，不会卡住 LVGL 与按键回调。
// 背景乐由 main/bgm.h 提供数据，本任务的混音器把它与提示音叠加后一起送 I2S
// （I2S 只有一路，必须单写入者，不能两个任务各写各的）。
#pragma once

#include <stdbool.h>

typedef enum {
    UI_SND_SWITCH = 0,      // 上下切换 / 确定：清脆短音「哒」
    UI_SND_CORRECT,         // 答对：上行三音「叮-叮-叮」
    UI_SND_WRONG,           // 答错：下行两音「咚-咚」
    UI_SND_COUNT
} ui_sound_t;

// 建队列 + 播放任务。需在 bsp_audio_init() 之后调用（顺序反了会自动置为不可用）。
// 幂等。音频不可用时本模块静默降级，调用方不用判空。
void ui_sound_init(void);

// 音频通路是否可用（供 UI 提示用）。
bool ui_sound_ready(void);

// 请求播放。非阻塞；若上一个音还在放，会丢弃积压、立刻切到新音（手感更跟手）。
void ui_sound_play(ui_sound_t id);

// 背景乐开关（8bit 芯片音乐，数据见 main/bgm.h）。
// 开启时会重新声明 16kHz 采样格式 —— Audio 演示页可能改过采样率，
// 不重设的话背景乐会以错误时钟送出（变调/变速）。
// 静默期（无背景乐、无提示音）音频任务会完全停下来，把 I2S 让给别的页。
void ui_sound_bgm(bool on);
