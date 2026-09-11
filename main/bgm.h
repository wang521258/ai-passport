// main/bgm.h —— 8-bit 芯片音乐背景乐（运行时合成，不占 flash）。
//
// 数据在 main/bgm_score.h（由 gen_bgm.py 生成），合成器在 main/bgm.c。
// 混音与 I2S 输出由 main/ui_sound.c 统一负责，本模块只管"给我 n 个样本"。
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 合成采样率。必须与 gen_bgm.py 的 RATE 一致 ——
 * bgm_score.h 里也用同一个宏名定义，两处值不同编译器会直接报重复定义，
 * 等于一个免费的防呆检查。 */
#define BGM_RATE 16000

// 复位内部状态并打印一行就绪日志（由 ui_sound_init 调用）。
void bgm_init(void);

// 开/关背景乐。开启时会从第 1 小节重新开始（不会接上次的半拍子）。
void bgm_set_on(bool on);
bool bgm_is_on(void);

// 渲染 n 个【单声道 16bit】样本到 out；关闭时填 0（静音）。
// 由音频任务按固定块长调用，请勿在其他地方调用。
void bgm_render(int16_t *out, int n);
