// components/bsp/include/bsp_audio.h
// 音频输出/输入。两条通路按板级宏 BSP_AUDIO_ES8311 选择:
//   NS4168(纯 I2S 功放,本板实际使用) / ES8311(I2C 控制口 + I2S 全双工)。
// 对上层的 API 一致,调用方不需要区分。
#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

// 初始化音频通路并打开功放使能。幂等。
esp_err_t bsp_audio_init(void);

// 设置采样格式。
//   NS4168 通路:仅支持 16bit;I2S 硬件固定走双声道槽,单声道数据请用
//                bsp_audio_write_mono() 输出(内部自动复制成 L=R)。
//   ES8311 通路:格式变化时会先 close 再 open。
//               ⚠ esp_codec_dev_open() 在 codec【已打开】时会直接返回 OK 且
//               【不重新配置采样率】。若不先 close,16kHz 播完再播 8kHz 会以 16k
//               时钟送出 —— 音调和速度都快一倍。
esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch);

// 播放 / 录音。bytes 为字节数(16bit 时 = 采样数 x 2)。
// ⚠ bsp_audio_write() 要求数据已按硬件槽格式排好(本板=双声道交错);
//   手上有单声道 PCM 时请用 bsp_audio_write_mono()。
esp_err_t bsp_audio_write(const void *pcm, size_t bytes);
esp_err_t bsp_audio_read(void *pcm, size_t bytes);

// 写单声道 PCM(内部复制成 L=R 后交给硬件)。提示音/语音都用这个。
esp_err_t bsp_audio_write_mono(const int16_t *pcm, size_t samples);

// 输出音量 0..100(%)。NS4168 通路无硬件音量,为空实现(软件增益见 ui_sound)。
void bsp_audio_set_volume(uint8_t percent);
