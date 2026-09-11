// components/bsp/src/bsp_audio.c
// 两条音频通路,按板级宏 BSP_AUDIO_ES8311 二选一:
//   1 = ES8311(I2C 控制口 + I2S 全双工) —— 移植自 trae_card 的 audio_es8311.c
//   0 = NS4168(纯 I2S 功放,无 I2C 控制口) —— 本板 atk-dnesp32s3-box 实际使用
// 两者对上层的 API 完全一致,调用方不需要区分。
#include "bsp_audio.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "bsp_audio";

// ============================================================================
// 通路 B:NS4168 —— 纯 I2S DAC + 功放,只需把 PCM 顺 I2S 推出去
// ============================================================================
#if !BSP_AUDIO_ES8311

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "bsp_xio.h"        // 功放使能 P0_5

static i2s_chan_handle_t s_tx;
static uint32_t          s_rate;                 // 当前 I2S 时钟采样率
static int16_t          *s_mono_scratch;         // write_mono 的展开缓冲
static size_t            s_mono_scratch_cap;     // 容量(立体声样本数)

esp_err_t bsp_audio_init(void) {
    if (s_tx) return ESP_OK;

    i2s_chan_config_t chan = {
        .id = BSP_I2S_PORT,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    esp_err_t e = i2s_new_channel(&chan, &s_tx, NULL);   // 本板只放音,不占 DIN
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel 失败: %s", esp_err_to_name(e));
        s_tx = NULL;
        return e;
    }

    i2s_std_config_t std = {
        .clk_cfg = {
            .sample_rate_hz = BSP_AUDIO_SAMPLE_RATE,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            // ⚠ 必须 STEREO + BOTH:本板官方板级(ATK_NoAudioCodecDuplex)即如此。
            //   若改单声道槽,功放取的可能是另一半无声槽 → 干脆没声。
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = true,
            .big_endian     = false,
            .bit_order_lsb  = false,
        },
        .gpio_cfg = {
            .mclk = (gpio_num_t)BSP_I2S_MCLK,     // -1:本板 codec 不需要 MCLK
            .bclk = (gpio_num_t)BSP_I2S_BCLK,
            .ws   = (gpio_num_t)BSP_I2S_WS,
            .dout = (gpio_num_t)BSP_I2S_DOUT,
            .din  = GPIO_NUM_NC,                  // 只放音
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    if ((e = i2s_channel_init_std_mode(s_tx, &std)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s tx 初始化失败: %s", esp_err_to_name(e));
        i2s_del_channel(s_tx); s_tx = NULL;
        return e;
    }
    if ((e = i2s_channel_enable(s_tx)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s tx 使能失败: %s", esp_err_to_name(e));
        i2s_del_channel(s_tx); s_tx = NULL;
        return e;
    }
    s_rate = BSP_AUDIO_SAMPLE_RATE;

    // 功放使能:XL9555 P0_5 高电平(与官方板级 SetOutputState(5,1) 一致)
    bsp_xio_set_pa(1);

    ESP_LOGI(TAG, "NS4168 就绪(I2S TX: BCLK=GPIO%d WS=GPIO%d DOUT=GPIO%d, %luHz/16bit/stereo)",
             BSP_I2S_BCLK, BSP_I2S_WS, BSP_I2S_DOUT, (unsigned long)s_rate);
    return ESP_OK;
}

esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch) {
    (void)ch;                       // 硬件固定走双声道槽,由 write_mono 负责展开
    if (!s_tx) return ESP_ERR_INVALID_STATE;
    if (bits != 16) { ESP_LOGW(TAG, "仅支持 16bit,收到 %u", bits); return ESP_ERR_NOT_SUPPORTED; }
    if (hz == s_rate) return ESP_OK;

    i2s_channel_disable(s_tx);
    i2s_std_clk_config_t clk = {
        .sample_rate_hz = hz,
        .clk_src        = I2S_CLK_SRC_DEFAULT,
        .ext_clk_freq_hz = 0,
        .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
    };
    esp_err_t e = i2s_channel_reconfig_std_clock(s_tx, &clk);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "重配采样率失败: %s", esp_err_to_name(e));
        i2s_channel_enable(s_tx);
        return e;
    }
    s_rate = hz;
    i2s_channel_enable(s_tx);
    ESP_LOGI(TAG, "I2S 采样率改为 %luHz", (unsigned long)hz);
    return ESP_OK;
}

esp_err_t bsp_audio_write(const void *pcm, size_t bytes) {
    if (!s_tx) return ESP_ERR_INVALID_STATE;
    size_t done = 0;
    esp_err_t e = i2s_channel_write(s_tx, pcm, bytes, &done, portMAX_DELAY);
    return e == ESP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t bsp_audio_read(void *pcm, size_t bytes) {
    (void)pcm; (void)bytes;
    return ESP_ERR_NOT_SUPPORTED;   // NS4168 是纯输出功放,本板无录音通路
}

// 单声道 PCM → 双声道槽(L=R)后写出。缓冲区按需增长(用堆,PSRAM 充足)。
esp_err_t bsp_audio_write_mono(const int16_t *pcm, size_t samples) {
    if (!s_tx) return ESP_ERR_INVALID_STATE;
    if (samples == 0) return ESP_OK;

    size_t need = samples * 2;      // 立体声样本数
    if (need > s_mono_scratch_cap) {
        int16_t *p = realloc(s_mono_scratch, need * sizeof(int16_t));
        if (!p) { ESP_LOGE(TAG, "展开缓冲分配失败(%u 样本)", (unsigned)need); return ESP_ERR_NO_MEM; }
        s_mono_scratch = p; s_mono_scratch_cap = need;
    }
    for (size_t i = 0; i < samples; i++) {
        s_mono_scratch[2 * i] = pcm[i];
        s_mono_scratch[2 * i + 1] = pcm[i];
    }
    return bsp_audio_write(s_mono_scratch, need * sizeof(int16_t));
}

void bsp_audio_set_volume(uint8_t percent) { (void)percent; /* 软件增益在 ui_sound 内 */ }

// ============================================================================
// 通路 A:ES8311(I2C 控制口 + I2S 全双工)
// ============================================================================
#else

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "driver/i2s_std.h"

static esp_codec_dev_handle_t s_dev;
static i2s_chan_handle_t      s_tx, s_rx;
// 记录当前已打开的格式,用于判断"要不要 close 重开"(见头文件里的坑说明)。
static uint32_t s_hz;
static uint8_t  s_bits, s_ch;
static bool     s_opened;

static esp_err_t i2s_full_duplex_init(void) {
    i2s_chan_config_t chan = {
        .id = BSP_I2S_PORT,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    esp_err_t e = i2s_new_channel(&chan, &s_tx, &s_rx);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel 失败: %s", esp_err_to_name(e)); return e; }

    // 这里的采样率只用于建通道;实际速率由 esp_codec_dev_open() 按需重配。
    i2s_std_config_t std = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = (gpio_num_t)BSP_I2S_MCLK, .bclk = (gpio_num_t)BSP_I2S_BCLK,
            .ws = (gpio_num_t)BSP_I2S_WS,
            .dout = (gpio_num_t)BSP_I2S_DOUT, .din = (gpio_num_t)BSP_I2S_DIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    if ((e = i2s_channel_init_std_mode(s_tx, &std)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s tx 初始化失败: %s", esp_err_to_name(e)); return e;
    }
    if ((e = i2s_channel_init_std_mode(s_rx, &std)) != ESP_OK) {
        ESP_LOGE(TAG, "i2s rx 初始化失败: %s", esp_err_to_name(e)); return e;
    }
    // esp_codec_dev_open 内部重配前会先 i2s_channel_disable,而 disable 要求通道处于
    // RUNNING;刚 init 的通道是 READY,会打一条 "channel has not been enabled yet" 错误日志。
    // 这里先 enable 一次让那次 disable 合法(此时 codec 未配,不出声)。
    i2s_channel_enable(s_tx);
    i2s_channel_enable(s_rx);
    return ESP_OK;
}

esp_err_t bsp_audio_init(void) {
    if (s_dev) return ESP_OK;

    esp_err_t e = bsp_i2c_init();
    if (e != ESP_OK) return e;

    const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&(audio_codec_i2c_cfg_t){
        .port = BSP_I2C_PORT,
        .addr = BSP_I2C_ES8311_ADDR << 1,   // 该接口要 8 位地址形式
        .bus_handle = bsp_i2c_bus(),
    });
    if (!ctrl) {
        ESP_LOGE(TAG, "ES8311 控制口创建失败 —— 确认 0x%02X 是否应答;检查 SDA=GPIO%d / SCL=GPIO%d",
                 BSP_I2C_ES8311_ADDR, BSP_I2C_SDA, BSP_I2C_SCL);
        return ESP_FAIL;
    }

    if ((e = i2s_full_duplex_init()) != ESP_OK) return e;

    const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&(audio_codec_i2s_cfg_t){
        .port = BSP_I2S_PORT, .tx_handle = s_tx, .rx_handle = s_rx,
    });
    if (!data) { ESP_LOGE(TAG, "I2S 数据口创建失败"); return ESP_FAIL; }

    const audio_codec_if_t *codec = es8311_codec_new(&(es8311_codec_cfg_t){
        .ctrl_if     = ctrl,
        .gpio_if     = audio_codec_new_gpio(),
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin      = BSP_I2S_PA_CTRL,
        .pa_reverted = false,
        .master_mode = false,          // MCU I2S 为 master,codec 为 slave
        .use_mclk    = true,
        .hw_gain     = { .pa_voltage = 5.0f, .codec_dac_voltage = 3.3f },
        // ⚠ 单声道纯麦克风录音必须为 true。false 会让驱动写 REG44=0x58 进入
        //   ADCL+DACR 参考模式,单声道读到的那一路是 DAC 参考 → 【录音恒为 0】。
        .no_dac_ref  = true,
    });
    if (!codec) { ESP_LOGE(TAG, "es8311_codec_new 失败"); return ESP_FAIL; }

    s_dev = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec,
        .data_if  = data,
    });
    if (!s_dev) { ESP_LOGE(TAG, "esp_codec_dev_new 失败"); return ESP_FAIL; }

    ESP_LOGI(TAG, "ES8311 就绪");
    return ESP_OK;
}

esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (s_opened && s_hz == hz && s_bits == bits && s_ch == ch) return ESP_OK;   // 同格式复用

    if (s_opened) {
        esp_codec_dev_close(s_dev);
        s_opened = false;
        // close 把 I2S 通道退回 READY,而接下来的 open 内部又会 disable 一次 →
        // 会打 "channel has not been enabled yet"。补一次 enable 让它合法。
        if (s_tx) i2s_channel_enable(s_tx);
        if (s_rx) i2s_channel_enable(s_rx);
    }

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = bits,
        .channel = ch,
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
        .sample_rate = hz,
        .mclk_multiple = 0,          // 0 → 驱动按默认 256xfs 取 MCLK
    };
    int r = esp_codec_dev_open(s_dev, &fs);
    if (r != 0) { ESP_LOGE(TAG, "esp_codec_dev_open 失败: %d", r); return ESP_FAIL; }

    // ⚠ open 之后【不要】手动覆写 ES8311 的时钟分频寄存器(REG01~06):
    //   驱动已按采样率与 MCLK 精确算好,覆写会导致 ADC/DAC 时序错乱、录音回放全是杂音。
    //   这里只设麦克风模拟 PGA 增益。
    esp_codec_dev_set_in_gain(s_dev, 30.0f);

    s_opened = true; s_hz = hz; s_bits = bits; s_ch = ch;
    ESP_LOGI(TAG, "codec 打开 %luHz/%ubit/%uch", (unsigned long)hz, bits, ch);
    return ESP_OK;
}

esp_err_t bsp_audio_write(const void *pcm, size_t bytes) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return esp_codec_dev_write(s_dev, (void *)pcm, bytes) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t bsp_audio_write_mono(const int16_t *pcm, size_t samples) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (!samples) return ESP_OK;
    if (s_ch == 2) {
        // 已是双声道通道 → 复制成 L=R 再交给 codec
        int16_t *tmp = malloc(samples * 2 * sizeof(int16_t));
        if (!tmp) return ESP_ERR_NO_MEM;
        for (size_t i = 0; i < samples; i++) { tmp[2 * i] = pcm[i]; tmp[2 * i + 1] = pcm[i]; }
        esp_err_t e = esp_codec_dev_write(s_dev, tmp, samples * 2 * sizeof(int16_t)) == 0 ? ESP_OK : ESP_FAIL;
        free(tmp);
        return e;
    }
    return bsp_audio_write(pcm, samples * sizeof(int16_t));
}

esp_err_t bsp_audio_read(void *pcm, size_t bytes) {
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return esp_codec_dev_read(s_dev, pcm, bytes) == 0 ? ESP_OK : ESP_FAIL;
}

void bsp_audio_set_volume(uint8_t percent) {
    if (s_dev) esp_codec_dev_set_out_vol(s_dev, percent);
}

#endif  // BSP_AUDIO_ES8311
