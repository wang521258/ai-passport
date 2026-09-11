// main/bgm.c —— 8-bit 芯片音乐合成器（Game Boy / NES 听感）。
//
// 设计要点：
//   * 【不占 flash】只存曲谱（main/bgm_score.h，384 字节），方波在设备上实时算。
//     16 秒 16kHz 的 PCM 要 512KB，塞进 3.4MB 的 app 分区太亏。
//   * 【整数合成】相位用 Q16 定点累加，查 BGM_FREQ_Q16 表拿增量；
//     包络用 0..65536 的整数，逐样本线性衰减到各声部的"地板值"。
//     全程无浮点，单样本成本约十几条指令 —— 24000 样本/秒 完全不是负担。
//   * 【起音不爆】每次重触发把相位归零 + 包络回满 + 一个极短的衰减段，
//     这是芯片音乐"颗粒感"的来源，也顺带避免了波形中途被切断的爆音。
//   * 【TIE 连音】同音跨步（如四分音符 = 两个 16 分步）不重触发，
//     相位与包络都接着走，听感是连贯的长音而不是"啊啊"两下。
//
// 音量平衡：提示音峰值约 16~19% 满量程，背景乐压到 ~8%，
// 保证按键反馈永远盖得住背景乐。想调大小只改 BGM_GAIN 一个数。
#include "bgm.h"
#include "bgm_score.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "bgm";

/* ---------------------------------------------------------------- 音量 */
#define BGM_GAIN  12      /* 背景乐总音量(%)。提示音是 2~2.5 倍响度，别调太高 */

#define MIX_LEAD  42      /* 各声部相对音量(%)，合计约 108（未归一） */
#define MIX_BASS  30
#define MIX_ARP   16
#define MIX_SNARE 20
#define MIX_HAT    7

/* 占空比(0..255)：方波的"音色旋钮"，越窄越尖、越像老主机 */
#define DUTY_LEAD 128     /* 50%   —— 主旋律，饱满 */
#define DUTY_BASS  64     /* 25%   —— 低音，有木质味 */
#define DUTY_ARP   32     /* 12.5% —— 琶音，尖锐闪亮 */

/* 包络：起点都是满值，按各自的速率衰减到"地板"后保持（模拟 NES 的
   attack→decay→sustain），地板不同让三个声部的尾音长度自然区分开 */
#define ENV_FLOOR_LEAD 43000          /* 66%：旋律尾音收得干净 */
#define ENV_FLOOR_BASS 52000          /* 79%：低音铺底要撑住 */
#define ENV_FLOOR_ARP   8000          /* 12%：琶音只要一点点闪光 */
#define ENV_DEC_LEAD   120            /* 每样本衰减量 → 约 15ms 落到地板 */
#define ENV_DEC_BASS   110
#define ENV_DEC_ARP    520            /* 琶音衰减最快 → 像拨弦 */
#define ENV_SNARE_N   1500            /* 军鼓尾巴 ~94ms */
#define ENV_HAT_N      330            /* 踩镲 ~21ms */

/* ---------------------------------------------------------------- 状态 */
typedef struct {
    uint32_t ph;       /* Q16 相位，一个完整周期 = 65536 */
    uint16_t inc;      /* Q16 每样本相位增量 */
    uint8_t  live;     /* 本步是否发声 */
} voice_t;

static voice_t s_v[3];
static int32_t s_env[3];        /* 当前包络 0..65536 */
static int32_t s_flo[3];        /* 该声部的地板值 */
static int32_t s_edec[3];       /* 该声部的每样本衰减量 */

static int32_t  s_nenv, s_ndec, s_nmix;   /* 噪声通道（打击乐） */
static uint32_t s_lfsr = 0xACE1u;         /* 15bit 线性反馈移位寄存器 */

static int  s_step;             /* 当前 16 分音符序号 0..BGM_STEPS-1 */
static int  s_left;             /* 当前步还剩多少样本 */

static volatile bool s_on;
static volatile bool s_suspend;  /* 临时静音但保留进度（进题板时用） */

/* ---------------------------------------------------------------- 实现 */
static const uint8_t DUTY[3] = { DUTY_LEAD, DUTY_BASS, DUTY_ARP };
static const uint8_t MIXV[3] = { MIX_LEAD, MIX_BASS, MIX_ARP };

static void voice_load(int i, uint8_t note, int32_t floor_v, int32_t dec)
{
    if (note == BGM_REST) { s_v[i].live = 0; return; }
    if (note != BGM_TIE) {                 /* 重触发：相位归零、包络回满 */
        s_v[i].inc = (note >= 36 && note <= 96) ? BGM_FREQ_Q16[note - 36] : 0;
        s_v[i].ph  = 0;
        s_env[i]   = 65536;
        s_flo[i]   = floor_v;
    }
    s_v[i].live = 1;
    s_edec[i]   = dec;
}

static void step_load(void)
{
    const int i = s_step;
    const int bar = i >> 4;                /* 每小节 16 个 16 分音符 */
    const int st  = i & 15;

    voice_load(0, BGM_LEAD[i], ENV_FLOOR_LEAD, ENV_DEC_LEAD);
    voice_load(1, BGM_BASS[i], ENV_FLOOR_BASS, ENV_DEC_BASS);
    voice_load(2, BGM_ARP[i],  ENV_FLOOR_ARP,  ENV_DEC_ARP);

    if (BGM_SNARE[bar] & (1u << st)) {
        s_nenv = 65536; s_ndec = 65536 / ENV_SNARE_N; s_nmix = MIX_SNARE;
    } else if (BGM_HAT[bar] & (1u << st)) {
        s_nenv = 26000; s_ndec = 26000 / ENV_HAT_N;  s_nmix = MIX_HAT;
    }
    /* 没命中的步不动噪声状态 → 上一击的尾巴自然衰减，不用额外处理 */

    s_left = BGM_STEP_SAMPLES;
    s_step = (i + 1) % BGM_STEPS;
}

static inline int32_t noise_bit(void)
{
    s_lfsr = (s_lfsr >> 1) ^ ((s_lfsr & 1u) ? 0x6000u : 0u);
    return (s_lfsr & 1u) ? 32767 : -32767;
}

void bgm_init(void)
{
    memset(s_v, 0, sizeof(s_v));
    memset(s_env, 0, sizeof(s_env));
    s_nenv = 0; s_ndec = 1; s_nmix = 0;
    s_step = 0; s_left = 0;
    ESP_LOGI(TAG, "背景乐就绪: %d 步 x %d 样本 = %.1f 秒循环, %dHz",
             BGM_STEPS, BGM_STEP_SAMPLES,
             (double)BGM_STEPS * BGM_STEP_SAMPLES / (double)BGM_RATE, BGM_RATE);
}

void bgm_set_on(bool on)
{
    if (s_on == on) return;
    s_on = on;
    s_suspend = false;                     /* 显式开/关都以"该响/该停"为准 */
    if (on) {                              /* 从头开始，别接着上次的半拍子 */
        s_step = 0; s_left = 0;
        memset(s_v, 0, sizeof(s_v));
        memset(s_env, 0, sizeof(s_env));
        s_nenv = 0;
    }
    ESP_LOGI(TAG, "背景乐%s", on ? "开始" : "停止");
}

bool bgm_is_on(void) { return s_on; }

void bgm_set_suspend(bool sus)
{
    if (s_suspend == sus) return;
    s_suspend = sus;
    if (s_on) ESP_LOGI(TAG, "背景乐%s", sus ? "静音(保留进度)" : "续播");
}

bool bgm_is_suspended(void) { return s_suspend; }

void bgm_render(int16_t *out, int n)
{
    if (!s_on || s_suspend) { memset(out, 0, (size_t)n * sizeof(int16_t)); return; }

    for (int k = 0; k < n; k++) {
        if (s_left <= 0) step_load();

        int32_t acc = 0;
        for (int i = 0; i < 3; i++) {
            voice_t *v = &s_v[i];
            if (!v->live || !v->inc) continue;
            v->ph += v->inc;
            int32_t w = ((v->ph >> 8) < DUTY[i]) ? 32767 : -32767;
            /* w>>8 = ±127, env>>8 = 0..256 → 乘积 ±32512，落在 int16 量级 */
            acc += ((w >> 8) * (s_env[i] >> 8)) * MIXV[i];
            if (s_env[i] > s_flo[i]) {
                s_env[i] -= s_edec[i];
                if (s_env[i] < s_flo[i]) s_env[i] = s_flo[i];
            }
        }
        if (s_nenv > 0) {
            acc += ((noise_bit() >> 8) * (s_nenv >> 8)) * s_nmix;
            s_nenv -= s_ndec;
            if (s_nenv < 0) s_nenv = 0;
        }

        acc /= 100;                        /* 声部权重归一 */
        acc = acc * BGM_GAIN / 100;        /* 总音量 */
        if (acc > 32767) acc = 32767;
        else if (acc < -32768) acc = -32768;
        out[k] = (int16_t)acc;

        s_left--;
    }
}
