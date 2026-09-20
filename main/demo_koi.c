// main/demo_koi.c —— 「锦鲤池」240x320 真机版（第一版：先在板上跑起来）
//
// 血统：这份代码是 240x320 网页版《锦鲤池》的**固件等效移植**。
//   几何 / 运动 / 涟漪参数与网页版逐式对齐；改一边必须改另一边。
//   网页版的渲染数学另有一份 Python 参考实现（工作区 _tools/koi_sim.py）：
//   形状、配色、神态先在 PC 上出图验过，再逐式转写到这里 ——
//   因为本机没有 ESP-IDF 工具链，云编译一趟几分钟，几何错误必须上板之前消灭。
//
// 关键取舍（都是被 C3 的现实逼出来的，别当成"简化"改回去）：
//   · 帧缓冲 = LVGL canvas 240x320 RGB565，静态 153.6KB。
//     ★ C3 无 PSRAM，这是本页最大的单项内存，已按 153,600 B 记账。
//   · **只重画脏区**：每个会动的对象报自己的包围盒 → 合并成矩形表 →
//     所有绘制裁到矩形里 → lv_obj_invalidate_area 只让这些像素走 blend + SPI。
//     全屏重传是 30.7ms SPI（物理下限），只推 15~25% 才留得出 24fps 的余量。
//   · 三角函数走 1024 项查表 + 线性插值：C3 是 RV32IMC，**没有 FPU**，
//     一次 sinf 上千周期；波带每帧要上千次正弦，照抄网页版会直接掉到个位数帧率。
//   · 光栅化 = 世界空间水平扫描线 + Q8 定点 + 2 条子扫描线 AA。
//     仿射变换保直线，所以"局部形状 → 世界多边形"这一步是精确的。
//   · 水面 = 每行渐变 + 径向中心光晕 + 静态波光点，烘成 (行, 光晕级) 二维表；
//     逐像素只剩一次查表 + 一次写入（否则每像素一次开方，76800 次开方要十几毫秒）。
//   · 波光点**不做逐帧呼吸**：44 个点散在全屏，逐帧标脏会把脏矩形彻底打散成整屏，
//     脏区优化当场失效。这是与网页版的一处有意差异（网页版是 canvas 全量重画，
//     这 44 个点本来就不进脏区账）。真机上它们作为水面的一部分常驻。
//
// 按键（沿用仓库约定；长按确定返回菜单由 main.c 统一拦截）：
//   上   短按 = 投喂（14 颗饲料 + 8 滴雨点）
//   下   短按 = 拍水（3 道同心涟漪 + 鱼受惊四散）
//   确定 短按 = 昼夜切换（1 秒过渡）
//
// 已经做了的（第 25 轮补上荷叶与浮萍）：巡检 / 投喂抢食 / 拍水三圈同心涟漪 /
//   昼夜两套端点配色 / **荷叶与伴生浮萍**。
// 其余**故意没做**：首页整屏设计稿 / 开局三屏（选条数图章、分色）/
//   成长与存档 / 夜间月相。别以为漏了，是排在后面。
//
// 与 docs/development/ai-guide.md 的一处**有意偏离**并说明理由：
//   本页是全屏画面，没有套 ui_pixel 主题（天空底 + 标题牌 + 吉祥物）——
//   那些会被 240x320 的画面整个盖住，留着只是每帧白画一遍。同理也没画右上角电量：
//   本页画面来自王总给的整屏设计稿，里面没有电量，而"屏内保持干净"是硬口径。
#include "demo.h"
#include "bsp_display.h"
#include "bsp_pins.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
/* ★★ 第 35 轮 · 热路径常驻 IRAM 的属性宏。
   C3 跑在**外部 flash**（8MB XMC），取指要过 cache；把"每帧进几千上万次"的小函数
   搬进 IRAM，省掉 cache miss 的等待 —— **纯提速，一个像素都不改**。
   台架（x86）没有 IRAM 这个概念，置空即可，一行都不影响。
   ⚠️ IRAM 有容量上限（IDF 的 .iram1 段），所以只放真正热的：
      fill_poly / span_draw / stroke_line / stroke_pts / water_rect 这几个，
      编译后合起来几 KB，而每帧要进它们十几万次。
   ⚠️ 别把 koi_draw / lily_paint 也搬进来 —— 它们几百行，占地方、又未必省得回来。 */
#ifdef KOI_HOST_PROBE
  #define KOI_HOT
#else
  #include "esp_attr.h"
  #define KOI_HOT IRAM_ATTR
#endif

/* ★★ 第 35 轮 · lrintf 的定点替代（与 FE_TONEAREST「就近取偶」逐位一致）
   为什么要换：C3 是 **RV32IMC —— 没有 F 扩展**，float 运算和 lrintf 都是软件库。
   lrintf 要先读当前舍入模式、再判范围与次正规，单次上百周期；而本页的调用点
   全都满足「|v| < 2^23、非 NaN/Inf、非次正规」（Q8 世界坐标 ≤ 6 万），
   按 IEEE754 位域手工做只要一次移位 + 一次比较 —— 二十来周期。
   调用量：台架量到 stroke_line 一处就 5251 次/帧，加上 to_q8 / lily_xf 约 8000+。
   ⚠️ 名字叫 rne_f2i 而不是 lrintf_：「受限替代，不是通用实现」。
   ⚠️ 台架用**逐字节对拍**验证它 —— x86 的 lrintf 同样是就近取偶，所以
      "换完之后 20 个状态的帧缓冲一字不差"就是等价性的证明（不靠推导靠证据）。 */
static inline int32_t rne_f2i(float v)
{
    union { float f; uint32_t u; } b;
    b.f = v;
    uint32_t u = b.u;
    int      e = (int)((u >> 23) & 0xFFu) - 127;      /* 2 的指数 */
    if (e < 0) {                                      /* |v| < 1 → 结果 0 或 ±1 */
        if (e != -1) return 0;                        /* |v| < 0.5 → 0 */
        int32_t q = ((u & 0x7FFFFFu) != 0u) ? 1 : 0;  /* >0.5 → 1；恰 0.5 → 取偶 = 0 */
        return (u >> 31) ? -q : q;
    }
    /* |v| < 2^23 ⇒ e ≤ 22，这一支走不到；留着只是为止住 NaN/Inf 造成的移位 UB。 */
    if (e >= 23) return 0;
    uint32_t m = (u & 0x7FFFFFu) | 0x800000u;         /* 隐含前导 1 + 23 位小数 */
    int      sh = 23 - e;
    uint32_t q  = m >> sh;                            /* 截断 */
    uint32_t r  = m & ((1u << sh) - 1u);              /* 被丢掉的小数 */
    uint32_t h  = 1u << (sh - 1);                     /* 半个单位 */
    if (r > h || (r == h && (q & 1u))) q++;           /* 就近取偶 */
    return (u >> 31) ? -(int32_t)q : (int32_t)q;
}
/* ★★ 第 36 轮 · floor 的定点实现（fp → int 向下取整，纯位操作）
   与 rne_f2i 是同一个套路、同一个理由：C3 无 FPU，`floorf` 和 `(int)` 转换
   各是一次软浮点库调用。fsin_t 每次调用都要这两下，台架量到 4449 次/帧。
   这里按 IEEE754 位域手工做，**与 floorf 逐位一致**（|v| < 2^31 时），二十来周期。
   定义域：非 NaN / Inf、|v| < 2^31 —— 本页的相位与世界坐标都远小于它。
   ⚠️ 为什么不做"Q15 相位全整数插值"那一版：试过了，**对拍 19/20 状态不一致**。
      Q15 的圈内小数精度是 1/32768 圈，而原实现的 float 小数精度随 tf 量级
      可精细到 1e-10 圈 —— 差五个数量级。偏差经 koi_step 的积分放大，
      整条动画轨迹都会分岔。**纯提速不许改一个像素，所以那版必须退掉。** */
static inline int32_t floor_f2i(float v)
{
    union { float f; uint32_t u; } b;
    b.f = v;
    uint32_t u = b.u;
    int sign = (int)(u >> 31);
    int e    = (int)((u >> 23) & 0xFFu) - 127;
    if (e < 0) {                                     /* |v| < 1 */
        if (!sign || (u & 0x7FFFFFFFu) == 0u) return 0;   /* [0,1) 与 ±0 → 0 */
        return -1;                                        /* (-1,0) → -1 */
    }
    if (e >= 23) {                                   /* 已经是整数：左移尾数 */
        if (e > 30) return sign ? (-2147483647 - 1) : 2147483647;   /* 饱和，避开移位 UB */
        uint32_t m = (u & 0x7FFFFFu) | 0x800000u;
        int32_t  q = (int32_t)(m << (e - 23));
        return sign ? -q : q;
    }
    uint32_t m = (u & 0x7FFFFFu) | 0x800000u;
    int      sh = 23 - e;
    uint32_t q  = m >> sh;                           /* 截断 = 向零取整 */
    if (sign && (m & ((1u << sh) - 1u)) != 0u) q++;  /* 负且有小数 → 再下一格 = 向下取整 */
    return sign ? -(int32_t)q : (int32_t)q;
}
/* 开局三屏的位图资源（首页整屏设计稿 / 7 枚档位图章 / UI 文字小图）。
   全是 const，进 flash；由 _tools/koi_assets_gen.py 从网页版抽出，**勿手改**。 */
#include "koi_assets.h"

static const char *TAG = "koi";

#define KW        BSP_LCD_W
#define KH        BSP_LCD_H
#define NPX       (KW * KH)
#define FPS       24
#define NSUB      2
#define PXMAX     128
#define NPTS      64

#define KOI_PI    3.14159265358979f
#define MAX_KOI   9
#define MAX_RIP   24
#define MAX_PEL   24
#define MAX_FOOD  24
/* 脏矩形表容量。6 片荷叶每帧各占 1 个框，加上 koi×2 / 涟漪 / 饲料本来就在
   竞争这 18 个位置 —— 表满会退化成整屏（s_full=1），那是 100% 脏区、
   直接击穿 41.7ms 预算。多给 10 个位置只花 160 B DRAM，换"不会被静悄悄顶成全屏"。 */
#define MAX_RECT  28
#define MAX_XS    32

/* ==========================================================================
   1. 三角函数查表
   ========================================================================== */
/* ==========================================================================
   ★★ 第 35 轮 · 性能归因计数器（只在台架编译；固件一行都不编）
   --------------------------------------------------------------------------
   真机读数（v11）：渲染 151.2ms / 预算 41.7ms，剖开是 **鱼 72.9 + 荷 62.9 = 92%**。
   但"一条 33px 的鱼画一次要 11.3ms = 180 万周期 @160MHz"这个数，用常规模型
   **怎么算都算不出来**：鱼身 49 点、荷叶 48 点，fill_poly 的内层迭代也就几千次，
   按 50 周期/次算才 0.1~0.2ms。差一个数量级 ⇒ 一定有个我没数到的大项。

   最大嫌疑：**RISC-V 软浮点的库调用**。C3 无 FPU，每一次 lrintf / sqrtf / floorf
   都是一次 libm 函数调用（压栈 + IEEE754 位操作迭代），单次可能上百上千周期。
   而每条鱼每帧要描 ~49 段边（每段 1 次 sqrtf + 4 次 lrintf）、to_q8 再来 2×49 次
   lrintf —— 约 300 次库调用。

   ⚠️ **x86 台架的时间读数在这里会骗人**：x86 有 FPU，lrintf/sqrtf 就几条指令，
   而整数内循环一样跑满 —— 于是"浮点"的相对成本在台架上被严重低估。
   **只有"调用次数"是跨平台可信的量。** 所以本组只数次数、不测时间；
   归因 = 次数 × C3 单次成本。
   ⚠️ 宏与变量必须放在**本文件最前面**：fsin_t（第 84 行）就要用 PCNT，
      宏必须早于第一次使用；宏又展开出 s_c_* 变量，所以两者同批上提。
   ========================================================================== */
#ifdef KOI_HOST_PROBE
static long long s_c_px, s_c_span, s_c_fp, s_c_fpinc, s_c_fpedge, s_c_fpdrop,
                 s_c_pts, s_c_sl, s_c_sqrt, s_c_lrint, s_c_quad, s_c_trig,
                 s_c_floor, s_c_koi, s_c_lily;
#define PCNT(k)       do { s_c_##k++; } while (0)
#define PCNT_N(k, n)  do { s_c_##k += (n); } while (0)
#else
#define PCNT(k)       do { } while (0)
#define PCNT_N(k, n)  do { } while (0)
#endif

static int16_t s_sin[1024];

static void sin_init(void)
{
    for (int i = 0; i < 1024; i++)
        s_sin[i] = (int16_t)lrintf(sinf(2.0f * KOI_PI * (float)i / 1024.0f) * 16384.0f);
}

/* ★★ 第 36 轮 · fsin_t —— 只把 `floor_f2i(t)` 换成定点 floor_f2i
   --------------------------------------------------------------------------
   原式每次调用约 9 次软浮点库调用：
       a*K(1) + floorf(1) + (int)(1) + (float)i0(1) + 减(1)
       + (float)(表值差)(1) + 乘 fr(1) + 加(1) + 除 16384(1)
   C3 是 RV32IMC（无 F 扩展），一次软浮点 = 一次库函数调用、几十~上百周期；
   而本函数是「影 / 身 / 尾」那二十几毫秒里最大的单项（台架 4449 次/帧）。
   这里只省掉 floorf + (int) 这两次，**其余一个字都不动** —— 原因见下面这段教训。
   ★★ 做废的一版（同轮）：曾把它整个换成「Q15 相位 + 全整数插值」，
      理论上每次只剩两次 float 乘，账面上最漂亮。结果 **对拍 19/20 个状态不一致**，
      而且不只是像素差 —— 脏区从 16.8% 掉到 1.9%、各状态帧数全变，**整条动画
      轨迹都分岔了**。根因：Q15 的圈内小数精度是 1/32768 圈，而原实现的 float
      小数精度随 t 量级可细到 1e-10 圈，**差五个数量级**；这点偏差再被 koi_step
      的积分放大，鱼就游到别处去了。
      ⇒ 结论：**能省的只有"调用次数"，不能省"精度"。** 纯提速不许改一个像素。
   ⚠️ 等价性不靠上面推导，靠 koi_bin_diff.py 逐字节对拍（改前先备份 _host/out 下的帧缓冲）。 */
static inline float fsin_t(float a)
{
    float t = a * (1024.0f / (2.0f * KOI_PI));
    PCNT(trig);                          /* ★ 三角函数查表调用（跨平台可比的量） */
    int   i0 = floor_f2i(t);             /* ★ 第 36 轮：替代 floor_f2i(t) —— 省两次软浮点 */
    PCNT(floor);
    float fr = t - (float)i0;
    i0 &= 1023;
    int i1 = (i0 + 1) & 1023;
    return ((float)s_sin[i0] + (float)(s_sin[i1] - s_sin[i0]) * fr) / 16384.0f;
}
static inline float fcos_t(float a) { return fsin_t(a + KOI_PI * 0.5f); }

/* ★★ 第 36 轮 · 常量多边形单位圆表
   --------------------------------------------------------------------------
   两处"每点现算 sin/cos"的热点，角度集合其实是**编译期常量**：
     · stroke_line 的 10 边形圆接头（`disc[10]`）—— 每点 10 组 cos/sin；
     · 荷叶缺口顶点那个 8 边形小圆 —— 每片叶子 8 组。
   它们与"点在哪、线宽多少"毫无关系，只有 k 在做功。台架量到三角函数
   **5538 次/帧**（改前 7631），这条路占掉其中一大块。
   表在 sin_init 之后用**同一个 fcos_t/fsin_t** 填一次值（就一次，不是每帧），
   所以 `表[k]*hw` 与 `fcos_t(t_k)*hw` 逐位相同 —— 等价性由 koi_bin_diff.py 对拍背书，
   不靠上面这段推导（铁律：优化前先备份帧缓冲，改完必须逐字节对拍）。 */
static float s_disc_u[10][2], s_oct_u[8][2];
static void unitcircle_init(void)
{
    for (int k = 0; k < 10; k++) {
        float t = 2.0f * KOI_PI * (float)k / 10.0f;
        s_disc_u[k][0] = fcos_t(t); s_disc_u[k][1] = fsin_t(t);
    }
    for (int k = 0; k < 8; k++) {
        float t = 2.0f * KOI_PI * (float)k / 8.0f;
        s_oct_u[k][0] = fcos_t(t); s_oct_u[k][1] = fsin_t(t);
    }
}

/* ==========================================================================
   2. 调色板 —— **一套配色 + 整屏压暗**（第 34 轮）
   ★ 王总原话：「昼夜交替我觉不要要换晚上颜色了 直接就是把屏幕暗度调低即可」。
     旧版抄了网页版的两套端点色 DAY_PAL / NIGHT_PAL，按 night 连续插值 ——
     夜里水面变成深蓝色、绿藻变冷青，**等于换了一个池子**，不是"天黑了"。
     现在只留一套 DAY_PAL，夜间就是把它整体乘一个亮度系数：
         night=0 → ×1.00（白天原色）   night=1 → ×NIGHT_DIM
     好处不止是"看着像同一个池子"：`>>8` 的定点乘法在 k=256 时**恒等于原值**，
     所以白天那一档与旧版逐字节相同，历史对照图不作废。
   ★ 稳态零重建：night 没变就立刻返回（第 19 轮的教训，别退回每帧重建）
   ========================================================================== */
/* ★ PI_BAND 已删 —— 王总定"波带不要了"。
   ★ PI_GLOW 第 34 轮删 —— 水面那个"圆圆晕染"就是它（见第 6 节）。 */
enum {
    PI_WTOP, PI_WBOT, PI_SPARK,
    PI_KBODY, PI_KSPOT, PI_KGOLD, PI_KFIN, PI_KEDGE,
    PI_RIPPLE, PI_PELLET,
    PI_LILYFILL, PI_LILYEDGE, PI_LILYVEIN, PI_WEED, PI_WEEDPALE,
    PI_NPAL
};

static const uint8_t DAY_PAL[PI_NPAL][3] = {
    { 44, 126, 112}, { 17,  70,  70}, {220, 254, 240},
    {250, 246, 238}, {232,  90,  38}, {253, 216, 124}, {252, 204, 176},
    {158,  78,  38}, {232, 255, 248}, {246, 208, 138},
    /* 荷叶 / 浮萍（网页版 DAY.lilyFill / lilyEdge / lilyVein / weed / weedPale） */
    { 52, 156,  88}, { 22,  94,  54}, {104, 210, 140}, {116, 192,  96},
    {164, 226, 118},
};

/* 夜间亮度 = 白天的 42%。挑这个数不是拍的：再低鱼红就开始并档（232,90,38 乘到
   0.30 只剩 70,27,11，与体缘暗线 158,78,38 的暗版撞车，红白鲤会糊成一条褐鱼）。 */
#define NIGHT_DIM  0.42f
/* night=1 时要减掉的量（0..255）。拆成常量是为了让 build_palette 里只剩一次整数乘。 */
#define NIGHT_DROP ((int)((1.0f - NIGHT_DIM) * 256.0f + 0.5f))

static uint8_t s_pal[PI_NPAL][3];
static int     s_pal_night = -1;    // 已建立对应的 night×255；-1 = 还没建过

/* ==========================================================================
   2c. ★ 第 37 轮「观感档位」—— 一条参数阶梯，档 0 = 改动前**逐字节等价**的现状
   --------------------------------------------------------------------------
   王总第 37 轮原话（掌上锦鲤）：
     「荷叶 应该像我给到你的图样效果 现在不是 / 水池背景色好像也不对 还有横横横的一段
       我需要干净的湖水背景 / 上面还有星星点点的杂质不知道是怎么回事 /
       鱼的这个两条鱼上下的话好像还要透明度 应该是不对的」
   这四条里，**"横条 / 星点 / 尾鳍透"是可量化的缺陷**，而**"弱化到什么程度"是观感刻度**，
   没有唯一答案（铁律 14）。所以做成一条阶梯 + 运行时档位，**档 0 保持原样**，
   王总点着挑，挑中哪个再把那个档定成默认。

   ★ 每一档的量**都是从王总给的参考图样实测反推的**，不是拍的
     （工具：_tools/koi_lily_ref.py、_tools/bg4.py；读数在 _tools/_lily_ref_colors.log）：
       参考图样  叶身 (53,155,89) · 叶脉亮部 (65,167,101) · 叶缘暗部 (28,103,66)
                 水面 上(47,123,110) → 下(20,74,74)
                 波光点只比水色亮 **+20 / +28 / +23**
       固件现状  叶身 (48,156,88) —— **与参考几乎一样，叶身色不用动，别乱改**
                 叶脉实测只到 (48,148,88)：不是颜色错，是**笔画宽 0.5px 只覆盖一半**
                 波光点偏 **+86 / +71 / +69** —— **亮了 3 倍**，这就是"星星点点的杂质"
                 水面跨度 ΔG=56 摊到 320 行：565 的 G 只有 64 级 → 49 级台阶、
                 **每 6.5 行跳一级** → 肉眼就是"横横横的一段"
   ========================================================================== */
#define LOOK_LILY_DEF   0
#define LOOK_BGD_DEF    3    /* ★ 第 40 轮：水面改「纯色」，去掉 ΔG=56 那 22 条横条带（王总原话"跟足球场一样"） */
#define LOOK_SPARK_DEF  2    /* ★ 第 41 轮：关掉波光点（王总原话"星星点点的跟星星一样的是啥东西啊 去掉他"；第 37 轮默认开是为了"水面提亮版"观感，现在宁可干净） */
#define LOOK_TAIL_DEF   0
#define LOOK_NIGHT_DEF  1    /* ★ 昼夜量化是纯性能修正（王总要的"不卡"），默认打开 */
/* ★ 昼夜过渡分成几阶（0 = 不量化 = 逐帧推进 = 改动前的现状）。
   为什么要有这一列档：**这是成本与平滑度的直接兑换**，而"该多滑"是观感刻度。
   台架实测（24 帧 = 完整 1 秒过渡，逐帧采全屏平均 G）：
     0 阶（逐帧）：23/23 帧都在变（帧帧整屏）· 每步跳幅 0.65 G 级 · 均匀度 1.17×
     8 阶        ： 8/23 帧在变（少 65%）  · 每步跳幅 1.82 G 级 · 均匀度 1.14×
   也就是说：省掉三分之二的整屏，代价是每步跳幅变 2.8 倍。
   **"8 阶够不够滑"没有唯一答案 → 做成档位让用户挑，别替他定。**
   注意档位与阶数的映射把 24 阶排除了 —— 24 阶 ≈ 帧帧整屏，等于把性能修回去。 */
static const int LOOK_NIGHT_STEPS[4] = {0, 8, 12, 6};
#define LOOK_NIGHT_NSTEP 4

static int s_look_lily  = LOOK_LILY_DEF;
static int s_look_bgd   = LOOK_BGD_DEF;
static int s_look_spark = LOOK_SPARK_DEF;
static int s_look_tail  = LOOK_TAIL_DEF;
static int s_look_night = LOOK_NIGHT_DEF;

#ifdef KOI_HOST_PROBE
/* ★ 第 39 轮「叶下鱼台架」专用：这一帧**跳过鱼 0 的绘制**。
   它只掐绘制、不碰模拟（鱼 0 照常走位置、照常参与同类避让、照常报脏），
   所以"画了 / 没画"两组出图的差异**只可能**来自鱼 0 的像素本身
   —— 这正是判断"荷叶盖不盖得住鱼"唯一干净的判据。
   （先试过"把鱼挪到屏外"当对照：鱼 0 一挪走，其他鱼的避让全变了，
     差异包围盒 x[17..141] y[16..177] 远大于叶盘 —— 判据被模拟变化污染。） */
static int s_hide_koi0;
#endif

/* 水面档：只改 WTOP / WBOT 两项。
   ΔG 与"565 上的台阶数"是同一件事：G 只有 64 级，320 行分摊 ΔG →
   台阶间隔 = 320 / ΔG。ΔG=56 → 每 5.7 行一跳（看得见）；ΔG=8 → 每 40 行一跳（看不见）。 */
static const uint8_t BGD_W[4][2][3] = {
    /* 档 0 现状   ΔG=56 → 49 级台阶 / 每 5.7 行一跳 */
    {{ 44, 126, 112}, { 17,  70,  70}},
    /* 档 1 弱渐变 ΔG=16 → 14 级 / 每 20 行；保留一点纵深 */
    {{ 36, 104,  96}, { 24,  88,  82}},
    /* 档 2 极弱   ΔG= 8 →  7 级 / 每 40 行；肉眼基本看不出带 */
    {{ 33, 100,  93}, { 27,  92,  87}},
    /* 档 3 纯色   ΔG= 0 → 零台阶；代价是失去纵深（干净到底） */
    {{ 30,  96,  90}, { 30,  96,  90}},
};

/* DAY_PAL 的可变副本 —— 水面档要改它两项，但 DAY_PAL 是 const 表（别的档都读它） */
static uint8_t s_day[PI_NPAL][3];
static void day_pal_sync(void)
{
    for (int i = 0; i < PI_NPAL; i++)
        for (int c = 0; c < 3; c++) s_day[i][c] = DAY_PAL[i][c];
    int b = s_look_bgd;
    if (b < 0) b = 0;
    if (b > 3) b = 3;
    for (int c = 0; c < 3; c++) {
        s_day[PI_WTOP][c] = BGD_W[b][0][c];
        s_day[PI_WBOT][c] = BGD_W[b][1][c];
    }
    /* ★ 必须让 build_palette 的稳态门失效，否则档位换了调色板还留在旧档 ——
       这正是铁律 13 那一族（"重建了但内容一样"与"根本没重建"画面完全相同，
       必须能读数）。这里用 s_pal_night = -1 当"脏门"，是最省事也最看得见的读法。 */
    s_pal_night = -1;
}

/* ==========================================================================
   3. 帧缓冲与像素混合（RGB565）
   ========================================================================== */
static uint16_t s_fb[NPX];
static uint8_t  s_e5[32], s_e6[64];

static void expand_init(void)
{
    for (int i = 0; i < 32; i++) s_e5[i] = (uint8_t)((i * 255 + 15) / 31);
    for (int i = 0; i < 64; i++) s_e6[i] = (uint8_t)((i * 255 + 31) / 63);
}

static inline uint16_t pack565(int r, int g, int b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/* ==========================================================================
   2b. ★ 临时诊断探针（量完就摘）
   --------------------------------------------------------------------------
   为什么加：真机 log 报 avg=307ms/帧、实际 2fps，预算是 41.7ms。
   光知道"总共很慢"没法修 —— 得先分清是「填充像素慢」还是「绘制原语个数太多」。
   esp_timer_get_time() 每次约 1~3us，每帧十几个点 ≈ 0.05ms，相对 307ms 可忽略。
   放在这里（而不是第 13 节绘制里）是因为 lily_paint（第 7b 节）和 koi_draw
   （第 8 节）比它早 —— 放晚了就是"未声明的标识符"，本机预检直接拦下。

   第 1 组 槽位：0=调色板 1=step 2=render 总 3=水 5=鱼 6=涟漪 7=荷叶浮萍 8=饲料
   （4 是原「波带」，波带删掉后已废）
   第 2 组 槽位（鱼内部）：0=spine+轮廓 1=水下投影 2=鱼身+暗线 3=红斑 4=高光 5=尾鳍 6=胸鳍
   ========================================================================== */
static int64_t s_prof[10];
static int64_t s_pt;
#define PROF_START()  do { s_pt = esp_timer_get_time(); } while (0)
#define PROF_TICK(i)  do { int64_t _n = esp_timer_get_time();   \
                           s_prof[i] += _n - s_pt; s_pt = _n; } while (0)

/* ★ 第 2 组必须用独立时钟 s_pt2 —— 要是共用 s_pt，鱼内部的打点会把时间先分走，
   外层 s_prof[5]（鱼总）就会读到个小数，两组读数互相污染。 */
static int64_t s_prof2[8];
static int64_t s_pt2;
#define PROF2_START() do { s_pt2 = esp_timer_get_time(); } while (0)
#define PROF2_TICK(i) do { int64_t _n = esp_timer_get_time();    \
                           s_prof2[i] += _n - s_pt2; s_pt2 = _n; } while (0)

/* ★ 重复绘制计数：一帧里 koi_draw / lily_paint 到底被调了几次。
   对象个数是死的（5 条鱼、6 片叶），所以"调了 12 次"就直接意味着重复画。
   scene_draw 一帧被调 s_nrect 次（5~14），每次都会把命中的对象重画一遍 ——
   这条计数就是用来量那个倍数的。 */
static int s_koi_n, s_lily_n;

static int s_cx0, s_cy0, s_cx1, s_cy1;      // 当前裁剪框（含端点）

/* ★ 真实 AABB 累加器（第 34 轮，铁律 15）
   王总原话：「有时候后面会有拖影」—— 旧口径是"以 (x,y) 为中心、边长 0.75L·grow+10 的
   方块"，而鱼连尾鳍往后要伸 1.22L、尾梢摆动还有 ±0.11L 的横向行程 → 尾梢整个在框外，
   擦不掉，就成了拖在身后的影子。
   这里挂在 span_draw 上，是**零遗漏**的做法：不管画的是鱼身、红斑、尾鳍，还是描边的
   圆接头，最后都要过这里，而且拿到的是"最终落在哪个像素"的真值，不必逐处补算。
   ⚠️ 必须在 clip 判断**之前**记录 —— 那是"想画到哪"的范围。先裁后记会自我循环：
      clip 来自脏区，而脏区正是要用这份 AABB 去报的。 */
static int s_bb_on;
static int s_bbx0, s_bby0, s_bbx1, s_bby1;

/* ★ 不变量计数器（只给台架编译）：span 覆盖率 cov **必须 ≤ 256**（= 一个整像素）。
   一旦越过，说明"边缘像素覆盖率"的算法又被裁剪污染了 —— 那条路会让
   px_blend 收到 a>256，颜色冲出 [0,255] 后被 pack565 折成乱色。
   ⚠️ 这是"上限校验"型判据（铁律 12）：不看画面就能发现，而且不会像
      "颜色看着不对"那样随观感漂移。固件不带 KOI_HOST_PROBE → 一行都不编。 */
#ifdef KOI_HOST_PROBE
static long s_cov_over;
/* 预测脏盒"不够多少"的最大值（正数 = 那一侧会残留；应 ≤ 0）。
   它就是 KOI_DIRT_MARGIN 的取值依据 —— 换鱼速 / 转角 / 体长参数后重跑，读数会变。 */
static float s_bbox_short = -1e9f;
/* 脏区表追踪窗口（台架排障用）：s_rtrace_from..to 之间把鱼报脏的每一步打出来。
   ★ 为什么需要它：光看"帧末的矩形表"，分不清"没报"和"报了又被并掉/吃掉"。 */
static int s_rtrace_from = 1 << 30, s_rtrace_to = -(1 << 30);
static int s_step_no;
void demo_koi_rect_trace(int from, int to)
{ s_rtrace_from = from; s_rtrace_to = to; }
/* ★ 把"池内步号"给台架看 —— gold_check 报"第 N 帧（全局帧号）"时，
   同时报 s_step_no，两个号一对就能把追踪窗口设准。
   上一版就是没这个口子，凭"帧号≈步号"猜了个 (79,81)，结果窗口落在
   巡游末尾之外 → 一行 trace 都没打出来（假阴性：以为"没有异常"）。 */
int demo_koi_step_no(void) { return s_step_no; }
#endif

/* ==========================================================================
   ★★ 第 35 轮 · 性能归因计数器（只在台架编译；固件一行都不编）
   --------------------------------------------------------------------------
   真机读数（v11）：渲染 151.2ms / 预算 41.7ms，剖开是 **鱼 72.9 + 荷 62.9 = 92%**。
   但"一条 33px 的鱼画一次要 11.3ms = 180 万周期 @160MHz"这个数，用常规模型
   **怎么算都算不出来**：鱼身 49 点、荷叶 48 点，fill_poly 的内层迭代也就几千次，
   按 50 周期/次算才 0.1~0.2ms。差一个数量级 ⇒ 一定有个我没数到的大项。

   最大嫌疑：**RISC-V 软浮点的库调用**。C3 无 FPU，每一次 lrintf / sqrtf / floorf
   都是一次 libm 函数调用（压栈 + IEEE754 位操作迭代），单次可能上百上千周期。
   而每条鱼每帧要描 ~49 段边（每段 1 次 sqrtf + 4 次 lrintf）、to_q8 再来 2×49 次
   lrintf —— 约 300 次库调用。300 × 3000 周期 ≈ 90 万周期，量级正好对上。

   ⚠️ **x86 台架的时间读数在这里会骗人**：x86 有 FPU，lrintf/sqrtf 就几条指令，
   而整数内循环一样跑满 —— 于是"浮点"的相对成本在台架上被严重低估。
   **只有"调用次数"是跨平台可信的量。** 所以本组只数次数、不测时间；
   归因 = 次数 × C3 单次成本（成本模型写在 host_main 的打印里）。
   ========================================================================== */
/* ★ 计数器的变量与宏已上提到第 1 节之前（见那里的注释）。这里只留两个台架口子。 */
#ifdef KOI_HOST_PROBE
/* ★ 37 轮：波光点实际落笔读数（在 water_rect 里累加）。
   为什么要从绘制路径数、不从截图数：截图判据分不开"波光点"与"白鱼抗锯齿边"
   —— 两者都是"高 G + R/G≈0.8 + B/G≥0.8"，实测连"关掉波光"那一档都还剩 59 个"亮点"。
   只有这里拿到的是"px_blend 前后帧缓冲的真实 ΔG"，骗不了人。 */
static long s_spark_px;
static int  s_spark_maxd;
long demo_koi_spark_px(void)   { return s_spark_px; }
int  demo_koi_spark_maxd(void) { return s_spark_maxd; }

/* 清零 —— 采样必须"从稳态起算"：从进程启动累计的话，开局三屏的几千次
   fill_poly 会把池内每帧的均值稀释掉（那个数就没意义了）。 */
void demo_koi_perf_reset(void)
{
    s_c_px = s_c_span = s_c_fp = s_c_fpinc = s_c_fpedge = s_c_fpdrop = 0;
    s_c_pts = s_c_sl = s_c_sqrt = s_c_lrint = s_c_quad = s_c_trig = 0;
    s_c_floor = s_c_koi = s_c_lily = 0;
    s_spark_px = 0; s_spark_maxd = 0;      /* ★ 37 轮：波光点读数也一起从零起算 */
}

void demo_koi_dump_perf(int frames)
{
    if (frames <= 0) frames = 1;
    double f = (double)frames;
    printf("  ── 每帧调用次数（%d 帧均值）──────────────────────────\n", frames);
    printf("     fill_poly 调用 %8.1f  ·  点数合计 %9.1f  ·  入表边数 %9.1f\n",
           (double)s_c_fp / f, (double)s_c_pts / f, (double)s_c_fpedge / f);
    printf("     fill_poly 内层迭代(边×扫描线) %12.0f  ·  MAX_XS 截断丢点 %8.1f\n",
           (double)s_c_fpinc / f, (double)s_c_fpdrop / f);
    printf("     span_draw 调用 %8.1f  ·  实际写入像素(px_blend) %10.0f\n",
           (double)s_c_span / f, (double)s_c_px / f);
    printf("     stroke_line 段 %8.1f  ·  sqrtf 调用 %8.1f\n",
           (double)s_c_sl / f, (double)s_c_sqrt / f);
    printf("     ★软浮点库调用:  lrintf %9.1f  ·  floorf %9.1f  ·  三角函数 %9.1f\n",
           (double)s_c_lrint / f, (double)s_c_floor / f, (double)s_c_trig / f);
    printf("     pt_quad %9.1f  ·  koi_draw %7.1f  ·  lily_paint %7.1f\n",
           (double)s_c_quad / f, (double)s_c_koi / f, (double)s_c_lily / f);
}
#else
#define PCNT(k)       do { } while (0)
#define PCNT_N(k, n)  do { } while (0)
#endif

static inline void bb_begin(void)
{
    s_bbx0 = 1 << 20; s_bby0 = 1 << 20;
    s_bbx1 = -(1 << 20); s_bby1 = -(1 << 20);
}
static inline int bb_empty(void) { return s_bbx1 < s_bbx0; }

/* 把一串世界坐标（float，x/y 交替）也纳入 AABB —— 尾鳍摆动包络要用 */
static inline void bb_pts(const float *xy, int n)
{
    if (!s_bb_on) return;
    for (int i = 0; i < n; i++) {
        int x = floor_f2i(xy[i * 2]), y = floor_f2i(xy[i * 2 + 1]);
        if (x < s_bbx0) s_bbx0 = x;
        if (x > s_bbx1) s_bbx1 = x;
        if (y < s_bby0) s_bby0 = y;
        if (y > s_bby1) s_bby1 = y;
    }
}

static inline void set_clip(int x0, int y0, int x1, int y1)
{
    s_cx0 = x0 < 0 ? 0 : x0;
    s_cy0 = y0 < 0 ? 0 : y0;
    s_cx1 = x1 > KW - 1 ? KW - 1 : x1;
    s_cy1 = y1 > KH - 1 ? KH - 1 : y1;
}

/* ==========================================================================
   ★★ 第 37 轮排障：**像素归属图**（"这个像素最后是谁写的"）
   --------------------------------------------------------------------------
   起因：台架两条硬判据同时红 —— 「增量 ≠ 整屏」218 像素 + 「漏画」165 像素，
   而现有诊断（demo_koi_full_redraw_diff 里的"盖着:"）只能说
   "这个像素**被哪些对象的包围盒盖着**"。包围盒是**猜**：
   框住了答不出"是哪一笔画出去的"，框不着就只剩「（无对象）」三个字 ——
   上一轮就是卡在这一步，只能按"漏点离荷叶框 3~5px"去反推几何，越推越像玄学。

   本探针把问题改成**可读的读数**（铁律 11 的同族做法）：
     · 在**唯一**的写像素出口（px_blend / px_set，外加两处直写 s_fb 的快路径）
       落一个"画笔编号"；
     · 画笔编号由每个绘制点自己设（水 / 波光 / 涟漪 / 饲料 / 鱼i / 荷i / 浮萍i …）。
   于是判据能直接打出一行真值：
     「整屏 pass = 萍17 画的；增量 pass 本帧 = 无人」
   —— 一句话定位，而且**不依赖包围盒算得对不对**（框算错了它照样告诉你谁画的）。

   ★ 为什么必须挂"出口"而不是"入口"：一条鱼的尾鳍要过 fill_poly + 描边圆接头
     好几支笔，挂入口就得逐处补，漏一处就是假阴性。
   ⚠️ 只在台架编译（-DKOI_HOST_PROBE）；固件构建不带这个宏 → 一行都不存在。
   ⚠️ 归属图 = 240×320 = 76800 B × 2 份 = 150 KB —— 只有 x86 台架敢这么花；
      这也是它**不能**照搬到固件的原因（C3 无 PSRAM）。
   ========================================================================== */
#ifdef KOI_HOST_PROBE
/* ★★ 画笔表容量 —— 这个数是**算出来的，不许拍**：
     水 1 + 幕 1 + 光 44 + 涟 8 + 料 16 + 食 8 + 鱼 9 + 荷 6 + 萍 56 = 149。
   ⚠️ 第 37 轮第一版写 40，而**光波光点一项就 44 个** —— 表一满，
      `demo_koi_who` 走兜底分支把**所有后来者都标成同一个 id**，
      于是判据里满屏"整屏画笔=水"，真凶被洗白。
      这是铁律 9 的同一族：**探针自己坏了，它给的读数比没有读数更坏**
      （会把人引向"真的是水"这个错误结论，而不是"不知道"）。
      所以容量按上面那张清单算，并留一倍余量。 */
#define WHO_MAX     192
#define WHO_KIND_N  17
static unsigned char s_who[KW * KH];        /* 本帧归属图 */
static unsigned char s_who_snap[KW * KH];   /* 快照（判据在两次 render 之间存一次） */
static const char   *s_who_lbl[WHO_MAX];
static char          s_who_buf[WHO_MAX][24];
static int           s_who_cur;
static const char *s_who_kindname(int k)
{
    /* ★ 10 起是**荷叶内部的分笔**（影/身/脉/芯/斑/光/缺）——
       有了它，"画到叶半径外 16px"这件事能直接落到具体那一笔上，
       不用再在 lily_paint 里对着七八笔挨个推半径。 */
    static const char *KN[WHO_KIND_N] =
        {"无", "水", "光", "涟", "料", "食", "鱼", "荷", "萍", "幕",
         "荷影", "荷身", "荷脉", "荷芯", "荷斑", "荷光", "荷缺"};
    return (k < 0 || k >= WHO_KIND_N) ? KN[0] : KN[k];
}
/* 设"当前画笔"。同名的只登记一次，返回稳定的编号（编号 0 = 无人画）。 */
static void demo_koi_who(int kind, int idx)
{
    char b[24];
    int k = (kind < 0 || kind >= WHO_KIND_N) ? 0 : kind;
    if (idx >= 0) snprintf(b, sizeof(b), "%s%d", s_who_kindname(k), idx);
    else          snprintf(b, sizeof(b), "%s",   s_who_kindname(k));
    for (int i = 1; i < WHO_MAX; i++) {
        if (s_who_lbl[i]) { if (strcmp(s_who_lbl[i], b) == 0) { s_who_cur = i; return; } }
        else { snprintf(s_who_buf[i], sizeof(s_who_buf[i]), "%s", b);
               s_who_lbl[i] = s_who_buf[i]; s_who_cur = i; return; }
    }
    s_who_cur = 1;              /* 表满：退化成"水"，至少不静默 */
}
static inline void who_put(int x, int y) { s_who[y * KW + x] = (unsigned char)s_who_cur; }
/* 每帧开头清一次 —— 于是"本帧没有任何人写过它"也能读出来（= 0）。 */
void demo_koi_who_reset(void)
{
    for (int i = 0; i < NPX; i++) s_who[i] = 0;
    s_who_cur = 0;
}
void demo_koi_who_snapshot(void)
{
    for (int i = 0; i < NPX; i++) s_who_snap[i] = s_who[i];
}
const char *demo_koi_who_at(int x, int y)
{ int i = s_who[y * KW + x]; return i ? s_who_lbl[i] : "无"; }
const char *demo_koi_who_snap_at(int x, int y)
{ int i = s_who_snap[y * KW + x]; return i ? s_who_lbl[i] : "无"; }
#define WHO_PUT(x, y)  who_put(x, y)
#else
#define demo_koi_who(k, i)  do { } while (0)
#define WHO_PUT(x, y)       do { } while (0)
#endif

/* alpha 0..256；a=256 时精确等于 src */
static inline void px_blend(int x, int y, int r, int g, int b, int a)
{
    if (a <= 0 || x < s_cx0 || x > s_cx1 || y < s_cy0 || y > s_cy1) return;
    PCNT(px);                            /* ★ 第 35 轮：真正写进去的像素数 */
    WHO_PUT(x, y);                       /* ★ 第 37 轮：像素归属探针（仅台架） */
    uint16_t d = s_fb[y * KW + x];
    int dr = s_e5[(d >> 11) & 31];
    int dg = s_e6[(d >> 5) & 63];
    int db = s_e5[d & 31];
    dr += ((r - dr) * a) >> 8;
    dg += ((g - dg) * a) >> 8;
    db += ((b - db) * a) >> 8;
    s_fb[y * KW + x] = pack565(dr, dg, db);
}

static inline void px_set(int x, int y, int r, int g, int b)
{
    if (x < s_cx0 || x > s_cx1 || y < s_cy0 || y > s_cy1) return;
    WHO_PUT(x, y);
    s_fb[y * KW + x] = pack565(r, g, b);
}

/* ==========================================================================
   4. 光栅化：世界空间水平扫描线（Q8 定点）+ 子扫描线 AA
   ========================================================================== */
typedef struct { int32_t x, y; } ipt_t;          // 世界坐标 Q8（像素 × 256）

/* 明度渐变：alpha 是"世界坐标在方向 u 上的投影 s"的分段线性函数。
   对应网页版的 createLinearGradient —— 尾鳍根部盖住接缝那一段全靠它。 */
typedef struct {
    int32_t ux, uy, cx, cy;      // 单位方向 ×256 / 投影原点（均 Q8）
    int32_t s0, s1, s2, s3;      // 断点（Q8 px）
    int     a0, a1, a2, a3;      // 对应 alpha（0..256）
} grad_t;

static inline int grad_alpha(const grad_t *g, int32_t s)
{
    if (s <= g->s0) return g->a0;
    if (s >= g->s3) return g->a3;
    if (s < g->s1)  return g->a0 + (int)(((int32_t)(g->a1 - g->a0) * (s - g->s0)) / (g->s1 - g->s0));
    if (s < g->s2)  return g->a1 + (int)(((int32_t)(g->a2 - g->a1) * (s - g->s1)) / (g->s2 - g->s1));
    return g->a2 + (int)(((int32_t)(g->a3 - g->a2) * (s - g->s2)) / (g->s3 - g->s2));
}

static KOI_HOT void span_draw(int32_t xa, int32_t xb, int32_t ly,
                              const uint8_t *col, const grad_t *gr, int a_sub)
{
    if (xb <= xa || a_sub <= 0) return;
    PCNT(span);
    int y = ly >> 8;
    if (s_bb_on) {                       /* ★ 真实 AABB：clip 之前记录"想画到哪" */
        int bx0 = xa >> 8, bx1 = (xb - 1) >> 8;
        if (bx0 < s_bbx0) s_bbx0 = bx0;
        if (bx1 > s_bbx1) s_bbx1 = bx1;
        if (y   < s_bby0) s_bby0 = y;
        if (y   > s_bby1) s_bby1 = y;
    }
    if (y < s_cy0 || y > s_cy1) return;
    /* ★★ 第 34 轮修正：**coverage 必须与裁剪无关**。
       原来先把 pxa/pxb 夹进裁剪框，再拿"夹过的端点"去判"是不是边缘像素"：
           int pxa = xa>>8; if (pxa < s_cx0) pxa = s_cx0;      … pxb 同理
           if (pxa == pxb) cov = xb - xa;                      ← 单像素特例
           else if (x == pxa) cov = ((x+1)<<8) - xa;
       三步都建立在"pxa 就是这条 span 的真实首像素"这个前提上，而夹过之后它不再是。
       后果：一条横跨 20px 的 span 落进 2px 宽的脏矩形里，cov 会算出 1700 多
       （真值 ≤256），a = (a_sub*cov)>>8 随之爆表；px_blend 的
       `dr += ((r-dr)*a)>>8` 于是冲出 [0,255]，而 pack565 是位运算**不夹值** ——
       负值被 &0xF8 折回高位，直接吐出饱和乱色（实测出过 255,4,74 / 32,16,98）。
       更要命的是"同一个像素画出来是什么颜色，取决于这帧它是被哪个矩形画的"：
       脏矩形一变，叶子/鱼身上就浮出一条会闪的杂色带 —— 王总那句
       「经过荷叶下方会有不知道什么多余的杂颜色出来」，一半是它。
       正解：**用未裁剪的真实端点 (e0,e1) 判分支，裁剪只用来决定"写哪几个像素"**。
       于是 clip 只影响"画不画"，不影响"画成什么样" —— 与 Canvas 的语义一致，
       也让"擦掉重画"与"整屏重画"逐字节等价（台架可以直接断言）。 */
    int e0 = xa >> 8;                     /* 真实首像素（未裁剪） */
    int e1 = (xb - 1) >> 8;               /* 真实末像素（未裁剪） */
    int pxa = e0 < s_cx0 ? s_cx0 : e0;    /* 只裁剪"写到哪" */
    int pxb = e1 > s_cx1 ? s_cx1 : e1;
    if (pxa > pxb) return;
    int r = col[0], g = col[1], b = col[2];
    for (int x = pxa; x <= pxb; x++) {
        int cov;
        if (e0 == e1)      cov = (int)(xb - xa);
        else if (x == e0)  cov = (int)(((int32_t)(x + 1) << 8) - xa);
        else if (x == e1)  cov = (int)(xb - ((int32_t)x << 8));
        else               cov = 256;
#ifdef KOI_HOST_PROBE
        if (cov > 256) s_cov_over++;
#endif
        if (cov <= 0) continue;
        int a;
        if (!gr) {
            a = (a_sub * cov) >> 8;
        } else {
            int32_t px = ((int32_t)x << 8) + 128;
            int32_t s = ((px - gr->cx) * gr->ux + (ly - gr->cy) * gr->uy) >> 8;
            a = (grad_alpha(gr, s) * cov) >> 8;
            a = (a * a_sub) >> 8;
        }
        px_blend(x, y, r, g, b, a);
    }
}

/* 世界空间多边形填充。a_flat = 总 alpha（0..256），内部切成 NSUB 份做超采样。 */
static int32_t s_ex[PXMAX], s_esl[PXMAX], s_ey0[PXMAX], s_ey1[PXMAX];
static int32_t s_xs[MAX_XS];

static KOI_HOT void fill_poly(const ipt_t *p, int n, const uint8_t *col,
                              const grad_t *gr, int a_flat)
{
    if (n < 3 || a_flat <= 0) return;
    PCNT(fp);                            /* ★ 调用次数 */
    PCNT_N(pts, n);                      /* ★ 多边形点数合计（≈ 入表边数） */
    /* ★★★ 第 39 轮修正：**a_flat = 256 必须真的是 256**，不能再切成 NSUB 份各自混合。
       原式是 `a_sub = a_flat / NSUB`，而 span_draw 对**每条子扫描线**各 px_blend 一次
       —— 于是内部像素被混合 NSUB 次、每次 a_flat/NSUB：
           实际不透明度 = 1 − (1 − 1/NSUB)^NSUB
       NSUB=2 ⇒ **0.75，而不是 1.0**。也就是说：**所有本该完全不透明的填充
       （荷叶叶身、鱼身、浮萍、饲料…）其实只有 75% 不透明，底下的东西透出 25%。**

       王总这一轮的**两条诉求是同一个根因**：
         · 「荷叶需要做成不能透视的，不能透看到下面的鱼」
           —— 叶面 75% ⇒ 水色/波光/鱼从叶面下透上来；
         · 「鱼也一样，现在鱼是透视的，需要实体感」
           —— 鱼身 75% ⇒ 整条鱼蒙着一层水色。
       顺带解释了 37 轮那条「荷叶上星星点点的杂质」：波光点比叶面亮得多，
       25% 的透出量足以在叶面上留下可见的亮点。

       ★ 为什么"改成 256、第一条子线就写实"是对的：
         px_blend 在 a=256 时 `dr += ((r-dr)*256)>>8 = r-dr` ⇒ 精确等于 src；
         同一条线上后续子扫描线再写一次是幂等的。所以内部像素 = 纯色（真 100%），
         边缘像素仍按 cov<256 部分覆盖 ⇒ 抗锯齿保留（只是过渡比原来更"实"，
         这正是"实体感"要的方向）。

       ⚠️ 只动**完全不透明**这一档：半透明笔画（叶脉 α153 / 鳍 α179 / 叶斑 α46 …）
         继续走 `a_flat / NSUB`，保持第 37 轮已经调好的那些观感档**一个像素不变**。
       ⚠️ 这条是**渲染路径改动** ⇒ 旧截图全部作废、性能数字要重采（铁律 3）；
          台架两条硬判据（脏区渲染 / 脏区覆盖）必须重跑。 */
    int a_sub = (a_flat >= 256) ? 256 : (a_flat / NSUB);
    if (a_sub <= 0) return;

    int32_t ymin = p[0].y, ymax = p[0].y;
    for (int i = 1; i < n; i++) {
        if (p[i].y < ymin) ymin = p[i].y;
        if (p[i].y > ymax) ymax = p[i].y;
    }
    int iy0f = ymin >> 8;                // 算术右移 = floor（负数也对）
    int iy1f = (ymax - 1) >> 8;
    int iy0 = iy0f, iy1 = iy1f;
    if (iy0 < s_cy0) iy0 = s_cy0;
    if (iy1 > s_cy1) iy1 = s_cy1;
    /* ★★ 第 34 轮：**累加 AABB 时不能按裁剪行范围循环**。
       原来这里 `if (iy0 > iy1) return;` 直接把整段跳掉，于是"真实绘制包围盒"
       只记到了**落在那个脏矩形里的那部分** —— 框反而越记越小，下一帧照它报脏，
       框外的鱼身就永远擦不掉（台架金标准判据抓到过：残留在叶缘上，报的框里没有）。
       现在：累加 AABB 时走**未裁剪**的完整行范围（span_draw 会先把"想画到哪"记进
       AABB，再按 clip 决定写不写像素 —— 顺序是关键，见它开头的注释）；
       不累加 AABB 时（普通绘制）保持原来的快路径，一行都不多算。 */
    if (s_bb_on) { iy0 = iy0f; iy1 = iy1f; }
    if (iy0 > iy1) return;

    /* 每条边预计算：起点 x + 斜率（dx / 每 Q8 行）。
       ★★ 第 35 轮：**把 int64 换成 int32** —— 这里是本页最热的两处运算
          （真机台架量到：入表边 8752 次/帧 · 交点 127647 次/帧）。
          C3 是 RV32IMC：int32 的乘/除各是一条 MUL / DIV 指令（DIV 十几周期），
          而 int64 会落到 `__muldi3` / `__divdi3` 的**软件例程**（上百周期）。
          这笔钱本来就不必花 —— 这里的取值范围天生装得下 int32：
            · 分子 (bx−ax)*256：两点同在一屏内，|bx−ax| ≤ (KW+KH)*256 = 143360，
              积 ≤ 36,700,160 < 2^31 ✓
            · 除数 (by−ay) ≠ 0（水平边上面已被 continue 掉）
            · 斜率 s_esl = 分子/除数，|…| ≤ 36,700,160 < 2^31 ✓
            · 交点增量 (ly−ey0)*s_esl：ly ∈ [ey0, ey1) ⇒ 0 ≤ ly−ey0 < den，
              故积 < |dx|*256 ≤ 36,700,160 < 2^31 ✓
          三条同时成立 ⇒ **结果与 int64 版逐位相同**（不溢出就没有截断差异）。
          ⚠️ 必须写 `* 256` 而不是 `<< 8`：bx−ax 可能为负，负值左移是 UB
             （主机台架带 UB 检查，会当场抛 "left shift of negative value"）。
          ⚠️ 台架同时开了 signed-overflow 检查：上面那个上界一旦估错，
             这里会当场报出来，不会静默算错。 */
    int ne = 0;
    for (int i = 0; i < n; i++) {
        int32_t ax = p[i].x, ay = p[i].y;
        int32_t bx = p[(i + 1) % n].x, by = p[(i + 1) % n].y;
        if (ay == by) continue;
        /* ★★ 第 37 轮修正：**基准 x 必须取"较上那个端点"的 x**。
           插值式是 `x = s_ex + (ly - s_ey0) * slope`，而下面 s_ey0 = min(ay,by)。
           原代码写死 `s_ex = ax`（p[i] 的 x），于是**当边朝上（by < ay）时**
           s_ey0 指的是 p[i+1] 的 y、s_ex 却是 p[i] 的 x —— 整条边被**横移了 (ax-bx)**。
           影响面：
             · 圆弧相邻点：横移量 ≈ r·Δa ≈ 2px → 被 1px 描边盖住，肉眼看不出来，
               所以这个错存在了很久都没被发现；
             · **荷叶缺口那条边**：从叶缘(≈±r)连到叶心/缺口顶点，横移量 ≈ 半径
               → 缺口侧直接鼓出十几像素的**多填**，而且位置正好在缺口方向。
           第 37 轮王总说"荷叶不像图样"，那个鼓包就是它；台架"增量≠整屏"也是它
           （鼓包越出上报脏框 → 增量的边缘像素永远擦不掉）。
           ⚠️ 修完之后**画面会变**（是变对），所以：旧的截图一律作废、
              性能数字要重采 —— 这是"渲染路径改动"，按铁律 3 走完整体检。 */
        s_ex[ne]  = (ay < by) ? ax : bx;
        s_esl[ne] = ((bx - ax) * 256) / (by - ay);
        s_ey0[ne] = ay < by ? ay : by;
        s_ey1[ne] = ay < by ? by : ay;
        ne++;
        PCNT(fpedge);                    /* ★ 实际入表的边数（水平边已被 continue 掉） */
    }
    if (ne < 2) return;

    /* ★★ 第 35 轮：**活动边表（AET）**。
       原内层是 `行 × NSUB × 全部边` —— 每条扫描线把 ne 条边从头扫一遍
       （台架量到 **127647 次/帧**），而每行真正活跃的通常只有 2~6 条。
       这里按 ey0 排序后用滑动窗口维护活跃集：
         · ly 在两层循环上都是**单调递增**的（iy 递增、s 递增）⇒ 加入与移除
           都只往前走，均摊 O(1)；
         · 活跃集只存下标（uint8_t），每行做一次"剔除 ey1 <= ly"的压实。
       ★ 等价性论证：排序只改变"谁提供交点"，**交点集合与原来完全相同**；
         而台架上另一个计数 s_c_fpdrop（MAX_XS 截断丢点）恒为 **0** ——
         从未发生过"交点太多被丢掉"，所以顺序不影响结果 ⇒ 逐字节等价。
         反过来：哪天 fpdrop != 0 了，这条等价性就不成立，必须回来复查。 */
    for (int a = 1; a < ne; a++) {       /* 按 ey0 插入排序（沿轮廓生成，近乎有序） */
        int32_t k0 = s_ey0[a], k1 = s_ey1[a], kx = s_ex[a], ks = s_esl[a];
        int b = a - 1;
        while (b >= 0 && s_ey0[b] > k0) {
            s_ey0[b + 1] = s_ey0[b]; s_ey1[b + 1] = s_ey1[b];
            s_ex[b + 1]  = s_ex[b];  s_esl[b + 1] = s_esl[b];
            b--;
        }
        s_ey0[b + 1] = k0; s_ey1[b + 1] = k1; s_ex[b + 1] = kx; s_esl[b + 1] = ks;
    }
    static uint8_t s_act[PXMAX];         /* 活跃边下标（ne ≤ PXMAX，uint8_t 装得下） */
    int na = 0, next_new = 0;

    for (int iy = iy0; iy <= iy1; iy++) {
        for (int s = 0; s < NSUB; s++) {
            int32_t ly = ((int32_t)iy << 8) + (int32_t)(((2 * s + 1) * 128) / NSUB);
            while (next_new < ne && s_ey0[next_new] <= ly)   /* 新进入的边 */
                s_act[na++] = (uint8_t)next_new++;
            int w = 0;                                       /* 已越过 ey1 的边出表 */
            for (int t = 0; t < na; t++) {
                uint8_t e = s_act[t];
                if (s_ey1[e] > ly) s_act[w++] = e;
            }
            na = w;
            int nx = 0;
            for (int t = 0; t < na; t++) {
                uint8_t e = s_act[t];
                PCNT(fpinc);             /* ★ 内层"边 × 扫描线"迭代总数 —— 归因的主角 */
                int32_t x = s_ex[e] + (((ly - s_ey0[e]) * s_esl[e]) >> 8);
                if (nx < MAX_XS) s_xs[nx++] = x;
                else             PCNT(fpdrop);   /* ★ 被 MAX_XS 截断丢掉的交点（AET 等价性的凭据） */
            }
            if (nx < 2) continue;
            for (int a = 1; a < nx; a++) {          // 交点极少，插入排序
                int32_t v = s_xs[a];
                int b = a - 1;
                while (b >= 0 && s_xs[b] > v) { s_xs[b + 1] = s_xs[b]; b--; }
                s_xs[b + 1] = v;
            }
            for (int k = 0; k + 1 < nx; k += 2)
                span_draw(s_xs[k], s_xs[k + 1], ly, col, gr, a_sub);
        }
    }
}

/* 折线描边：每段摊一个四边形；够粗时补圆接头，否则拐角缺口肉眼可见 */
static ipt_t s_quad[4];

static KOI_HOT void stroke_line(float x0, float y0, float x1, float y1,
                                float halfw, const uint8_t *col, int a_flat)
{
    float dx = x1 - x0, dy = y1 - y0;
    PCNT(sl);                            /* ★ 描边段数（每段都要一次 sqrtf + 四次 lrintf） */
    PCNT(sqrt);
    float d = sqrtf(dx * dx + dy * dy);
    if (d < 1e-4f) return;
    float nx = -dy / d * halfw, ny = dx / d * halfw;
    PCNT_N(lrint, 4);
    s_quad[0].x = (int32_t)rne_f2i((x0 + nx) * 256.0f); s_quad[0].y = (int32_t)rne_f2i((y0 + ny) * 256.0f);
    s_quad[1].x = (int32_t)rne_f2i((x1 + nx) * 256.0f); s_quad[1].y = (int32_t)rne_f2i((y1 + ny) * 256.0f);
    s_quad[2].x = (int32_t)rne_f2i((x1 - nx) * 256.0f); s_quad[2].y = (int32_t)rne_f2i((y1 - ny) * 256.0f);
    s_quad[3].x = (int32_t)rne_f2i((x0 - nx) * 256.0f); s_quad[3].y = (int32_t)rne_f2i((y0 - ny) * 256.0f);
    fill_poly(s_quad, 4, col, NULL, a_flat);
}

static KOI_HOT void stroke_pts(const float *xy, int n, int closed, float width,
                               const uint8_t *col, int a_flat)
{
    float hw = width * 0.5f;
    if (hw < 0.35f) hw = 0.35f;
    for (int i = 0; i + 1 < n; i++)
        stroke_line(xy[i * 2], xy[i * 2 + 1], xy[i * 2 + 2], xy[i * 2 + 3], hw, col, a_flat);
    if (closed && n > 2)
        stroke_line(xy[(n - 1) * 2], xy[(n - 1) * 2 + 1], xy[0], xy[1], hw, col, a_flat);
    if (hw >= 1.2f) {
        ipt_t disc[10];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < 10; k++) {   /* ★ 第 36 轮：改用常量单位圆表 s_disc_u */
                disc[k].x = (int32_t)rne_f2i((xy[i * 2]     + s_disc_u[k][0] * hw) * 256.0f);
                disc[k].y = (int32_t)rne_f2i((xy[i * 2 + 1] + s_disc_u[k][1] * hw) * 256.0f);
            }
            fill_poly(disc, 10, col, NULL, a_flat);
        }
    }
}

/* --------------------------------------------------------------------------
   4b. 通用点列工具 + 小工具（第 7b 节的荷叶也要用，故从第 8 节上提到这里）
   -------------------------------------------------------------------------- */
static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

/* 角度差，归到 (-π, π] */
static inline float angdiff(float a, float b)
{
    float d = a - b;
    while (d >  KOI_PI) d -= 2.0f * KOI_PI;
    while (d < -KOI_PI) d += 2.0f * KOI_PI;
    return d;
}

/* 通用 float 点列 → 世界 Q8 多边形（静态，避免大数组上 LVGL 任务栈） */
static float s_pts[NPTS * 2];
static int   s_npts;

static inline void pt_push(float x, float y)
{
    if (s_npts < NPTS) { s_pts[s_npts * 2] = x; s_pts[s_npts * 2 + 1] = y; s_npts++; }
}

/* ★★ 第 36 轮 · 二次贝塞尔采样：把 t 展开成常量系数
   --------------------------------------------------------------------------
   原式每个采样点算 `mt*mt*x0 + 2*mt*t*cx + t*t*x1`（mt = 1−t）：
   t 是变量 ⇒ 7 次乘（2+3+2）。而 t 只取 **0.25 / 0.5 / 0.75 / 1.0** 四个值，
   系数全是**精确的二次幂分数**：3/4·3/4 = 9/16、2·3/4·1/4 = 3/8、1/4·1/4 = 1/16，
   尾数装得下 ⇒ 在 float32 里**逐位精确**，不是近似。
   本页每帧约 30 次 pt_quad（body 12 / 尾 4 / 胸鳍 8 / …），每次 4 点 × 省 4 乘。
   ⚠️ 等价性：系数值相同 + **加法顺序不变**（都是 `(k0*x0 + k1*cx) + k2*x1`
      对原来的 `((mt*mt*x0) + (2*mt*t*cx)) + (t*t*x1)`）⇒ 逐位相同。
      依旧由 koi_bin_diff.py 对拍背书。 */
static const float s_pq[4][3] = {      /* {x0 系数, cx 系数, x1 系数} */
    {0.5625f, 0.375f, 0.0625f},        /* t = 0.25 */
    {0.25f,   0.5f,   0.25f  },        /* t = 0.50 */
    {0.0625f, 0.375f, 0.5625f},        /* t = 0.75 */
    {0.0f,    0.0f,   1.0f   },        /* t = 1.00（退化成端点 x1） */
};

static void pt_quad(float x0, float y0, float cx, float cy, float x1, float y1)
{
    for (int j = 0; j < 4; j++) {
        const float *k = s_pq[j];
        pt_push(k[0] * x0 + k[1] * cx + k[2] * x1,
                k[0] * y0 + k[1] * cy + k[2] * y1);
    }
}

static ipt_t s_poly[NPTS];

static void to_q8(int n, float dx, float dy)
{
    for (int i = 0; i < n; i++) {
        s_poly[i].x = (int32_t)rne_f2i((s_pts[i * 2] + dx) * 256.0f);
        s_poly[i].y = (int32_t)rne_f2i((s_pts[i * 2 + 1] + dy) * 256.0f);
    }
}

/* ==========================================================================
   5. 场景状态（先声明：第 6 节的水面要用到波光点、时间与脏区表）
   ========================================================================== */
/* ★ 第 34 轮：中心光晕（GLOW_CX/CY/R0/R1 + s_dx2 + s_glow_lvl + s_water_lut）整段删除。
   王总原话：「现在醒目里水面颜色不纯净并且有个圆圆圆圆晕染 需要修正」。
   它原来做的事：以 (120, 121.6) 为心、R0=8 内全亮、R1=178 处归零，把水色往 PI_GLOW
   拉一个圆 —— 在 240×320 的池子里那就是**一个能看出来的大圆盘**，不是"水面的光"。
   代价还不止观感：water_rect 每个像素都要算 dx²+dy² → 查 s_glow_lvl → 再查
   s_water_lut[y][lv]，76800 像素全屏就是 76800 次平方距离 + 两次查表。
   删掉后水面只剩**纵向渐变**，每行一个颜色 → 内层退化成"整行写同一个 uint16"，
   这同时是"脏区一大就掉帧"的主因之一。 */

typedef struct { float x, y, phase; int size; } spark_t;

static spark_t s_spark[44];

static float   s_time, s_night, s_nightTarget;
/* ★ 37 轮：昼夜过渡的**逻辑值** —— 每帧连续走 dt（1 秒走完），
   而 s_night 是**量化后真正生效**的那个（档位 >0 时按 `LOOK_NIGHT_STEPS` 的阶数跳）。
   分成两个变量的理由见 step() 里那段注释：卡顿的根因是"每帧都整屏重画"。 */
static float   s_night_log;

/* 脏区表定义在第 7 节，第 6 节的水面要用 —— 先把原型摆出来。
   （漏了它就是 ISO C99 的隐式声明；在 -Werror 下直接编不过）
   注：波带删掉后，"波带条"那第二条通路（band_dirty_add / s_brc）也一并拆了。 */
static void dirty_add_ext(int x0, int y0, int x1, int y1);

/* ---- 开局三屏的状态 -------------------------------------------------------
   放在这里（而不是第 14 节）是因为第 7 节的 pond_init 要用 s_split_kh 配鱼色：
   鱼的红白/黄金是在**建池子那一刻**定下来的，不是进池子之后才染。 */
/* 条数档位用吉祥成语命名。避开 4 和 7 —— 国人忌讳，不是漏写。
   数组下标 == NM_ASSET 的下标（DZ/JY/SL/WF/LL/BH/JL 同序）。 */
#define N_PICK  7
static const uint8_t PICK_N[N_PICK] = {1, 2, 3, 5, 6, 8, 9};

static int s_scene;              /* 0 = 开局三屏 · 1 = 池子 */
static int s_setup_step;         /* 0 首页 / 1 选条数 / 2 分色 */
static int s_pick_idx;           /* PICK_N 的下标 */
static int s_pick_n   = 1;       /* 条数 = PICK_N[s_pick_idx] */
static int s_split_kh = 1;       /* 红白条数；御黄金 = s_pick_n - s_split_kh */
static int s_setup_dirty = 1;    /* 换屏请求：下一帧整屏重画一次 */

/* 网页版 defaultSplit：偶数对半，奇数红白多一条（红白是主色，也最好认） */
static int default_split(int n) { return (n == 2) ? 1 : (n + 1) / 2; }

/* 网页版 colorSeq(n, nk)：先全铺 gold，再把 nk 条红白**均匀交错**放进去。
   不这么做的话"选 5 条红白 2"会变成前两条红、后三条金 —— 池子左半边一片红、
   右半边一片金，读起来像两队鱼而不是一群。写进 out 的是 pat：0 = kohaku / 1 = gold。

   ⚠️ 这个函数会影响**全局随机数序列的长度**：make_spots 里 gold 一脚不抽、
   kohaku 要抽 11~16 脚，所以换个配色会让后面荷叶 / 波光的位置整体平移。
   网页版 initPond(colors) 本来就是这个行为，不是 bug；
   反过来讲，改配色时不能指望"只有鱼的颜色变了"。 */
static void pick_colors(uint8_t *out, int n, int nk)
{
    for (int i = 0; i < n; i++) out[i] = 1;
    if (nk <= 0) return;
    if (nk > n) nk = n;
    for (int i = 0; i < nk; i++) {
        int idx = (int)(((float)i + 0.5f) * (float)n / (float)nk);
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        out[idx] = 0;
    }
}


/* ==========================================================================
   6. 水面
   ========================================================================== */
/* ★ 第 34 轮：不再有 s_water_lut / s_glow_lvl / s_dx2 —— 水面就是**每行一个颜色**。
   320 个 uint16 = 640 B，比原来 320×32 的 LUT（20 KB！）省得离谱，而且查表变直写。 */
static uint16_t s_water_row[KH];

static void water_row_build(void)
{
    const uint8_t *wt = s_pal[PI_WTOP], *wb = s_pal[PI_WBOT];
    for (int y = 0; y < KH; y++) {
        float u = (float)y / (float)(KH - 1);
        s_water_row[y] = pack565((int)(wt[0] + (wb[0] - wt[0]) * u),
                                 (int)(wt[1] + (wb[1] - wt[1]) * u),
                                 (int)(wt[2] + (wb[2] - wt[2]) * u));
    }
}

/* ★ 尾鳍"淡色"表（尾鳍档 1 用）—— 定义在 TJ_GA/TJ_GG 之后（第 8 节），
   但调色板每次重建都要跟着重建，所以这里先声明。 */
static void build_tail_pale(void);

/* ★ 返回"这一档是不是真的重建了"。调用方必须拿它去置 s_full ——
   见 tick_cb 里的注释（palette 换档与整屏重画必须同帧，第 34 轮修正③）。 */
static int build_palette(float night)
{
    int n8 = (int)rne_f2i(night * 255.0f);
    if (n8 == s_pal_night) return 0;          // ★ 稳态复用，零重建
    s_pal_night = n8;
    /* 整屏压暗：k = 256（白天）→ 256-NIGHT_DROP（夜里）。
       ★ 必须写成 `(v * k) >> 8` 而**不是** `v * NIGHT_DIM`：
         k == 256 时右移 8 位恒等于原值（IEEE754 下 0 误差），
         白天那一档与旧版逐字节相同 → 历史对照图不作废。
         ⚠️ 这条是第 34 轮特意保的：改配色公式最容易顺手把"白天"也改动 1 个色阶，
            然后就再也对不上之前所有截图了。 */
    int k = 256 - (NIGHT_DROP * n8) / 255;
    for (int i = 0; i < PI_NPAL; i++)
        for (int c = 0; c < 3; c++)
            s_pal[i][c] = (uint8_t)(((int)s_day[i][c] * k) >> 8);
    water_row_build();
    build_tail_pale();                        /* 尾鳍淡色跟着调色板走 */
    return 1;
}

/* 水面（含常驻波光点）填一个矩形 */
static KOI_HOT void water_rect(int x0, int y0, int x1, int y1)
{
    demo_koi_who(1, -1);                          /* ★ 归属探针：这一片是"水" */
    for (int y = y0; y <= y1; y++) {
        uint16_t *o = &s_fb[y * KW];
        uint16_t  c = s_water_row[y];
        for (int x = x0; x <= x1; x++) {
            o[x] = c;                             /* 整行同色，无查表无混合 */
#ifdef KOI_HOST_PROBE
            WHO_PUT(x, y);                        /* 直写 s_fb 的快路径，探针要单独补 */
#endif
        }
    }
    /* ★ 第 37 轮「波光点档」—— 王总：「上面还有星星点点的杂质不知道是怎么回事」。
       量出来的真值（参考图样 vs 固件，同一把尺子量"彩色偏移量"）：
         参考图样 波光点只比水色亮 **+20 / +28 / +23**（是"水色提亮版"，不是白点）
         固件现状 alpha 118/256 = 0.46，PI_SPARK 又是近纯白 (220,254,240)
                  → 实际偏 **+86 / +71 / +69**，**整整亮了 3 倍** → 刺眼的白点。
       所以不是"有没有"的问题，是"太亮"的问题。档 1 取参考等效（46），档 2 直接关掉。 */
    /* 档 1 的 46 → 40 是**量出来再回推**的：台架探针读到 alpha 46 时单点提亮 ΔG=32，
       而王总参考图样是 +28 → 46 × 28/32 = 40.2 → 取 40。 */
    static const int SPARK_A[3] = {118, 40, 0};
    int sa = SPARK_A[(s_look_spark < 0) ? 0 : (s_look_spark > 2 ? 2 : s_look_spark)];
    const uint8_t *sp = s_pal[PI_SPARK];
    if (sa > 0)
    for (int i = 0; i < 44; i++) {            // 波光点：44 个，按矩形过滤，代价可忽略
        int px = (int)s_spark[i].x, py = (int)s_spark[i].y;
        demo_koi_who(2, i);
        for (int dy = 0; dy < s_spark[i].size; dy++)
            for (int dx = 0; dx < s_spark[i].size; dx++) {
                int x = px + dx, y = py + dy;
                if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
#ifdef KOI_HOST_PROBE
                    /* ★ 37 轮：直接从绘制路径数"提亮了多少" ——
                       截图猜的失败史：判据窗口要么漏掉波光、要么把**白鱼的抗锯齿边**
                       （白与水的混合色，G 高、R/G≈0.8、B/G≥0.8）也数进来，
                       三档量出 186/200/143，甚至"关掉波光"那档还剩 59 个。
                       只有在这里取前后帧缓冲的真实差值才不会骗人（铁律 11）。 */
                    uint16_t before = s_fb[y * KW + x];
                    px_blend(x, y, sp[0], sp[1], sp[2], sa);
                    int dg = (int)s_e6[(s_fb[y * KW + x] >> 5) & 63]
                           - (int)s_e6[(before >> 5) & 63];
                    s_spark_px++;
                    if (dg > s_spark_maxd) s_spark_maxd = dg;
#else
                    px_blend(x, y, sp[0], sp[1], sp[2], sa);
#endif
                }
            }
    }
}

/* ★ 水面波带（bands_draw / bands_mark_dirty）已整段删除 —— 王总定"波带不要了"。
   真机读数（C3 实测）：波带条循环 75ms + scene_draw 里的波带 37ms = 112ms/帧，
   占 307ms 的 37%，是第二大开销；更狠的是它每帧报一个**整屏宽**横条，
   脏区因此被顶到 67.7%（它一碰谁，谁就变整屏宽）。
   水面现在只剩「纵向渐变 + 44 个波光点」，全在 water_rect 里，一次画完。
   ★ 注意：波带的 5×7=35 次 rnd_f 仍留在 pond_init 里照抽 —— 见那里的说明。 */

/* ==========================================================================
   7. 脏区矩形表
   ========================================================================== */
static int s_nrect;
static struct { int x0, y0, x1, y1; } s_rc[MAX_RECT];
static int s_full;

/* 合并值不值？ —— 判据一：**并集面积相对"两块各自面积之和"的膨胀率**。
   合并本身是必要的：LVGL 每帧的失效区个数有上限（LV_INV_BUF_SIZE），
   而且小矩形（饲料 8x8、涟漪）本来就该并起来。
   但"碰一下就并"会把散落的鱼 / 荷叶一路粘成一个包围盒 ——
   实测夜态 4 个框粘成 1 个 202x257（= 全屏 68% 脏区），而真正变化的像素只有 6%。
   所以只在「几乎不膨胀」或「本来就大面积重叠」时才并。 */
static int merge_worth(int i, int x0, int y0, int x1, int y1)
{
    int ux0 = s_rc[i].x0 < x0 ? s_rc[i].x0 : x0;
    int uy0 = s_rc[i].y0 < y0 ? s_rc[i].y0 : y0;
    int ux1 = s_rc[i].x1 > x1 ? s_rc[i].x1 : x1;
    int uy1 = s_rc[i].y1 > y1 ? s_rc[i].y1 : y1;
    long sa = (long)(s_rc[i].x1 - s_rc[i].x0 + 1) * (s_rc[i].y1 - s_rc[i].y0 + 1);
    long sb = (long)(x1 - x0 + 1) * (y1 - y0 + 1);
    long su = (long)(ux1 - ux0 + 1) * (uy1 - uy0 + 1);
    /* ★ 第 34 轮：膨胀阈值 15% → 45%。
       原来卡在 15% 是为了"脏区别变大"；但那笔账的前提是"水面填充很贵"——
       而水面第 34 轮已经退化成"每行写同一个 uint16"（原来每像素要算 dx²+dy²
       再查两次表）。天平因此翻了过来：
         多并一点 → 脏区面积 ↑（多填的水几乎免费）→ 矩形个数 ↓ →
         鱼/荷叶被重复重画的次数 ↓（那才是真机 216ms/frame 的主项）。
       数值会写进文档：脏区占比与"鱼画/荷画"两个读数都要一起重新采。 */
    if (su * 100 <= (sa + sb) * 145) return 1;          /* 膨胀 ≤ 45% → 值得并 */

    int ox0 = s_rc[i].x0 > x0 ? s_rc[i].x0 : x0;
    int oy0 = s_rc[i].y0 > y0 ? s_rc[i].y0 : y0;
    int ox1 = s_rc[i].x1 < x1 ? s_rc[i].x1 : x1;
    int oy1 = s_rc[i].y1 < y1 ? s_rc[i].y1 : y1;
    if (ox1 >= ox0 && oy1 >= oy0) {
        long sm = sa < sb ? sa : sb;
        long ov = (long)(ox1 - ox0 + 1) * (oy1 - oy0 + 1);
        if (ov * 100 >= sm * 40) return 1;              /* 重叠 ≥ 40% → 值得并 */
    }
    return 0;
}

static void dirty_add_ext(int x0, int y0, int x1, int y1)
{
    if (s_full) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > KW - 1) x1 = KW - 1;
    if (y1 > KH - 1) y1 = KH - 1;
    if (x0 > x1 || y0 > y1) return;
    /* ★★ 第 34 轮揪出的真凶：**级联合并的下标失效**。
       旧写法是 `s_rc[k] = s_rc[--s_nrect];` —— 把**末元素填到被并掉的那个洞 k**。
       危险在于：如果 `i` 恰好是最后一个（i == 旧 s_nrect-1），那么这一搬
       就是把"并集"本身从 i 搬到了 k，而 s_nrect 已经缩到 i → **并集掉出表外**；
       循环却还在往 `s_rc[i]`（越界幽灵槽）上继续做并集 →
       表内那份是旧的、真正的并集写在幽灵槽里 → **脏区凭空消失，像素永远擦不掉**。
       台架实测（脏区表追踪）：一帧里连并三次，最终 `表内 0 个`，
       三个鱼框全丢；那几行像素就是王总说的「拖影 / 多余的杂颜色」。
       正解：搬完必须**跟着修正 i**（`if (i == s_nrect) i = hit;`），
       并且循环条件别再写死 4 次 —— 一直并到没有值得并的为止（每次至少少一个框，必然收敛）。 */
    for (int i = 0; i < s_nrect; i++) {
        if (!merge_worth(i, x0, y0, x1, y1)) continue;
        if (x0 < s_rc[i].x0) s_rc[i].x0 = x0;
        if (y0 < s_rc[i].y0) s_rc[i].y0 = y0;
        if (x1 > s_rc[i].x1) s_rc[i].x1 = x1;
        if (y1 > s_rc[i].y1) s_rc[i].y1 = y1;
        for (;;) {                          // 并进来之后可能与别的框也值得并 → 反复
            int hit = -1;
            for (int k = 0; k < s_nrect; k++) {
                if (k == i) continue;
                if (!merge_worth(i, s_rc[k].x0, s_rc[k].y0, s_rc[k].x1, s_rc[k].y1)) continue;
                hit = k;
                break;
            }
            if (hit < 0) break;
            if (s_rc[hit].x0 < s_rc[i].x0) s_rc[i].x0 = s_rc[hit].x0;
            if (s_rc[hit].y0 < s_rc[i].y0) s_rc[i].y0 = s_rc[hit].y0;
            if (s_rc[hit].x1 > s_rc[i].x1) s_rc[i].x1 = s_rc[hit].x1;
            if (s_rc[hit].y1 > s_rc[i].y1) s_rc[i].y1 = s_rc[hit].y1;
            s_nrect--;
            if (hit != s_nrect) s_rc[hit] = s_rc[s_nrect];   // 末元素填洞（hit 就是末元素则不必搬）
            if (i == s_nrect) i = hit;                       // ★ 我（并集）刚被搬到 hit 去了
        }
        return;
    }
    if (s_nrect >= MAX_RECT) { s_full = 1; return; }   // 表满 → 退化成整屏，宁可慢也别漏画
    s_rc[s_nrect].x0 = x0; s_rc[s_nrect].y0 = y0;
    s_rc[s_nrect].x1 = x1; s_rc[s_nrect].y1 = y1;
    s_nrect++;
}

/* ---- （已拆）波带条单独表 s_brc / band_dirty_add ----
   它存在的唯一理由是"波带是整屏宽横条，不能跟对象矩形合并"（详见上一版注释：
   它一碰谁就变整屏宽，实测把脏区从 8.7% 虚报成 95%）。
   波带整个删掉后这条通路就没有使用者了，一并拆除 —— 少一张表、少一条分支，
   也少一个"忘了同步改"的地方（铁律第 9 条那类坑）。 */

/* ==========================================================================
   7b. 荷叶与浮萍
   --------------------------------------------------------------------------
   逐式对移植自网页版的 paintLily / paintWeedPad / buildWeeds / drawLily。
     · 荷叶：环形散布最多 6 片（最小间距 r+6 检查），缓慢自转 + 上下呼吸。
     · 浮萍：**荷叶的伴生**（第 22 轮口径）—— 长在叶缘外一圈，跟荷叶一起生成、
       一起绘制、落进荷叶本来报的那个脏框，不额外报脏区。绝不烘进共用底图，
       否则"选数字 / 选颜色"那两屏也长浮萍（王总原话"在选数字和颜色时候就出来了"）。
     · 浮萍档位：WEED_LV 作下标进 WEED_PER[] ——
       0 = 关 / 1 / 2 / **3 = 中档（王总第 25 轮定档）** / 4

   ★ 这里**不做精灵位图缓存**（网页版有，是为了省浏览器里重复的矢量绘制）。
     固件是逐帧矢量画：6 片荷叶约 100 个小多边形 —— 主机台架实测成本在预算内，
     而改成位图要多占 20KB 以上的 DRAM 存精灵表（C3 无 PSRAM，不值当）。
   ★ 浮萍的随机数用**独立种子**的 LCG，绝不能借全局 rnd()：
     全局序列是给鱼 / 荷叶 / 波光用的，借一脚会让它们整体平移。
   ========================================================================== */
#define MAX_LILY   6
#define MAX_WEED   56
#define LILY_ARC_N 46
/* 脏框外扩量。网页版写 WEED_PAD = 9，但它 markDirty 用 floor/ceil 外扩，
   实际生效约 10；而最坏情形（叶缘 4.6 + 簇内 2.2 + 浮萍含根影半径 1.64r=3.6）
   ≈ 10.4，本来就压线。固件直接取 11 —— 每边多 1px 的代价约 0.1% 脏区，换"绝不漏"。 */
#define WEED_PAD   11.0f
#define WEED_LV    3        /* ★ 定档「中档」 */

/* ★ 荷叶画法的粗细档（28 轮）。王总原话：
   「荷叶你要不要看你画的啥样  荷叶就按咱们之前那样就是一个平面画而已 视觉过了就可以啦」

   为什么必须改：上一版的移植是**逐式照抄网页版** paintLily —— 根影 / 叶身 / 叶脉 11 条 /
   叶芯 / 老叶斑 2 块 / 叶缘反光 / 缺口补点，一个不少。问题不在"多画了东西"，
   而在**固件画不出网页版那个效果**：
     网页版那道叶脉是 `lineWidth 0.7 + globalAlpha 0.60`，Canvas 有抗锯齿 →
     柔细的浅色纹理；固件的 stroke_line 最小 1px、没有抗锯齿、alpha 只能靠
     像素混合近似 → 同一行参数到这里变成**一条条实心暗绿硬线**。
     6 片叶子 × 11 条 = 66 条黑线压在深绿底上，叶子看着像"裂了"。
     老叶斑（0.18）与叶缘反光（0.13）同理：网页版几乎看不见，固件是几块硬边脏色。

   LILY_LV = 0  旧精细版（逐式照抄网页版）：叶脉 11 条从圆心起 + 老叶斑 2 块 + 叶缘反光
   LILY_LV = 1  ★平面版（默认）：根影 + 叶身 + 1px 描边 + 缺口补点 + 叶芯 + 叶脉 9 条
   ★ 平面档比精细档省 —— 少了老叶斑 2 块与叶缘反光那道弧（那批是真机 108ms 的大头）。

   ── 29 轮修正（王总原话："**没有三角开口 位置不对 然后荷叶上没有线条**"）──

   ① "没有三角开口 / 位置不对" —— 是**缺口形状错了**，不是没画。
      旧 lily_body_pts 写的是 `圆心 → 绕一圈 → 回圆心`，等于从**圆心**切掉一个扇形；
      圆心处宽度恒为 0 → 视觉上是"从叶缘一路裂到叶心的一条缝"，
      再被 1px 描边从两侧各糊掉半个像素，就是他说的"没有开口"。
      现在改成真正的 V 形切口：顶点收在 r*LILY_GAP_IN 处（**不到圆心**），
      叶缘开口宽 2r·sin(gap) —— 读起来才是"叶子边上开了个三角口"。

   ② "荷叶上没有线条" —— 28 轮我把叶脉整个删了（只留根影/叶身/描边/叶芯）。
      这一句就是要线。加回来，但**不能照抄网页版**：
        网页版 11 条全是 `moveTo(0,0)`，canvas 有抗锯齿 → 中心那片只是稍亮；
        固件是 1px 硬线，11 条挤在 r<5px 处叠成一坨亮斑 —— 那正是"蒲公英"的来源。
      改法：9 条、**从 0.30r 起笔**（中心留给叶芯）、画到 0.92r；
            alpha 压到 0.41（网页版 0.60，固件硬边会显得更亮）。
   ────────────────────────────────────────────────────────────────── */
#define LILY_LV      1
/* ★★ 第 37 轮：整段照 HTML 原版（= 王总给的图样）还原。
   29 轮我"以为"缺口被 1px 描边吃掉了，于是做了两件事 —— **两件都做反了**：
     ① 把缺口顶点从**圆心**挪到 0.34r（"改成真正的 V 形切口"）；
     ② 把缺口角度从 rnd(0.13,0.25) 提到 rnd(0.26,0.36)，翻了一倍。
   王总第 37 轮给的参考图（就是本项目的 HTML 预览页截图）摆出来一看：
     HTML 的 paintLily 是 `moveTo(0,0) → arc → closePath`，**顶点就在圆心**，
     缺口是一条**从叶心笔直射到叶缘的细缝**（两侧几乎平行），不是"一角被咬掉"。
   真机那个 V 形大开口，正是上面两处改动叠出来的 —— 所以他看到的是"荷叶不是那个样"。
   ⚠️ 还原缺口**不动随机数**：gap 只改**取值范围**、rnd_f 的调用次数一次不差。 */
#define LILY_GAP_IN  0.00f   /* 缺口内顶点半径 / 叶半径（0 = 切到圆心，1 = 切到叶缘） */
#define LILY_VEIN_N  11      /* 叶脉条数（照 HTML 原版 paintLily 的 `for(i<11)`） */
/* 叶脉 alpha —— 这里要**补偿"固件没有抗锯齿"**，不是照抄 HTML 的 0.60。
   HTML：lineWidth 0.7px + globalAlpha 0.60 → 0.7px 的线分摊在相邻 2 个像素上，
         每像素覆盖约 0.35 → **单像素有效 alpha ≈ 0.21**。
   固件：stroke_line 最小 1px，整条线**全落在 1 个像素**上（硬边、无抗锯齿）。
   照抄 0.60 会让叶脉比 HTML 亮近 3 倍（这正是真机"叶脉像伞骨"的来源之一）；
   取 0.25（≈64）才与 HTML 的**单像素观感**对齐。 */
#define LILY_VEIN_A  64

static const int WEED_PER[5] = {0, 1, 2, 3, 4};

typedef struct { float x, y, r, t; } weed_t;

/* ★ 荷叶动效的"量化门"（27 轮，真机性能）。
   背景：真机实测荷叶+浮萍 108ms/帧，占删掉波带后那 195ms 的 55%，是最大的一块。
   但荷叶每帧只动 0.02px（bob 幅度 ±0.9px、spin ≤0.07rad/s）——
   **为了肉眼看不见的位移，每帧重画 6 片叶子（每片含叶身 + 46 段描边 + 11 条叶脉
   + 叶芯 + 老叶斑 + 叶缘弧 + 伴生浮萍）**，而且 scene_draw 每帧被调 s_nrect 次，
   一片叶子一帧还不止画一遍。
   做法：把 bob 量化到整像素、rot 量化到 0.05rad，并且**只在量化档位跨档时才标脏**。
   于是重画频率从 24Hz 降到约 1~2Hz，画面差别 <1 像素（叶脉端点位移 r*0.05<1.1px）。
   量化档值存在 lily_t 里，首帧给一个大得不可能相等的初值，保证第一次一定标脏。 */
#define LILY_BOB_Q   1.0f     /* 呼吸量化到整像素 */
#define LILY_ROT_Q   0.05f    /* 自转量化到 0.05 rad ≈ 2.9° */
#define LILY_SLOP    2.0f     /* 脏框外扩：覆盖旧档位（bob 最多差 2px、rot 位移 <1.1px） */

typedef struct {
    float x, y, r, rot, spin, bob, gap;
    float qbob, qrot;             /* 上一帧用过的量化档（用于"跨档才重画"判定） */
    int   ws0, wsn;               /* 伴生浮萍在 s_weed 里的区间 [ws0, ws0+wsn) */
} lily_t;

static lily_t s_lily[MAX_LILY];
static int    s_nlily;
static weed_t s_weed[MAX_WEED];
static int    s_nweed;

/* 当前荷叶的 局部→世界 变换（旋转 + 平移），供 lily_xf / pt_xf_f / weed_paint 共用 */
static float s_lrx, s_lry, s_lca, s_lsa;

static inline void lily_xf(int n)            /* s_pts(局部 float) → s_poly(世界 Q8) */
{
    for (int i = 0; i < n; i++) {
        float px = s_pts[i * 2], py = s_pts[i * 2 + 1];
        s_poly[i].x = (int32_t)rne_f2i((s_lrx + px * s_lca - py * s_lsa) * 256.0f);
        s_poly[i].y = (int32_t)rne_f2i((s_lry + px * s_lsa + py * s_lca) * 256.0f);
    }
}

static float s_wf[NPTS * 2];
static void pt_xf_f(int n)                   /* s_pts(局部) → s_wf(世界 float)，给 stroke_pts 用 */
{
    for (int i = 0; i < n; i++) {
        float px = s_pts[i * 2], py = s_pts[i * 2 + 1];
        s_wf[i * 2]     = s_lrx + px * s_lca - py * s_lsa;
        s_wf[i * 2 + 1] = s_lry + px * s_lsa + py * s_lca;
    }
}

/* 荷叶局部坐标里画椭圆（tilt 是这个椭圆自身的倾角，与荷叶自转无关） */
static void lily_ellipse(float cx, float cy, float rx, float ry, float tilt, int n)
{
    s_npts = 0;
    float ct = fcos_t(tilt), st = fsin_t(tilt);
    for (int i = 0; i < n; i++) {
        float t = 2.0f * KOI_PI * (float)i / (float)n;
        float ex = fcos_t(t) * rx, ey = fsin_t(t) * ry;
        pt_push(cx + ex * ct - ey * st, cy + ex * st + ey * ct);
    }
}

/* 叶身：外弧一整圈 + 一个"从叶缘切进来、不到圆心就收住"的 V 形口子。
   缺口朝局部 +y（与网页版 arc 的起止角一致）。
   ★ 29 轮：顶点收在 r*LILY_GAP_IN，**不再从圆心起笔** —— 理由见上面 LILY_LV 那段。
   点数 = LILY_ARC_N+1（弧）+ 1（内顶点）；fill_poly 走奇偶规则，凹多边形没问题。 */
static void lily_body_pts(float r, float gap)
{
    s_npts = 0;
    float a1 = KOI_PI * 0.5f + gap;                        /* 缺口一侧嘴角 */
    float a0 = KOI_PI * 0.5f - gap;                        /* 另一侧嘴角 */
    float sweep = a0 + 2.0f * KOI_PI - a1;                 /* = 2π − 2·gap */
    for (int i = 0; i <= LILY_ARC_N; i++) {
        float a = a1 + sweep * (float)i / (float)LILY_ARC_N;
        pt_push(fcos_t(a) * r, fsin_t(a) * r);
    }
    pt_push(0.0f, r * LILY_GAP_IN);                        /* 缺口内顶点（闭环自动收回起点） */
}

/* 局部圆弧点列（叶缘内侧反光用）。
   ★ 37 轮：从"只有 LILY_LV=0 才编"改成**两档都编** —— 反光已加回平面版
      （见 lily_paint 末尾），再包在 #if 里会变成 -Wunused-function。 */
static void lily_arc_pts(float r, float a_from, float a_to, int n)
{
    s_npts = 0;
    for (int i = 0; i <= n; i++) {
        float a = a_from + (a_to - a_from) * (float)i / (float)n;
        pt_push(fcos_t(a) * r, fsin_t(a) * r);
    }
}

/* 荷叶这一帧的上下呼吸量（浮萍跟着一起呼吸，但不跟着自转）。
   ★ 已量化到整像素 —— 绘制与标脏共用这一个值，所以"画在哪"和"报哪块脏"永远一致。
     量化前后的差别 <1px（原始幅度只有 ±0.9px），但重画频率从 24Hz 降到 ~1Hz。
   ★ 29 轮**上提到这里**：lily_paint 现在自己也要用 bob ——
     网页版 drawLily 里 `var y = L.y + sin(...)*0.9` 是**叶身**跟着 bob 走的，
     之前固件只让浮萍走（叶身取的是 L->y），两边对不上。 */
static inline float lily_bob(const lily_t *L)
{
    float v = fsin_t(s_time * 0.55f + L->bob) * 0.9f;
    return floorf(v * (1.0f / LILY_BOB_Q) + 0.5f) * LILY_BOB_Q;
}

/* 荷叶这一帧的自转角（量化版，同理）。 */
static inline float lily_rot(const lily_t *L)
{
    float v = L->rot + s_time * L->spin;
    return floorf(v * (1.0f / LILY_ROT_Q) + 0.5f) * LILY_ROT_Q;
}

/* ★★ 第 37 轮「荷叶档」—— 一条阶梯，只动三个量，档 0 = 现状（逐字节等价）。
   ① 老叶斑 alpha   固件有 2 块 0.18α 的暗斑糊在叶面上，而**参考图样没有大面积暗斑**
                    → 这是叶面"发闷"的主因（实测叶身被拉低一档）
   ② 叶脉笔画宽     0.5px 只覆盖半个像素 → 叶脉实际只到 (48,148,88)；
                    参考叶脉是 (65,167,101)。改成 1.0px 覆盖率满，就正好落在参考值上。
                    ⚠️ 不是"把叶脉调亮" —— 颜色 (104,210,140) 配 α0.25 兑出来的
                       正是 (62,169,101)，**与参考一致，不用改色**。改的是覆盖率。
   ③ 叶缘反光弧     现状只绕 2.30~4.10 rad = 103°，参考图样的亮边**几乎绕整圈**
                    （只在缺口处断开） */
static const int   LOOK_LILY_SPOT_A[4] = {46, 0, 0, 0};
static const float LOOK_LILY_VEIN_W[4] = {0.5f, 0.5f, 1.0f, 1.0f};
/* ⚠️ 点数写成**显式表**，不用 `(int)(14*(a1-a0)/1.8)` 现算：
   档 0 的 a1-a0 = 4.10f-2.30f 在 float 下是 1.79999995，乘 14 除 1.8 = 13.999999 →
   取整成 13 —— **少一个点，弧末端差 1px**。第一版就是这么把 130 个像素改掉了
   （对拍才发现的：池内每个状态都差 ~130px，开局屏逐字节相同）。
   铁律 7 的同一族：跟几何耦合的量别留"能算错"的表达式。 */
static const int   LOOK_LILY_RIM_N[4]  = {14, 14, 14, 45};
static const float LOOK_LILY_RIM0[4]   = {2.30f, 2.30f, 2.30f, 1.85f};
static const float LOOK_LILY_RIM1[4]   = {4.10f, 4.10f, 4.10f, 7.58f};

static void lily_paint(const lily_t *L, float th)
{
    static const uint8_t c_shadow[3] = {1, 9, 6};       /* 网页版 '#010906'（根影，两档都画） */
    const uint8_t *fill = s_pal[PI_LILYFILL];
    const uint8_t *edge = s_pal[PI_LILYEDGE];
    float r = L->r;
    const uint8_t *vein = s_pal[PI_LILYVEIN];   /* ★ 29 轮起两档都用（平面档也画叶脉了） */
    /* 网页版 '#dff8e8'（固定色，不进调色板）—— ★ 37 轮起平面版也要用（叶缘反光加回来了） */
    static const uint8_t c_rim[3]    = {223, 248, 232};
    s_lily_n++;                                  /* 本帧第几次画叶子（重复绘制诊断用） */
    /* ★ 归属探针用：本片叶子的下标（由调用方 &s_lily[i] 推出来，不用另开一个全局） */
    const int LI = (int)(L - s_lily);
    /* ★ 37 轮「荷叶档」：本帧生效的档（一处算好，三个绘制点共用 —— 铁律 6：
       跟体量/观感耦合的量只留**一个**入口，别在三个地方各推一次） */
    const int lv = (s_look_lily < 0) ? 0 : (s_look_lily > 3 ? 3 : s_look_lily);

    /* ★ 29 轮：s_lry 补上 lily_bob —— 与网页版 drawLily 的 `y = L.y + sin(...)*0.9` 对齐。
       27 轮查出过一处不一致："标脏按 L->y+bob 报、画的是 L->y"，等于每帧都在为一份
       不存在的位移报脏。现在把两边统一到**同一个量化后的 bob**：画在哪 = 报哪块脏，
       改完不会白多报一个像素。
       ⚠️ 取的是 lily_bob(L)（量化版）而不是原始值 —— 量化档不变时叶身纹丝不动，
          所以量化门那套"跨档才重画"依然成立。 */
    s_lrx = L->x; s_lry = L->y + lily_bob(L);
    s_lca = fcos_t(th); s_lsa = fsin_t(th);

    /* ① 水下根影（偏右下 1.5 / 2.5） */
    demo_koi_who(10, LI);
    lily_ellipse(1.5f, 2.5f, r * 0.98f, r * 0.98f, 0.0f, 16);
    lily_xf(s_npts);
    fill_poly(s_poly, s_npts, c_shadow, NULL, 56);              /* 0.22 */

    /* ② 叶身 + 1px 深色描边 */
    demo_koi_who(11, LI);
    lily_body_pts(r, L->gap);
    pt_xf_f(s_npts);
    lily_xf(s_npts);
#ifdef KOI_HOST_PROBE
    /* ★★ 叶身双向自检（第 37 轮排障用）—— 三样一起量，一次定位：
         ① **顶点**：每个顶点到叶心的距离（应 ≤ r；证明"几何算对没"）；
         ② **填充实际落点**：fill_poly 真正写到的像素包围盒；
         ③ **描边实际落点**：stroke_pts 真正写到的像素包围盒。
       为什么要"实际落点"：顶点对、填充器照样可能填到多边形外面去（光栅化 bug）。
       只量①会得出"几何没问题"然后卡住 —— 第 37 轮就是这么卡了一轮。
       ⚠️ 用现成的真实 AABB 累加器（s_bb_on）来量**不会改画面**：
          它把扫描行范围放开成未裁剪，但 span_draw 仍然按 clip 决定写不写像素
          —— 这正是鱼脏盒那套机制，同一份代码已经跑了几十轮。
       括号里的预期盒 = 旋转后圆的包围盒 ±(r·(|cos|+|sin|)) + 描边余量。 */
    {
        static int shown;
        float md = -1.0f;
        for (int i = 0; i < s_npts; i++) {
            float dx = s_wf[i * 2] - s_lrx, dy = s_wf[i * 2 + 1] - s_lry;
            float d = sqrtf(dx * dx + dy * dy);
            if (d > md) md = d;
        }
        float exp_xy = r * (fabsf(s_lca) + fabsf(s_lsa)) + 2.0f;
        int ex0 = (int)(s_lrx - exp_xy), ex1 = (int)(s_lrx + exp_xy) + 1;
        int ey0 = (int)(s_lry - exp_xy), ey1 = (int)(s_lry + exp_xy) + 1;

        bb_begin(); s_bb_on = 1;
        fill_poly(s_poly, s_npts, fill, NULL, 256);
        s_bb_on = 0;
        int fx0 = s_bbx0, fx1 = s_bbx1, fy0 = s_bby0, fy1 = s_bby1;

        bb_begin(); s_bb_on = 1;
        stroke_pts(s_wf, s_npts, 1, 1.0f, edge, 256);
        s_bb_on = 0;
        int sx0 = s_bbx0, sx1 = s_bbx1, sy0 = s_bby0, sy1 = s_bby1;

        if (shown < 10 &&
            (md > r * 1.10f || fx0 < ex0 || fx1 > ex1 || fy0 < ey0 || fy1 > ey1)) {
            shown++;
            printf("  !! 叶身自检 荷%d: 最远顶点 %.1fpx(%.2fr)  "
                   "填充盒 x[%d..%d] y[%d..%d]  描边盒 x[%d..%d] y[%d..%d]  "
                   "预期盒 x[%d..%d] y[%d..%d]  心=(%.1f,%.1f) r=%.1f rot=%s\n",
                   LI, md, md / r,
                   fx0, fx1, fy0, fy1, sx0, sx1, sy0, sy1,
                   ex0, ex1, ey0, ey1, s_lrx, s_lry, r,
                   (s_lca > 0.9f ? "0" : (s_lca < -0.9f ? "pi" : "mid")));
        }
    }
#else
    fill_poly(s_poly, s_npts, fill, NULL, 256);
    stroke_pts(s_wf, s_npts, 1, 1.0f, edge, 256);
#endif

    /* ②b 缺口内顶点补一个小圆点：stroke_pts 在半个线宽 < 1.2px 时不补圆接头，
       V 形顶点那里会露出一条不足 1px 的缝（网页版靠 canvas 的 miter 接头自动填掉）。
       ★ 29 轮：这个点必须**跟着缺口走** —— 缺口从"切到圆心"改成"收在 0.34r"之后，
         还补在原点就等于补到叶心去了（白补一个点，而且被叶芯盖住看不见）。 */
    {
        demo_koi_who(16, LI);
        ipt_t d[8];
        float tvx = 0.0f, tvy = r * LILY_GAP_IN;
        float wx = s_lrx + tvx * s_lca - tvy * s_lsa;
        float wy = s_lry + tvx * s_lsa + tvy * s_lca;
        for (int k = 0; k < 8; k++) {        /* ★ 第 36 轮：改用常量单位圆表 s_oct_u */
            d[k].x = (int32_t)rne_f2i((wx + s_oct_u[k][0] * 0.5f) * 256.0f);
            d[k].y = (int32_t)rne_f2i((wy + s_oct_u[k][1] * 0.5f) * 256.0f);
        }
        fill_poly(d, 8, edge, NULL, 256);
    }

    /* ③ 叶脉（★ 29 轮起**两档都画**）—— 王总原话："荷叶上没有线条"。
       跳过缺口那一扇区；两个端点都取在叶身内（≤0.92r），不必裁剪。
       精细档 = 照抄网页版（11 条、从圆心起、0.60）；平面档 = 固件适配版，
       区别只有两处（起笔半径 + alpha），理由见上面 LILY_LV 那段注释。 */
#if LILY_LV == 0
    const int   vn = 11;
    const float v0 = 0.00f, v1 = 0.94f;
    const int   va = 153;
#else
    const int   vn = LILY_VEIN_N;
    /* ★ 37 轮：起笔从 0.30r 收到 0.14r（≈叶芯半径 0.13r）。
       HTML 原版是 `moveTo(0,0)` 从圆心起笔，靠**后画的叶芯**（半径 0.13r、alpha 0.42）
       把汇聚点盖住，视觉上就是"叶脉从叶芯射出去"。
       固件照抄 v0 = 0 会踩 29 轮那个坑：11 条 1px 硬线在圆心**叠在同一个像素**上，
       叠加后 1−(1−0.25)^11 ≈ 0.95 → 中心一个白亮斑（就是当时说的"蒲公英"）。
       起笔挪到叶芯边缘**观感完全一致、且没有叠加过曝** —— 这是"等效实现"不是"改设计"。
       终点 0.94r 照 HTML 原版（原为 0.92r）。 */
    const float v0 = 0.14f, v1 = 0.94f;
    const int   va = LILY_VEIN_A;
#endif
    demo_koi_who(12, LI);
    for (int i = 0; i < vn; i++) {
        float a = (float)i / (float)vn * 6.2832f + 0.22f;
        if (fabsf(angdiff(a, KOI_PI * 0.5f)) < L->gap + 0.10f) continue;
        float ca = fcos_t(a), sa = fsin_t(a);
        float x0 = ca * r * v0, y0 = sa * r * v0;
        float x1 = ca * r * v1, y1 = sa * r * v1;
        stroke_line(s_lrx + x0 * s_lca - y0 * s_lsa,
                    s_lry + x0 * s_lsa + y0 * s_lca,
                    s_lrx + x1 * s_lca - y1 * s_lsa,
                    s_lry + x1 * s_lsa + y1 * s_lca,
                    LOOK_LILY_VEIN_W[lv], vein, va);
    }

    /* 叶芯（两档都画：它就是叶脐，一个小圆，最便宜也最像叶子） */
    {
        demo_koi_who(13, LI);
        float cr = r * 0.13f;
        if (cr < 1.4f) cr = 1.4f;
        lily_ellipse(0.0f, 0.0f, cr, cr, 0.0f, 12);
        lily_xf(s_npts);
        fill_poly(s_poly, s_npts, edge, NULL, 108);                 /* 0.42 */
    }

    /* ★★ 37 轮：老叶斑 2 块 + 叶缘内侧反光**从精细档放开到两档都画**。
       28 轮删它们的理由是"真机 108ms 的大头"，但那是**波带还在、荷叶每帧重画**的年代；
       27 轮加了"跨档才重画"的量化门之后，荷叶重画频率已降到 1~2Hz，
       多这两笔的代价落在"偶尔重画一帧"上，肉眼与预算都吃得下。
       顺序照 HTML 原版 paintLily：叶脉 → 叶芯 → **老叶斑 → 反光**。
       ⚠️ HTML 里老叶斑在 `clip()` 内，固件没有 clip —— 但两块的几何都在叶内
          （最远 0.87r / 0.88r < r），且方向相对缺口是**局部坐标固定**的
          （局部 -0.69 / 2.46 rad，离缺口 π/2 至少 2.5 rad），不会随自转跑到缺口上。 */

    /* 老叶斑 2 块（叶面深浅不均）—— ★ 37 轮：受「荷叶档」控制，档 ≥1 整体不画 */
    demo_koi_who(14, LI);
    if (LOOK_LILY_SPOT_A[lv] > 0) {
        lily_ellipse(r * 0.36f, -r * 0.30f, r * 0.40f, r * 0.30f, -0.5f, 12);
        lily_xf(s_npts);
        fill_poly(s_poly, s_npts, edge, NULL, LOOK_LILY_SPOT_A[lv]);      /* 档0 = 46 (0.18) */
        lily_ellipse(-r * 0.42f, r * 0.34f, r * 0.34f, r * 0.26f, 0.6f, 12);
        lily_xf(s_npts);
        fill_poly(s_poly, s_npts, edge, NULL, LOOK_LILY_SPOT_A[lv]);
    }

    /* ④ 叶缘内侧反光（弧 2.30~4.10 rad，恰好避开缺口）—— ★ 37 轮：档 3 绕到近整圈 */
    demo_koi_who(15, LI);
    {
        float a0 = LOOK_LILY_RIM0[lv], a1 = LOOK_LILY_RIM1[lv];
        int rn = LOOK_LILY_RIM_N[lv];                      /* ★ 显式点数，不现算（见上） */
        if (rn > NPTS - 1) rn = NPTS - 1;
        lily_arc_pts(r * 0.84f, a0, a1, rn);
        pt_xf_f(s_npts);
        stroke_pts(s_wf, s_npts, 0, 1.4f, c_rim, 33);                   /* 0.13 */
    }
}

/* 浮萍：三笔 —— 偏右下的水下根影、略压扁的叶身、偏左上的亮芯。
   少了这三笔它就只是"屏上一个小色块"，读不出"浮在水面上"（王总说的"没有透视"）。 */
static void weed_paint(const weed_t *w, float dy)
{
    static const uint8_t c_shadow[3] = {1, 9, 6};
    const uint8_t *body = (w->t < 0.62f) ? s_pal[PI_WEEDPALE] : s_pal[PI_WEED];
    const uint8_t *pale = s_pal[PI_WEEDPALE];
    float r = w->r;

    /* 复用荷叶那套 局部→世界 变换，这里只做平移（浮萍不跟着荷叶自转） */
    s_lrx = w->x; s_lry = w->y + dy; s_lca = 1.0f; s_lsa = 0.0f;

    lily_ellipse(r * 0.36f, r * 0.58f, r * 0.96f, r * 0.74f, 0.0f, 12);
    lily_xf(s_npts);
    fill_poly(s_poly, s_npts, c_shadow, NULL, 66);                  /* 0.26 */
    lily_ellipse(0.0f, 0.0f, r, r * 0.86f, 0.0f, 12);
    lily_xf(s_npts);
    fill_poly(s_poly, s_npts, body, NULL, 246);                     /* 0.96 */
    lily_ellipse(-r * 0.26f, -r * 0.32f, r * 0.36f, r * 0.30f, 0.0f, 10);
    lily_xf(s_npts);
    fill_poly(s_poly, s_npts, pale, NULL, 108);                     /* 0.42 */
}

/* 浮萍生成器的随机数：独立 LCG，等价于 JS 的
   Math.imul(s,1103515245) + 12345 再 & 0x7fffffff */
static inline float weed_wr(uint32_t *s, float a, float b)
{
    *s = (*s * 1103515245u + 12345u) & 0x7fffffffu;
    return a + ((float)(*s) / 2147483647.0f) * (b - a);
}

static void build_weeds(void)
{
    s_nweed = 0;
    for (int i = 0; i < s_nlily; i++) { s_lily[i].ws0 = 0; s_lily[i].wsn = 0; }
    int maxN = WEED_PER[WEED_LV];
    if (maxN <= 0 || s_nlily == 0) return;        /* 0 档 = 完全关，留作回归基线 */

    uint32_t sg = 20260919u;
    for (int li = 0; li < s_nlily; li++) {
        lily_t *L = &s_lily[li];
        /* 每片叶子 0~maxN 撮 —— 有疏有密才自然，别均匀撒盐 */
        int n = floor_f2i(weed_wr(&sg, 0.0f, (float)maxN + 0.999f));
        if (!n) continue;
        int first = s_nweed;
        for (int h = 0; h < n; h++) {
            float a0 = weed_wr(&sg, 0.0f, 6.2832f);
            /* 离叶缘留 1.6~4.6 的水面缝隙：贴着长放大看像"荷叶掉下来的碎片" */
            float d0 = L->r + weed_wr(&sg, 1.6f, 4.6f);
            float lx = fcos_t(a0) * d0, ly = fsin_t(a0) * d0;
            float u = weed_wr(&sg, 0.0f, 1.0f);
            int leafN = (u < 0.45f) ? 1 : (u < 0.80f ? 2 : 3);      /* 独叶居多，小簇点缀 */
            for (int j = 0; j < leafN && s_nweed < MAX_WEED; j++) {
                float wa = weed_wr(&sg, 0.0f, 6.2832f);
                float wd2 = (j == 0) ? 0.0f : weed_wr(&sg, 1.0f, 2.2f);
                weed_t *w = &s_weed[s_nweed++];
                w->x = clampf(L->x + lx + fcos_t(wa) * wd2, 4.0f, (float)KW - 4.0f);
                w->y = clampf(L->y + ly + fsin_t(wa) * wd2, 4.0f, (float)KH - 4.0f);
                w->r = (j == 0) ? weed_wr(&sg, 2.2f, 3.2f) : weed_wr(&sg, 1.5f, 2.2f);
                w->t = weed_wr(&sg, 0.0f, 1.0f);
                /* 网页版这里还算了个 hi（只有大叶才点亮芯），但 paintWeedPad 根本没用它 ——
                   它只影响随机数序号，所以这一脚必须照样抽掉，否则后面每片浮萍全都错位。 */
                if (j == 0) (void)weed_wr(&sg, 0.0f, 1.0f);
            }
        }
        L->ws0 = first; L->wsn = s_nweed - first;
    }
}

/* （lily_bob / lily_rot 已上提到第 7b 节开头 —— lily_paint 现在也要用 bob。） */

/* 落点先躲开荷叶：涟漪与饲料都画在荷叶之下，落在叶面上等于白扔 ——
   玩家看到的是"撒了食水面没反应"，鱼也吃不到。
   （网页版 safeSpot；纯几何，不动随机数序列） */
static float s_spotX, s_spotY;
static void safe_spot(float x, float y)
{
    for (int q = 0; q < s_nlily; q++) {
        const lily_t *L = &s_lily[q];
        float d = hypotf(L->x - x, L->y - y);
        if (d < L->r + 4.0f) {
            float ax = (d < 0.5f) ? 1.0f : (x - L->x) / d;
            float ay = (d < 0.5f) ? 0.0f : (y - L->y) / d;
            x = clampf(L->x + ax * (L->r + 7.0f), 16.0f, (float)KW - 16.0f);
            y = clampf(L->y + ay * (L->r + 7.0f), 20.0f, (float)KH - 28.0f);
        }
    }
    s_spotX = x; s_spotY = y;
}

/* ==========================================================================
   8. 锦鲤
   ========================================================================== */
#define KSEG     5
static const float KBEND[KSEG] = {0.14f, 0.24f, 0.23f, 0.21f, 0.18f};
static const float KAMP[KSEG]  = {0.045f, 0.197f, 0.392f, 0.618f, 0.867f};
#define KPHASE   0.92f
static const float KDEPTH[KSEG + 1] = {0.55f, 0.90f, 0.92f, 0.78f, 0.55f, 0.30f};
                                       /* ★ 第 41 轮：原来 {0.46, 0.88, 1.00, 0.80, 0.56, 0.30}
                                          王总原话"肚子处有点胖 鱼头有点尖"——
                                          头 0.46→0.55（头变宽）、腹 1.00→0.92（腹部不那么大）、
                                          中后段 0.80→0.78 / 0.56→0.55（微微收回）。
                                          比例 belly/head 从 2.17 → 1.67。 */
/* ★ 第 38 轮：王总「把初始鱼的大小做成现在的 3 倍」→ 2.0 → 6.0。
   ★ 第 39 轮：王总「把鱼做成现在的大小的一半」→ 6.0 → 3.0
   （= 原基线 2.0 的 1.5 倍，开局体长 17~23 × 3.0 = 51~69px）。
   倍率**必须同时**作用在体长 / 游速 vT / 吃食 / 同类避让 / 边界硬边距 ——
   统一从 kh = L*grow*0.55 推（第 15 轮定下的规矩），所以这里改一个数就够，
   下面每处 `* KOI_SCALE` 与每个从 kh 推的量都会跟着变。 */
#define KOI_SCALE       3.0f
#define GROW_MAX        1.35f
#define GROW_PER_PELLET 0.018f
/* ★ 报脏外扩量（第 34 轮）。它要盖住"形状本身的变化"，而不仅仅是位移：
   一帧之内 AABB 还会因为 ① 转身（绕 (x,y) 转 dθ，最远点位移 ≈ R·dθ）
   ② 身体摆动 ③ 吃食长大 而变。位移由"上帧 AABB ∪ 平移副本"覆盖，这三样靠这个 margin。
   值不是拍的：台架里有个自检会逐帧量"真实 AABB 比预测盒超出多少"，
   取全帧最大值 —— 见 KOI_DIRT_MARGIN 的读数（kb_probe_bbox_short）。 */
#define KOI_DIRT_MARGIN  4.0f
#define KMOUTH  (0.175f * 0.46f * 0.875f)
#define BITE_T    0.42f
#define BITE_SLOW 0.10f
/* 尾鳍立度：王总第 25 轮定档「25°」= TAIL_LV[1] */
#define TK_TL 0.93f
#define TK_TO 1.04f
#define TK_TW 1.07f
#define TK_FK 0.86f
/* 尾鳍衔接：定档「松」= TAIL_JOIN[2] */
#define TJ_SINK 0.34f
#define TJ_FLAP 1.32f
#define TJ_OP   0.15f
#define TJ_SHAD 0.05f
static const int TJ_GA[3] = {256, 179, 108};    // 红白：1.00 / 0.70 / 0.42
static const int TJ_GG[3] = {236, 159,  82};    // 黄金：0.92 / 0.62 / 0.32

/* ★★ 第 37 轮「尾鳍档」—— 修王总那句「两条鱼上下的话好像还要透明度 应该是不对的」。
   根因：鱼的画质降级档 `fine = (k->grow > 0.56f)` 里，**非 fine 档用真 alpha 画尾鳍**：
       if (fine) fill_poly(..., bodyCol, &g, 256);      // 不透明 + 渐变
       else      fill_poly(..., bodyCol, NULL, A[1]);   // ← A[1] = 179/159 = 0.70/0.62
   开局 grow = 0.40~0.68，有的鱼不过 0.56，于是**那一部分鱼的尾鳍是半透明的**；
   两条鱼一上一下叠起来，上面那条的尾鳍就把下面那条透出来了。
   ★ 这不是"该不该淡"的问题 —— "尾鳍后半段变淡"是想要的观感，但**不能用半透明去模拟**：
     半透明的结果色 = bodyCol·α + **背景**·(1−α)，背景里包括别的鱼；
     而"变淡"要的是 bodyCol 与**水色**混，跟背景是谁无关。
   正解：**在调色板重建时把淡色预混好（不透明）**，绘制时 alpha = 256。
     代价：预混用的水色只能是**一个常数**（取 WTOP/WBOT 的中点），所以鱼在屏幕上下移动时
     尾鳍会比原来偏亮/偏暗最多约 8（G 通道）—— 这是为了"不透出别的鱼"付的价，值。
   ⚠️ 档 0 = 现状（真 alpha，会透），用于逐字节对拍。 */
static uint8_t s_tailpale[2][3];        /* [0] 红白 [1] 黄金 */

static void build_tail_pale(void)
{
    const int A[2] = { TJ_GA[1], TJ_GG[1] };          /* 179 / 159 */
    for (int i = 0; i < 2; i++) {
        const uint8_t *b = s_pal[i == 0 ? PI_KBODY : PI_KGOLD];
        for (int c = 0; c < 3; c++) {
            int wm = ((int)s_pal[PI_WTOP][c] + (int)s_pal[PI_WBOT][c] + 1) >> 1;
            s_tailpale[i][c] = (uint8_t)(((int)b[c] * A[i] + wm * (256 - A[i]) + 128) >> 8);
        }
    }
}

typedef struct {
    float x, y, headA, phase, L, grow, curv, v, hz, waveAmp;
    float gaitTime, wanderT, wx, wy, eat, biteT;
    int   turnSide, seek, burst;
    uint8_t pat;                         // 0 kohaku / 1 gold / 2 sanke
    int   ns;
    float sp[4][5];                      // 红斑 [段, 段内 t, 半长, 半宽, 横向偏移]
    float bx0, by0, bx1, by1;            // ★ 本帧**预测**的脏区包围盒（step 算出、已含 margin）
    float ax0, ay0, ax1, ay1;            // ★ 上帧**真实绘制**包围盒（像素，含尾鳍摆幅
                                         //   包络），由 koi_draw 逐点累加；空 = ax1 < ax0
} koi_t;

static koi_t s_koi[MAX_KOI];
static int   s_nkoi = 5;

static uint32_t s_rnd = 20260919u;
static inline uint32_t rnd_u(void) { s_rnd = s_rnd * 1664525u + 1013904223u; return s_rnd; }
static inline float rnd_f(float a, float b)
{ return a + (b - a) * ((float)(rnd_u() >> 8) / 16777216.0f); }
static inline int rnd_i(int n) { return (int)((rnd_u() >> 8) % (uint32_t)(n > 0 ? n : 1)); }
static float _spx[KSEG + 1], _spy[KSEG + 1], _spa[KSEG + 1];
static float _lx[KSEG + 1], _ly[KSEG + 1], _rx[KSEG + 1], _ry[KSEG + 1];
/* clampf / angdiff / s_pts / s_npts / pt_push / pt_quad / s_poly / to_q8
   已上提到第 4b 节 —— 第 7b 节的荷叶浮萍也要用，放在这里就来不及了。 */

static void make_spots(koi_t *k)
{
    if (k->pat == 1) { k->ns = 0; return; }               // 黄金鲤不该有红斑
    int n = (k->pat == 2) ? (3 + (rnd_f(0, 1) < 0.5f ? 0 : 1))
                          : (2 + (rnd_f(0, 1) < 0.75f ? 1 : 0));
    float head = rnd_f(0, 0.6f);
    k->ns = n;
    for (int i = 0; i < n; i++) {
        int big = (i % 2 == 0);
        int sg = (int)(head + ((float)i / (float)n) * (KSEG - 0.4f));
        if (sg > KSEG - 1) sg = KSEG - 1;
        if (sg < 0) sg = 0;
        float tt = clampf(rnd_f(0.18f, 0.82f), 0.10f, 0.90f);
        float sl = big ? rnd_f(0.098f, 0.128f) : rnd_f(0.052f, 0.072f);
        float so = rnd_f(-0.13f, 0.13f);
        float hw = KDEPTH[sg] + (KDEPTH[sg + 1] - KDEPTH[sg]) * tt;
        float lim = hw - 0.06f - fabsf(so);
        if (lim < 0.30f) lim = 0.30f;
        float sw = big ? rnd_f(0.50f, 0.62f) : rnd_f(0.30f, 0.42f);
        if (sw > lim) sw = lim;
        k->sp[i][0] = (float)sg; k->sp[i][1] = tt;
        k->sp[i][2] = sl;        k->sp[i][3] = sw; k->sp[i][4] = so;
    }
}

static void make_koi(koi_t *k, float x, float y, float g0, uint8_t pat)
{
    memset(k, 0, sizeof(*k));
    k->x = x; k->y = y;
    k->headA = rnd_f(0, 6.2832f);
    k->phase = rnd_f(0, 6.28f);
    k->L = rnd_f(17.0f, 23.0f) * KOI_SCALE;
    k->grow = g0;
    k->hz = rnd_f(1.5f, 2.4f);
    k->waveAmp = 0.26f;
    k->burst = 1;
    k->gaitTime = rnd_f(0.3f, 1.0f);
    k->wanderT = rnd_f(0, 1.2f);
    k->wx = x; k->wy = y;
    k->pat = pat;
    make_spots(k);
    k->bx0 = k->bx1 = x; k->by0 = k->by1 = y;
    k->ax0 = 1e30f; k->ax1 = -1e30f;      /* 空 AABB —— 还没画过，step 会退回保守方框 */
    k->ay0 = 1e30f; k->ay1 = -1e30f;
}

static void pond_init(void)
{
    s_rnd = 20260919u;
    s_nlily = 0;
    /* ★ 鱼色由开局三屏的「分色」决定（网页版 initPond(curColors())）：
       pick_colors 把 s_split_kh 条红白**均匀交错**排进 s_nkoi 条里。
       ⚠️ 原来这里是一张固定的 pats[5] = {0,1,0,2,0}，里面有条 pat=2（大正三色）——
          开局界面只能分出"红白 / 御黄金"两种，进池子却冒出第三条三色鱼，
          跟玩家刚选完的条数与配色对不上。换成分色结果。
       ⚠️ pat 会改变 make_spots 抽随机数的**个数**，换配色会连带平移荷叶/波光 ——
          这是网页版本来的行为（见 pick_colors 的注释），不是 bug。 */
    uint8_t pats[MAX_KOI];
    pick_colors(pats, s_nkoi, s_split_kh);
    for (int i = 0; i < s_nkoi; i++) {
        float ia = rnd_f(0, 6.2832f), ir = rnd_f(0.10f, 0.68f);
        make_koi(&s_koi[i],
                 (float)KW * 0.5f + cosf(ia) * ((float)KW * 0.5f - 46.0f) * ir,
                 (float)KH * 0.5f + sinf(ia) * ((float)KH * 0.5f - 64.0f) * ir,
                 rnd_f(0.52f, 0.72f), pats[i]);
    }

    /* 荷叶：环形散布 + 最小间距检查，避免叠成一片。
       ★ 位置必须夹在「鱼」与「波带」之间 —— 全局 rnd 是一条序列，
         插错地方会让波带和波光点整体平移（网页版 initPond 也是这个次序）。 */
    {
        int guard = 0;
        while (s_nlily < MAX_LILY && guard++ < 900) {
            float ang = rnd_f(0, 6.2832f), rad = rnd_f(0.30f, 0.96f);
            float x = clampf((float)KW * 0.5f + fcos_t(ang) * 116.0f * rad,
                             20.0f, (float)KW - 20.0f);
            float y = clampf((float)KH * 0.5f + fsin_t(ang) * 152.0f * rad,
                             24.0f, (float)KH - 24.0f);
            float r = rnd_f(13.0f, 21.0f);
            int ok = 1;
            for (int q = 0; q < s_nlily; q++) {
                if (hypotf(s_lily[q].x - x, s_lily[q].y - y) <
                    s_lily[q].r + r + 6.0f) { ok = 0; break; }
            }
            if (!ok) continue;
            lily_t *L = &s_lily[s_nlily++];
            L->x = x; L->y = y; L->r = r;
            L->rot  = rnd_f(0, 6.28f);
            L->spin = rnd_f(-0.07f, 0.07f);
            L->bob  = rnd_f(0, 6.28f);
            /* ★★ 第 37 轮：0.26~0.36 → 0.13~0.25（照 HTML 原版 `gap:rnd(0.13,0.25)`）。
               29 轮把它翻倍的理由是"开口被 1px 描边糊掉、看不出来" —— 但真正的根因在
               **顶点位置**（当时被我挪到了 0.34r，缺口从"细缝"变成"咬掉一角"），
               加宽角度只是在补偿一个被我改坏的形状，结果越走越远。
               顶点回圆心之后，叶缘开口宽 = 2r·sin(gap)：r=17 时 4.4~8.4px，
               减掉两侧描边各 0.5px 仍有 3.4~7.4px —— 参考图里就是这个量级。
               ⚠️ 只改**取值范围**，rnd 调用次数不变 → 全局随机序列一动不动。 */
            L->gap  = rnd_f(0.13f, 0.25f);
            /* 跨档门的上一次档位：给一个大得不可能被取到的值，
               保证第一帧一定标脏（量化后 bob∈{-1,0,1}、rot 有界，都撞不到 1e30） */
            L->qbob = 1e30f; L->qrot = 1e30f;
            L->ws0 = 0; L->wsn = 0;
        }
    }
    build_weeds();                      /* 浮萍是荷叶的伴生，必须跟在荷叶之后 */

    /* ★★★ 波带已删（王总定"波带不要了"），但下面这 5×7 = 35 次 rnd_f **必须照样抽掉**。
       全局 rnd 是一条序列，不是每处独立随机源。这段夹在「荷叶/浮萍」与「波光点」之间，
       少抽一脚，紧接着的 44 个波光点的 x/y/phase/size 会整体前移一格 ——
       表现是"水面的闪光点换了位置"，而画面照样能跑，所以极难被发现。
       同一个坑第 22 轮踩过一次：网页版浮萍那个没用的 hi 字段照样占一脚随机数。
       抽出来的值直接丢弃（(void)），没有别的作用。 */
    for (int b = 0; b < 5; b++) {
        (void)rnd_f(24, KH - 24); (void)rnd_f(4, 11);
        (void)rnd_f(0.010f, 0.020f); (void)rnd_f(0, 6.28f);
        (void)rnd_f(0.035f, 0.068f); (void)rnd_f(7, 18);
        (void)rnd_f(2, 5);
    }
    for (int i = 0; i < 44; i++) {
        s_spark[i].x = rnd_f(0, KW - 3);
        s_spark[i].y = rnd_f(0, KH - 3);
        s_spark[i].phase = rnd_f(0, 6.28f);
        s_spark[i].size = (rnd_f(0.7f, 1.7f) + 0.5f >= 2.0f) ? 2 : 1;
    }
}

static void koi_spine(const koi_t *k)
{
    float seg = (k->L * k->grow) / (float)KSEG * 0.98f;
    float px = k->x, py = k->y, a = k->headA;
    _spx[0] = px; _spy[0] = py; _spa[0] = a;
    for (int i = 0; i < KSEG; i++) {
        a += k->curv * KBEND[i] + k->waveAmp * KAMP[i] * fsin_t(k->phase - (float)i * KPHASE);
        px -= fcos_t(a) * seg;
        py -= fsin_t(a) * seg;
        _spx[i + 1] = px; _spy[i + 1] = py; _spa[i + 1] = a;
    }
}

/* 身体轮廓：头圆帽（必须单独绕一段二次曲线，否则头是尖的）→ 右缘 → 尾柄 → 左缘 */
static void body_pts(void)
{
    s_npts = 0;
    float ha = _spa[0];
    float hw = sqrtf((_lx[0] - _spx[0]) * (_lx[0] - _spx[0]) +
                     (_ly[0] - _spy[0]) * (_ly[0] - _spy[0]));
    pt_push(_lx[0], _ly[0]);
    pt_quad(_lx[0], _ly[0],
            _spx[0] + fcos_t(ha) * hw * 2.50f, _spy[0] + fsin_t(ha) * hw * 2.50f,
            _rx[0], _ry[0]);                                 /* ★ 第 41 轮：head cap 1.75→2.50（让头部更圆，王总要"鱼头有点尖"） */
    for (int i = 0; i < KSEG; i++) {
        pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
                _rx[i], _ry[i],
                (_rx[i] + _rx[i + 1]) * 0.5f, (_ry[i] + _ry[i + 1]) * 0.5f);
    }
    pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
            _rx[KSEG], _ry[KSEG], _lx[KSEG], _ly[KSEG]);
    for (int i = KSEG; i > 0; i--) {
        pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
                _lx[i], _ly[i],
                (_lx[i] + _lx[i - 1]) * 0.5f, (_ly[i] + _ly[i - 1]) * 0.5f);
    }
}

static void ellipse_pts(float cx, float cy, float rx, float ry, float rot, int n)
{
    s_npts = 0;
    /* ★★ 第 36 轮：rot 的 cos/sin 提到循环外。
       原式**每个点**都把 fcos_t(rot) / fsin_t(rot) 各算两遍（一共 4 次三角查表），
       而 rot 在整个循环里是常量 —— 白烧。台架量到三角函数 **7631 次/帧**，
       这条椭圆路（红斑 14 点 + 墨斑 12 点 + 涟漪/饲料同族）占其中一大块。
       提到外面后每点只剩 t 的 cos/sin 两次。
       fcos_t/fsin_t 是纯函数、无副作用，所以这是**逐位等价**的改写，
       不靠推导靠证据：改完必须过 koi_bin_diff.py。 */
    float cr = fcos_t(rot), sr = fsin_t(rot);
    for (int i = 0; i < n; i++) {
        float t = 2.0f * KOI_PI * (float)i / (float)n;
        float ex = fcos_t(t) * rx, ey = fsin_t(t) * ry;
        pt_push(cx + ex * cr - ey * sr,
                cy + ex * sr + ey * cr);
    }
}

/* 尾鳍局部点列：rx0 = 根部起笔的局部 x（填充传 0 = 落在身体内部；描边传 sink） */
static void tail_local(float rx0, float tl, float to, float tw, float fk, float flap)
{
    s_npts = 0;
    pt_push(rx0, -to);
    pt_quad(rx0, -to, tl * 0.40f, -to * 0.82f + flap * 0.30f, tl * 0.84f, -tw + flap);
    pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
            tl * 1.10f, -tw * 0.30f + flap, tl * 0.82f * fk, flap * 0.46f);
    pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
            tl * 1.10f, tw * 0.30f + flap, tl * 0.84f, tw + flap);
    pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
            tl * 0.40f, to * 0.82f + flap * 0.30f, rx0, to);
    pt_push(rx0 - tl * 0.05f, 0.0f);
}

/* 局部 → 世界（绕原点旋转 + 平移） */
static void tail_to_world(int n, float bx, float by, float ca, float sa)
{
    for (int i = 0; i < n; i++) {
        float px = s_pts[i * 2], py = s_pts[i * 2 + 1];
        s_pts[i * 2]     = bx + px * ca - py * sa;
        s_pts[i * 2 + 1] = by + px * sa + py * ca;
    }
}

static void koi_draw(koi_t *k)
{
    PROF2_START();
    s_koi_n++;
    s_bb_on = 1; bb_begin();          /* ★ 全程累加真实 AABB（收尾写回 k->ax0..） */
    koi_spine(k);
    float L = k->L * k->grow * (1.0f + 0.09f * (k->eat > 0 ? k->eat / 0.6f : 0.0f));
    float Wd = L * 0.155f;                                  /* ★ 第 41 轮：0.175→0.155（收窄；KDEPTH 同时重排让头相对更宽，整体看着不"胖"） */
    int fine = (k->grow > 0.56f);
    int isGold = (k->pat == 1);
    const uint8_t *bodyCol = isGold ? s_pal[PI_KGOLD] : s_pal[PI_KBODY];

    for (int i = 0; i <= KSEG; i++) {
        float w = Wd * KDEPTH[i];
        float nx = -fsin_t(_spa[i]), ny = fcos_t(_spa[i]);
        _lx[i] = _spx[i] + nx * w; _ly[i] = _spy[i] + ny * w;
        _rx[i] = _spx[i] - nx * w; _ry[i] = _spy[i] - ny * w;
    }
    PROF2_TICK(0);                       /* 0 = spine + 左右轮廓准备 */

    /* ★★ 第 36 轮：`body_pts()` 一帧只跑一次（原来投影、鱼身各跑一遍）。
       body_pts 是**纯函数** —— 只读 _lx/_ly/_rx/_ry/_spa，写 s_pts/s_npts，
       连跑两遍得到的 s_pts 逐位相同，第二遍纯属白烧：
       5 次 pt_quad × 4 个采样点 × 每个点 6 乘 4 加（全是软浮点）≈ 上百次库调用/条/帧。
       ⚠️ 合并后 s_pts 保持"投影时算出来的那一份"，② 段下面的 save[] 正是存它给
          stroke_pts 用 —— 值与原来第二遍重建的一模一样，所以下游不受影响。
       ⚠️ 顺序不能动：投影必须先于鱼身画（压在上面）。
       ⚠️ 投影段（PROF2_TICK(1)）仍包含 body_pts 的成本，口径不变；只有 ② 段变便宜。 */
    body_pts();
    {
        static const uint8_t shcol[3] = {1, 9, 7};
        to_q8(s_npts, 1.8f, 2.6f);
        fill_poly(s_poly, s_npts, shcol, NULL, 56);          // 0.22
    }
    PROF2_TICK(1);                                           /* 1 = ①水下投影 */

    /* ② 鱼身 + 体缘暗线（复用上面那一份点列，不再 body_pts） */
    to_q8(s_npts, 0.0f, 0.0f);
    fill_poly(s_poly, s_npts, bodyCol, NULL, 256);
    {
        float save[NPTS * 2];
        int n = s_npts;
        for (int i = 0; i < n * 2; i++) save[i] = s_pts[i];
        stroke_pts(save, n, 1, 0.7f, s_pal[PI_KEDGE], 133);  // 0.52
    }
    PROF2_TICK(2);                                           /* 2 = ②鱼身 + 体缘暗线 */

    /* ③ 红斑（黄金鲤没有；大正三色另加墨斑） */
    for (int s = 0; s < k->ns; s++) {
        int sg = (int)k->sp[s][0];
        float tt = k->sp[s][1];
        float aa = _spa[sg];
        float ax = _spx[sg] + (_spx[sg + 1] - _spx[sg]) * tt;
        float ay = _spy[sg] + (_spy[sg + 1] - _spy[sg]) * tt;
        float off = k->sp[s][4] * Wd;
        ellipse_pts(ax - fsin_t(aa) * off, ay + fcos_t(aa) * off,
                    L * k->sp[s][2], Wd * k->sp[s][3], aa, 14);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, s_pal[PI_KSPOT], NULL, 236);   // 0.92
    }
    if (k->pat == 2) {
        static const uint8_t ink[3] = {26, 24, 30};
        ellipse_pts(_spx[2], _spy[2], L * 0.055f, Wd * 0.20f, _spa[2], 12);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, ink, NULL, 159);
    }
    PROF2_TICK(3);                                           /* 3 = ③红斑 / 墨斑 */

    /* ④ 脊背高光：只填纯色就是纸片，一条亮带才有圆柱体积感
       —— ★ 第 41 轮整段关闭：王总原话"鱼的骨骼显现出来了，需要隐藏"
       （这条白高光在白鱼（红白）身上看起来就像一条脊骨）。
       体积感由 ① 鱼身渐变与 ③ 红斑本身承担，少这条高光鱼不会变纸片。 */
    PROF2_TICK(4);                                           /* 4 = ④脊背高光（已关） */

    /* ⑤ 尾鳍：根部一律埋进身体（局部 x=0 起笔）盖住身体尾端那道横截面；
       描边才从"露出身体那一点"（sink）起笔 —— 第 24 轮定死的口径。 */
    {
        float tpx = _spx[KSEG], tpy = _spy[KSEG];
        float segLen = sqrtf((tpx - _spx[KSEG - 1]) * (tpx - _spx[KSEG - 1]) +
                             (tpy - _spy[KSEG - 1]) * (tpy - _spy[KSEG - 1]));
        float sink = segLen * TJ_SINK;
        float flap = fsin_t(k->phase - (float)KSEG * KPHASE - 0.85f) *
                     (0.16f + k->waveAmp * 0.95f) * L * 0.20f * TJ_FLAP;
        float tl = L * 0.238f * TK_TL + sink;
        float to = Wd * KDEPTH[KSEG] * 1.10f * TK_TO;
        float tw = L * 0.094f * TK_TW;
        float bx = tpx + fcos_t(_spa[KSEG]) * sink;
        float by = tpy + fsin_t(_spa[KSEG]) * sink;
        float ca = fcos_t(_spa[KSEG] + KOI_PI), sa = fsin_t(_spa[KSEG] + KOI_PI);

        if (TJ_SHAD > 0.0f) {                                 // ⑤a 水下投影（与身体同偏移）
            static const uint8_t shcol[3] = {1, 9, 7};
            tail_local(0.0f, tl, to, tw, TK_FK, flap);
            tail_to_world(s_npts, bx + 1.8f, by + 2.6f, ca, sa);
            to_q8(s_npts, 0.0f, 0.0f);
            fill_poly(s_poly, s_npts, shcol, NULL, (int)(TJ_SHAD * 256.0f));
        }

        tail_local(0.0f, tl, to, tw, TK_FK, flap);            // ⑤b 鳍身
        tail_to_world(s_npts, bx, by, ca, sa);
        {
            float save[NPTS * 2];
            int n = s_npts;
            for (int i = 0; i < n * 2; i++) save[i] = s_pts[i];
            to_q8(n, 0.0f, 0.0f);
            int nend = (int)ceilf(sink) + 3;
            if (nend < (int)rne_f2i(tl)) nend = (int)rne_f2i(tl);
            int hold = (int)ceilf(sink) + 1;
            if (hold > nend - 1) hold = nend - 1;
            int mid = hold + (int)((float)(nend - hold) * TJ_OP);
            if (mid < hold + 1) mid = hold + 1;
            const int *A = isGold ? TJ_GG : TJ_GA;
            grad_t g;
            g.ux = (int32_t)rne_f2i(ca * 256.0f);
            g.uy = (int32_t)rne_f2i(sa * 256.0f);
            g.cx = (int32_t)rne_f2i(bx * 256.0f);
            g.cy = (int32_t)rne_f2i(by * 256.0f);
            g.s0 = 0;          g.a0 = A[0];
            g.s1 = hold * 256; g.a1 = A[0];
            g.s2 = mid * 256;  g.a2 = A[1];
            g.s3 = nend * 256; g.a3 = A[2];
            /* ★ 第 39 轮**不动这里**。曾试过把档 2 提到最前当"鱼要实体感"的解，
               但那样等于**擅自改立度**（档 0 的 21° → 档 2 的 30°），超出"就做这两个事"。
               本轮"实体感"由 fill_poly 的根因修复解决（鱼身 α 0.75 → 1.00）。
               遗留（**下一轮单独做**）：`s_look_tail >= 2` 排在 `if (fine)` 之后，
               而 fine = (grow > 0.56)，开局 6 条鱼 grow 0.56~0.68 ⇒ **档 2 从没覆盖过
               细档鱼的渐变尾鳍**（TJ_GA = 256/179/108，尾梢只剩 0.42）—— 真缺陷。 */
            if (fine) fill_poly(s_poly, n, bodyCol, &g, 256);
            else if (s_look_tail >= 2) fill_poly(s_poly, n, bodyCol, NULL, 256);
            else if (s_look_tail == 1) fill_poly(s_poly, n, s_tailpale[isGold ? 1 : 0], NULL, 256);
            else      fill_poly(s_poly, n, bodyCol, NULL, A[1]);

            if (fine) {                                       // 描边：只在"露出来"那一段
                tail_local(sink, tl, to, tw, TK_FK, flap);
                tail_to_world(s_npts, bx, by, ca, sa);
                stroke_pts(s_pts, s_npts, 0, 0.8f, s_pal[PI_KFIN], 128);   // 0.50
            }
            (void)save;
        }

        /* ★ 尾鳍摆动包络（第 34 轮）—— 拖影的正解在这里。
           尾鳍末端一帧里能横摆 ±flapM（flap 由 phase 连续推进，下一帧摆到哪不可预知），
           而报脏用的是"上帧 AABB"，所以它必须**覆盖整条摆动区间**：多算两个极端姿态
           的点列并进 AABB。代价只有两次 tail_local + tail_to_world（纯浮点，不光栅化）。 */
        if (s_bb_on) {
            float flapM = (0.16f + k->waveAmp * 0.95f) * L * 0.20f * TJ_FLAP;
            for (int e = 0; e < 2; e++) {
                tail_local(sink, tl, to, tw, TK_FK, flap + (e == 0 ? -flapM : flapM));
                tail_to_world(s_npts, bx, by, ca, sa);
                bb_pts(s_pts, s_npts);
            }
        }
    }
    PROF2_TICK(5);                                           /* 5 = ⑤尾鳍 */

    /* ⑥ 胸鳍（左右交替划水） */
    if (fine) {
        float px0 = _spx[1] + (_spx[2] - _spx[1]) * 0.28f;
        float py0 = _spy[1] + (_spy[2] - _spy[1]) * 0.28f;
        float fca = fcos_t(_spa[1]), fsa = fsin_t(_spa[1]);
        for (int si = 0; si < 2; si++) {
            float sd = (si == 0) ? -1.0f : 1.0f;
            float pad = 0.5f + 0.5f * fsin_t(k->phase * 0.62f + (sd > 0 ? 0.0f : KOI_PI));
            float pw = Wd * (1.00f + 0.50f * pad);
            float pl = L * (0.15f + 0.09f * pad);
            float r0 = Wd * KDEPTH[1] * 0.66f;
            s_npts = 0;
            pt_push(0.0f, sd * r0);
            pt_quad(0.0f, sd * r0, -pl * 0.34f, sd * (r0 + pw * 0.94f),
                    -pl * 0.58f, sd * (r0 + pw * 0.90f));
            pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
                    -pl * 0.86f, sd * (r0 + pw * 0.62f), -pl * 0.78f, sd * (r0 + pw * 0.28f));
            pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
                    -pl * 0.70f, sd * (r0 + pw * 0.05f), -pl * 0.38f, sd * r0 * 1.06f);
            pt_quad(s_pts[(s_npts - 1) * 2], s_pts[(s_npts - 1) * 2 + 1],
                    -pl * 0.16f, sd * r0 * 1.04f, -pl * 0.05f, sd * r0 * 0.96f);
            tail_to_world(s_npts, px0, py0, fca, fsa);
            to_q8(s_npts, 0.0f, 0.0f);
            /* ★ 第 39 轮**不动这里**：胸鳍 0.62 是"水里一片薄鳍"的设计，
               而且它盖在**鱼身**上（不是压在背景上），半透明读成"与身体融合"而非"透视"。
               本轮只修"鱼身整体蒙水色"那个根因（α 0.75 → 1.00）。 */
            fill_poly(s_poly, s_npts, bodyCol, NULL, 159);    // 0.62
        }
    }
    PROF2_TICK(6);                                           /* 6 = ⑥胸鳍 */

    /* ★ 收尾：写回真实 AABB（+1px 抗锯齿余量）。
       注意这里用的是 **上帧 AABB** 语义 —— step 在下一帧开头才读它，
       那时它描述的是"鱼上一帧画在哪"，正好是这一帧需要擦掉的范围。 */
    s_bb_on = 0;
    if (!bb_empty()) {
        k->ax0 = (float)(s_bbx0 - 1); k->ay0 = (float)(s_bby0 - 1);
        k->ax1 = (float)(s_bbx1 + 2); k->ay1 = (float)(s_bby1 + 2);
    }

#ifdef KOI_HOST_PROBE
    /* ★ 自检：**预测的脏盒够不够**。step 报脏时用的是"上帧 AABB 绕 (x,y) 转 Δθ
       再平移 + margin"，这里是本帧画完之后的**真值**。超出多少 = 那一侧会残留多少。
       逐帧取最大值 —— 它就是 KOI_DIRT_MARGIN 的取值依据，别再手拍。
       ⚠️ 只统计池子里那几条（s_koi）：setup_koi_icon 是拿一个**栈上的临时 koi_t**
          画的，它的 bx0.. 从来没被 step 写过（=0），混进来会读出 176px 这种假欠缺，
          把真读数盖掉（假 FAIL 比 FAIL 更坏，铁律 11）。 */
    if (k >= s_koi && k < s_koi + MAX_KOI) {
        float d0 = k->bx0 - k->ax0, d1 = k->ax1 - k->bx1;
        float d2 = k->by0 - k->ay0, d3 = k->ay1 - k->by1;
        if (d0 > s_bbox_short) s_bbox_short = d0;
        if (d1 > s_bbox_short) s_bbox_short = d1;
        if (d2 > s_bbox_short) s_bbox_short = d2;
        if (d3 > s_bbox_short) s_bbox_short = d3;
    }
#endif
}

/* 鱼的绘制包围盒：优先用 koi_draw 逐点累加的真实 AABB；
   还没画过（刚建池子、第一帧）时退回保守方框 —— 那个方框只在这唯一一帧用到，
   而这一帧是 s_full 整屏重绘，所以宽一点无所谓。 */
static void koi_bbox(const koi_t *k, float *x0, float *y0, float *x1, float *y1)
{
    if (k->ax1 >= k->ax0) {
        *x0 = k->ax0; *y0 = k->ay0; *x1 = k->ax1; *y1 = k->ay1;
    } else {
        float r = k->L * k->grow * 0.85f + 12.0f;
        *x0 = k->x - r; *y0 = k->y - r; *x1 = k->x + r; *y1 = k->y + r;
    }
}

/* ==========================================================================
   9. 涟漪（整数定点环带，与 koi_sim.py 同名函数同式）
   ========================================================================== */
#define RING_GAP_V  13.0f
#define RING_LIFE_V 0.82f
#define RING_R0_V   3.0f
#define RING_RMAX_V 46.0f
#define RING_F0_V   0.109f
#define RING_F1_V   0.326f
#define RING_N_V    3
#define SPLASH_MIN_V 26.0f

typedef struct {
    float x, y, r0, rMax, life, age, a0, cg;
    int   kind;                     // 0 tap / 1 drop / 2 eat
    int   cn;
} rip_t;

static rip_t s_rip[MAX_RIP];
static int   s_nrip;

static void ripple_draw(const rip_t *rp)
{
    float p = rp->age / rp->life;
    if (p <= 0.0f || p >= 1.0f) return;
    float R = rp->r0 + (rp->rMax - rp->r0) * powf(p, 0.82f);
    float A = powf(1.0f - p, 0.85f) * 0.88f * rp->a0;
    if (A <= 0.016f || R < 0.5f) return;
    const uint8_t *col = s_pal[PI_RIPPLE];
    float f0 = rp->rMax * RING_F0_V, f1 = rp->rMax * RING_F1_V;
    int cx8 = (int)rne_f2i(rp->x * 8.0f) - 4;
    int cy8 = (int)rne_f2i(rp->y * 8.0f) - 4;
    for (int ki = 0; ki < rp->cn; ki++) {
        float rk = R - (float)ki * rp->cg;
        if (rk < 0.5f) break;
        float ak = A * clampf((rk - f0) / f1, 0.0f, 1.0f);
        if (ak <= 0.016f) continue;
        int a_core = (int)(ak * 256.0f);
        int a_halo = (int)(ak * 256.0f * 0.30f);
        int R8 = (int)rne_f2i(rk * 8.0f);
        int w_core = (int)rne_f2i((0.85f + rk * 0.005f) * 0.5f * 8.0f);
        int w_halo = (int)rne_f2i((1.7f + rk * 0.016f) * 0.5f * 8.0f);
        if (w_core < 1) w_core = 1;
        if (w_halo < 1) w_halo = 1;
        int lo1 = (R8 - w_core) * (R8 - w_core), hi1 = (R8 + w_core) * (R8 + w_core);
        int lo2 = (R8 - w_core - w_halo) * (R8 - w_core - w_halo);
        int hi2 = (R8 + w_core + w_halo) * (R8 + w_core + w_halo);
        int rad = (R8 + w_core + w_halo) / 8 + 2;
        int cxi = (int)rp->x, cyi = (int)rp->y;
        int y0 = cyi - rad, y1 = cyi + rad;
        if (y0 < s_cy0) y0 = s_cy0;
        if (y1 > s_cy1) y1 = s_cy1;
        for (int yy = y0; yy <= y1; yy++) {
            int dy8 = yy * 8 + 4 - cy8;
            int dy2 = dy8 * dy8;
            if (dy2 > hi2) continue;
            int x0 = cxi - rad, x1 = cxi + rad;
            if (x0 < s_cx0) x0 = s_cx0;
            if (x1 > s_cx1) x1 = s_cx1;
            for (int xx = x0; xx <= x1; xx++) {
                int dx8 = xx * 8 + 4 - cx8;
                int d2 = dx8 * dx8 + dy2;
                if (d2 >= lo1 && d2 <= hi1)      px_blend(xx, yy, col[0], col[1], col[2], a_core);
                else if (d2 >= lo2 && d2 <= hi2) px_blend(xx, yy, col[0], col[1], col[2], a_halo);
            }
        }
    }
}

/* ==========================================================================
   10. 饲料
   --------------------------------------------------------------------------
   ★ 第 34 轮两条改动，都对着王总原话：
     ①「撒饲料喂食的饲料颗粒数量太多 就十颗左右就可以」
        食物 14 → **FEED_N 10**；"雨点"8 → **DROP_N 4**（保留"像下雨"的观感）。
        ⚠️ 雨点也画圆（网页版同样不区分 food）—— 所以屏上最多看到 10+4 个圆，
           其中 4 个是从左上角飞进来的，落地只留一圈涟漪、不留食物。
     ②「饲料在水里不消散」
        网页版有 `foods[f].age += dt; if (age > 14) foods.splice(...)` ——
        **14 秒没被吃掉就散掉**，固件漏实现了 → 落水的饲料永久留在池底，
        喂两轮就积一片，鱼吃不完的永远不退。现在补上（FOOD_LIFE 14）。
        ⚠️ 消散那一刻**必须标脏**，否则池底会留下一颗不会消失的饲料幽灵。
   ========================================================================== */
#define FEED_N      9          /* ★ 第 41 轮：10→9（王总原话"保持在 8~9 粒"；drop_n 仍 4） */
#define DROP_N      4          /* 只出涟漪、不产食物的"雨点" */
#define FOOD_LIFE   2.0f       /* ★ 第 41 轮：14s→2s（王总要"落水后溅水花然后就消失"；
                                 留 2s 给鱼追一下吃，不至于吃不到；过 2s 直接消失不再留底） */

typedef struct { float x, y, sx, sy, tx, ty, t, dur, delay; int food; } pel_t;
static pel_t s_pel[MAX_PEL];
static int   s_npel;
static float s_food_x[MAX_FOOD], s_food_y[MAX_FOOD], s_food_age[MAX_FOOD];
static int   s_nfood;

static void pellets_draw(void)
{
    const uint8_t *col = s_pal[PI_PELLET];
    for (int i = 0; i < s_npel; i++) {
        const pel_t *pe = &s_pel[i];
        if (pe->delay > 0) continue;
        int a = (int)((0.45f + 0.55f * pe->t) * 256.0f);
        float pr = 1.5f * (0.6f + 0.4f * pe->t);
        ellipse_pts(pe->x, pe->y, pr, pr, 0.0f, 8);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, col, NULL, a);
    }
    for (int i = 0; i < s_nfood; i++) {                    // 已落定的食物
        ellipse_pts(s_food_x[i], s_food_y[i], 1.6f, 1.6f, 0.0f, 8);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, col, NULL, 256);
    }
}

/* ==========================================================================
   11. 动作
   ========================================================================== */
static void ripple_add(float x, float y, float r0, float rMax, float life,
                       float a0, int kind, int cn, float cg)
{
    if (s_nrip >= MAX_RIP) {
        memmove(&s_rip[0], &s_rip[1], sizeof(rip_t) * (MAX_RIP - 1));
        s_nrip = MAX_RIP - 1;
    }
    rip_t *r = &s_rip[s_nrip++];
    r->x = x; r->y = y; r->r0 = r0; r->rMax = rMax; r->life = life;
    r->age = 0; r->a0 = a0; r->kind = kind; r->cn = cn; r->cg = cg;
}

/* 小圆总量管制（分类记账 + 近邻去重），照抄网页版 splashOK */
static int splash_ok(int kind, float x, float y, int cap)
{
    int n = 0;
    for (int i = 0; i < s_nrip; i++) {
        if (s_rip[i].kind != kind) continue;
        if (s_rip[i].age >= s_rip[i].life) continue;
        n++;
        if (hypotf(s_rip[i].x - x, s_rip[i].y - y) < SPLASH_MIN_V) return 0;
    }
    return n < cap;
}

static float s_shakeT, s_shakeX, s_shakeY;
static float s_satiety = 0.5f;

static void do_tap(void)
{
    s_shakeX = rnd_f(60.0f, 180.0f);
    s_shakeY = rnd_f(80.0f, 240.0f);
    s_shakeT = 1.2f;
    for (int q = s_nrip - 1; q >= 0; q--)           // 连按不叠加：清掉上一次拍水
        if (s_rip[q].kind == 0) s_rip[q] = s_rip[--s_nrip];
    ripple_add(s_shakeX, s_shakeY, RING_R0_V, RING_RMAX_V, RING_LIFE_V, 1.0f,
               0, RING_N_V, RING_GAP_V);
}

static void do_feed(void)
{
    for (int i = 0; i < FEED_N; i++) {
        if (s_npel >= MAX_PEL) break;
        const koi_t *near = s_nkoi ? &s_koi[rnd_i(s_nkoi)] : NULL;
        float tx = near ? clampf(near->x + rnd_f(-40, 40), 16, KW - 16) : rnd_f(16, KW - 16);
        float ty = near ? clampf(near->y + rnd_f(-40, 40), 20, KH - 28) : rnd_f(20, KH - 28);
        safe_spot(tx, ty); tx = s_spotX; ty = s_spotY;    // 别撒到荷叶上（撒了看不见也吃不到）
        pel_t *pe = &s_pel[s_npel++];
        pe->sx = tx + rnd_f(-14, 14); pe->sy = ty - rnd_f(46, 80);
        pe->tx = tx; pe->ty = ty;
        pe->x = pe->sx; pe->y = pe->sy;
        pe->t = 0; pe->dur = rnd_f(0.34f, 0.52f); pe->delay = rnd_f(0, 0.55f);
        pe->food = 1;
    }
    for (int k = 0; k < DROP_N; k++) {              // 只出涟漪、不产食物的"雨点"
        if (s_npel >= MAX_PEL) break;
        pel_t *pe = &s_pel[s_npel++];
        pe->sx = 0; pe->sy = 0;
        safe_spot(rnd_f(10, KW - 10), rnd_f(16, KH - 22));
        pe->tx = s_spotX; pe->ty = s_spotY;
        pe->x = pe->tx; pe->y = pe->ty;
        pe->t = 0; pe->dur = 0.36f; pe->delay = rnd_f(0, 0.9f);
        pe->food = 0;
    }
}

/* ==========================================================================
   12. 推进
   ========================================================================== */
static void koi_step(koi_t *k, float dt)
{
    float kh = k->L * k->grow * 0.55f;

    float tx = 0, ty = 0;
    int ti = -1;
    float best = 1e9f;
    for (int m = 0; m < s_nfood; m++) {
        float dd = (s_food_x[m] - k->x) * (s_food_x[m] - k->x) +
                   (s_food_y[m] - k->y) * (s_food_y[m] - k->y);
        if (dd < best) { best = dd; ti = m; tx = s_food_x[m]; ty = s_food_y[m]; }
    }
    if (ti >= 0 && best > 340.0f * 340.0f) ti = -1;      // 感知半径 ≈ 全屏
    k->seek = (ti >= 0);

    float vx = fcos_t(k->headA), vy = fsin_t(k->headA);  // 前进惯性项，不能省
    if (ti >= 0) {
        float d = hypotf(tx - k->x, ty - k->y);
        if (d < 1.0f) d = 1.0f;
        vx += (tx - k->x) / d * 2.2f;
        vy += (ty - k->y) / d * 2.2f;
    } else {
        k->wanderT -= dt;
        if (k->wanderT <= 0) {
            k->wanderT = rnd_f(3.4f, 6.4f);
            for (int tr = 0; tr < 8; tr++) {
                float wa = rnd_f(0, 6.2832f), wr = sqrtf(rnd_f(0.10f, 1.0f));
                /* ★ 第 38 轮：体量 ×3 后 kh 最大 ≈103px，KW/2−44−kh 会变**负**
                   → 目标点被甩到中心对面、鱼来回抽搐。加下限即可：
                   kh 小的时候（开局）这里读数和改动前**逐位相同**，只有大 kh 才起作用。 */
                float rwx = (float)KW * 0.5f - 44.0f - kh;
                float rwy = (float)KH * 0.5f - 54.0f - kh;
                if (rwx < 4.0f) rwx = 4.0f;
                if (rwy < 4.0f) rwy = 4.0f;
                k->wx = (float)KW * 0.5f + fcos_t(wa) * rwx * wr;
                k->wy = (float)KH * 0.5f + fsin_t(wa) * rwy * wr;
                if (hypotf(k->wx - k->x, k->wy - k->y) >= 48.0f &&
                    fabsf(angdiff(atan2f(k->wy - k->y, k->wx - k->x), k->headA)) <= 1.92f) break;
            }
        }
        float wdx = k->wx - k->x, wdy = k->wy - k->y;
        float wdd = hypotf(wdx, wdy);
        if (wdd < 36.0f) k->wanderT = 0.0f;              // 到点换目标，别在原点绕圈
        if (wdd < 0.6f) { wdx = 1.0f; wdy = 0.0f; wdd = 1.0f; }
        vx += wdx / wdd * 1.05f;
        vy += wdy / wdd * 1.05f;
    }

    if (s_shakeT > 0) {                                   // 受惊四散
        float sd = hypotf(k->x - s_shakeX, k->y - s_shakeY);
        if (sd < 1.0f) sd = 1.0f;
        vx += (k->x - s_shakeX) / sd * 3.0f;
        vy += (k->y - s_shakeY) / sd * 3.0f;
    }

    float m2 = 26.0f + kh;                                // 边界回避带（随体量缩放）
    /* ★ 第 38 轮：体量 ×3 后 m2 会超过半屏宽，左右两侧推力方向打架 → 鱼原地抖。
       钳到 0.40*KW（=96）以内，与下面 clamp 的可达区间一致。
       kh 小的时候（开局）m2 远小于这个上限，**读数与改动前逐位相同**。 */
    if (m2 > (float)KW * 0.40f) m2 = (float)KW * 0.40f;
    if (k->x < m2)              vx += (m2 - k->x) / m2 * 3.2f;
    if (k->x > KW - m2)         vx -= (k->x - (KW - m2)) / m2 * 3.2f;
    if (k->y < m2 + 6.0f)       vy += (m2 + 6.0f - k->y) / m2 * 3.2f;
    if (k->y > KH - m2 - 20.0f) vy -= (k->y - (KH - m2 - 20.0f)) / m2 * 3.2f;

    float sepW = k->seek ? 0.34f : 0.85f;                 // 抢食时别互推
    for (int o = 0; o < s_nkoi; o++) {
        if (&s_koi[o] == k) continue;
        float ox = k->x - s_koi[o].x, oy = k->y - s_koi[o].y;
        float od = hypotf(ox, oy);
        if (od < 26.0f * KOI_SCALE && od > 0.1f) { vx += ox / od * sepW; vy += oy / od * sepW; }
    }

    float err = angdiff(atan2f(vy, vx), k->headA);
    if (fabsf(err) > 2.60f) {
        if (k->turnSide == 0) k->turnSide = (err < 0 ? -1 : 1);
    } else if (fabsf(err) < 1.48f) {
        k->turnSide = 0;
    }
    float e = fabsf(err) / 1.15f;
    if (e > 1.0f) e = 1.0f;
    float shaped = powf(e, 1.7f) * (float)(k->turnSide ? k->turnSide : (err < 0 ? -1 : 1));
    float kk = dt * 4.2f; if (kk > 1.0f) kk = 1.0f;
    k->curv += (shaped * 0.40f - k->curv) * kk;
    k->headA += k->curv * 2.9f * dt;

    k->gaitTime -= dt;                                    // 步态 burst / coast
    if (k->gaitTime <= 0) {
        if (k->burst) { k->burst = 0; k->gaitTime = rnd_f(0.70f, 1.50f); }
        else          { k->burst = 1; k->gaitTime = rnd_f(0.30f, 0.55f); }
    }
    int power = k->seek || s_shakeT > 0.0f || fabsf(err) > 1.20f;
    int burst = power || k->burst;
    float ampT = k->seek ? 0.50f : (burst ? 0.38f : 0.24f);
    float wa_t = dt * (burst ? 5.0f : 2.6f); if (wa_t > 1.0f) wa_t = 1.0f;
    k->waveAmp += (ampT - k->waveAmp) * wa_t;
    float hzT = k->seek ? 3.0f : (burst ? 2.4f : 1.5f);
    float hz_t = dt * 3.6f; if (hz_t > 1.0f) hz_t = 1.0f;
    k->hz += (hzT - k->hz) * hz_t;
    k->phase += dt * k->hz * 6.2832f;
    if (!burst) k->phase += dt * 0.6f;                    // 滑行时相位继续推进

    float align = 0.5f + 0.5f * fcos_t(err);
    int biting = (k->biteT > 0);
    float vT = (k->seek ? 24.0f : (s_shakeT > 0 ? 26.0f : 19.0f))
             * (0.62f + 0.46f * k->grow)
             * (0.86f + 0.28f * s_satiety)
             * (0.42f + 0.58f * align) * KOI_SCALE
             * (biting ? BITE_SLOW : 1.0f);
    if (burst) {
        float vk = dt * (biting ? 10.0f : 3.8f); if (vk > 1.0f) vk = 1.0f;
        k->v += (vT - k->v) * vk;
    } else {
        float vk = dt * 0.72f; if (vk > 1.0f) vk = 1.0f;
        k->v -= k->v * vk;
    }
    k->v = clampf(k->v, 0.0f, 34.0f * KOI_SCALE);
    if (biting) k->biteT -= dt;

    k->x = clampf(k->x + fcos_t(k->headA) * k->v * dt, kh + 3.0f, KW - kh - 3.0f);
    k->y = clampf(k->y + fsin_t(k->headA) * k->v * dt, kh + 3.0f, KH - kh - 3.0f);

    /* 吃食：判定点 = 吻端 → 吃食圆也落在嘴上（第 18 轮口径） */
    float mx = k->x + fcos_t(k->headA) * k->L * k->grow * KMOUTH;
    float my = k->y + fsin_t(k->headA) * k->L * k->grow * KMOUTH;
    if (ti >= 0 && ti < s_nfood &&
        hypotf(tx - mx, ty - my) < k->L * k->grow * 0.0805f + 3.0f) {
        /* ★ 第 34 轮补标脏：这颗饲料的像素原来靠"鱼的大脏框顺手扫过"才被擦掉
           （判定点在吻端，食物必在鱼框内）—— 属于"靠画多了掩盖漏标"，
           脏区一旦收紧（本轮把鱼框换成真实 AABB）就会原形毕露成池底残留。
           现在自己报一份，两件事解耦。 */
        dirty_add_ext(floor_f2i(s_food_x[ti]) - 3, floor_f2i(s_food_y[ti]) - 3,
                      (int)ceilf(s_food_x[ti]) + 4, (int)ceilf(s_food_y[ti]) + 4);
        for (int i = ti; i + 1 < s_nfood; i++) {
            s_food_x[i] = s_food_x[i + 1]; s_food_y[i] = s_food_y[i + 1];
            s_food_age[i] = s_food_age[i + 1];
        }
        s_nfood--;
        s_satiety = clampf(s_satiety + 0.05f, 0.0f, 1.0f);
        k->eat = 0.6f;
        k->biteT = BITE_T;                                // 啄食停顿：圆灭之前嘴不离开圆
        k->grow = (k->grow + GROW_PER_PELLET > GROW_MAX) ? GROW_MAX
                                                         : k->grow + GROW_PER_PELLET;
        if (splash_ok(2, mx, my, 3)) ripple_add(mx, my, 1, 9, BITE_T, 0.70f, 2, 1, 0);
    }
}

/* 需要整屏重画的两种情形：开机第一帧（缓冲还是空的），以及昼夜过渡期
   （每一行的水色都在变，脏矩形帮不上忙）。其余时间一律走脏区。
   ★ 这个标志必须"稳态时归零" —— 第一版只在昼夜过渡里清它，白天稳态进不到那个
     分支，于是标志常年为 1、白天也每帧整屏重推，脏区优化形同虚设。
     是主机台架的日志"脏区=100.0% rect=0"把它揪出来的（不在板子上也看得见）。 */
static int s_first_frame = 1;

static void step(float dt)
{
#ifdef KOI_HOST_PROBE
    s_step_no++;
#endif
    s_time += dt;
    if (s_shakeT > 0) s_shakeT -= dt;

    /* ★★ 第 37 轮：昼夜过渡量化 —— 修王总「昼夜交换这个有点特别卡顿感」。
       根因：旧代码是
           s_night += dt;  s_full = 1;      // ← 过渡 1 秒 ≈ 24 帧，**每一帧都整屏重画**
         而且 build_palette 每帧都要重建调色板 + 重算 320 行水色（n8 每帧都在变）。
         24 帧连着撞 41.7ms 预算、全部超支 → 掉帧 = 肉眼看到的"卡顿感"。
       改法：过渡时长不变（还是 1 秒），但把亮度**量化成 N 阶**（N 由观感档给）：
         · 逻辑值 s_night_log 照旧每帧走 dt（保证 1 秒一定走完，不会"变慢"）；
         · 真正生效的 s_night 只取 k/N 这几个格点；
         · **只有跨格那一帧才 s_full = 1**，中间的帧 s_night 不动 →
           没有任何东西变 → 脏区为空 → 那一帧几乎不花时间。
       24 次整屏重画 → 8 次（每阶之间隔 3 帧，观感上仍是一条 1 秒的连续变暗）。
       ⚠️ 档 0 时走原路径（逐帧推进），用于逐字节对拍。 */
    if (s_night != s_nightTarget || s_night_log != s_night) {
        float st = dt;                                    // 昼夜过渡 1 秒
        float d = s_nightTarget - s_night_log;
        if (d >  st) d =  st;
        if (d < -st) d = -st;
        s_night_log += d;
        float ap = s_night_log;
        int lv = s_look_night;
        if (lv < 0) lv = 0;
        if (lv >= LOOK_NIGHT_NSTEP) lv = LOOK_NIGHT_NSTEP - 1;
        int ns = LOOK_NIGHT_STEPS[lv];
        if (ns > 1) {                     /* 0 = 不量化（逐帧推进 = 改动前的现状） */
            ap = floorf(s_night_log * (float)ns + 0.5f) / (float)ns;
            if (ap > 1.0f) ap = 1.0f;
            if (ap < 0.0f) ap = 0.0f;
        }
        if (ap != s_night) { s_night = ap; s_full = 1; }
    } else if (s_first_frame) {
        s_full = 1;
        s_first_frame = 0;
    }

    /* （原 bands_mark_dirty 在此 —— 波带删除后不再需要给它报脏区） */

    /* ★★ 第 34 轮：报脏换成**真实绘制包围盒**（铁律 15 的落实）。
       旧口径是"以 (x,y) 为中心、边长 0.75·L·grow+10 的方框"，而鱼连尾鳍往后要伸
       1.22·L、尾梢摆动还有横向行程 —— **尾梢整个在框外，擦不掉**，就是王总说的
       「有时候后面会有拖影」。
       实测量到的缺口（台架 金标准判据）：鱼3 中心 (135.73,200.22) 报的框到 y=227，
       而它的真实 AABB 到 y=231 → 尾巴那 2 行永远留着上一帧的像素。
       新口径：**上一帧的真实 AABB ∪ 它平移本帧的位移**，再外扩 KOI_DIRT_MARGIN。
         · 为什么不是"本帧 AABB"：本帧的 AABB 要等画完才知道，而报脏必须在画之前。
         · 为什么能这么用：AABB 里已经含了**整个尾鳍摆动包络**（koi_draw 里用 bb_pts
           把 flap±flapM 两个极端姿态都累进去了），所以"形状"是稳定的，一帧之内
           只有整体位移与缓慢转角。位移由并集覆盖，转角/摆幅/进食长大由 margin 覆盖。
         · 首帧（还没画过）退回保守方框 —— 那个方框只在唯一一帧用到。
       ⚠️ 别再退回"方形包围盒"：那正是"框错了却被画多了盖住"的坑（第 20 轮），
          水面变便宜之后它的代价也从"白多填水"变成"白重画好几遍对象"。 */
    for (int i = 0; i < s_nkoi; i++) {
        koi_t *k = &s_koi[i];
        float ox = k->x, oy = k->y, oa = k->headA;
        koi_step(k, dt);
        float dx = k->x - ox, dy = k->y - oy;
        float da = k->headA - oa;
        float x0, y0, x1, y1;
        if (k->ax1 >= k->ax0) {
            /* ★ 上帧 AABB 先**绕 (x,y) 转 Δθ**、再**平移 disp** —— 这两步合起来
               才精确等于"本帧的包围盒"（鱼体是刚体绕自身 (x,y) 转、再整体平移；
               摆动相位与体长的残差由 KOI_DIRT_MARGIN 覆盖）。
               ⚠️ 只做"平移副本"是不够的：**转身**时包围盒会朝侧向长出去，
                  平移并集盖不住 —— 台架实测过，一侧欠缺 176px（那样尾巴会残留）。 */
            float ca = fcos_t(da), sa = fsin_t(da);
            float bx[4] = {k->ax0, k->ax1, k->ax0, k->ax1};
            float by[4] = {k->ay0, k->ay0, k->ay1, k->ay1};
            x0 = 1e30f; y0 = 1e30f; x1 = -1e30f; y1 = -1e30f;
            for (int c = 0; c < 4; c++) {
                float px = bx[c] - ox, py = by[c] - oy;
                float rx = ox + px * ca - py * sa + dx;
                float ry = oy + px * sa + py * ca + dy;
                if (rx < x0) x0 = rx;
                if (rx > x1) x1 = rx;
                if (ry < y0) y0 = ry;
                if (ry > y1) y1 = ry;
            }
            /* ∪ 上帧自身：这一帧要"擦掉"的范围（旧位置） */
            if (k->ax0 < x0) x0 = k->ax0;
            if (k->ay0 < y0) y0 = k->ay0;
            if (k->ax1 > x1) x1 = k->ax1;
            if (k->ay1 > y1) y1 = k->ay1;
        } else {                       /* 还没画过（建池第一帧）：保守方框，只此一帧 */
            float r = k->L * k->grow * 0.85f + 14.0f;
            x0 = ox - r; y0 = oy - r; x1 = ox + r; y1 = oy + r;
        }
        k->bx0 = x0 - KOI_DIRT_MARGIN; k->by0 = y0 - KOI_DIRT_MARGIN;
        k->bx1 = x1 + KOI_DIRT_MARGIN; k->by1 = y1 + KOI_DIRT_MARGIN;
        dirty_add_ext(floor_f2i(k->bx0), floor_f2i(k->by0),
                      (int)ceilf(k->bx1), (int)ceilf(k->by1));
#ifdef KOI_HOST_PROBE
        if (s_rtrace_from <= s_step_no && s_step_no <= s_rtrace_to) {
            printf("    [trace 帧%d] 鱼%d 报 (%d,%d)-(%d,%d) → 表内 %d 个\n",
                   s_step_no, i, floor_f2i(k->bx0), floor_f2i(k->by0),
                   (int)ceilf(k->bx1), (int)ceilf(k->by1), s_nrect);
            for (int r = 0; r < s_nrect; r++)
                printf("                #%d x[%d..%d] y[%d..%d]\n", r,
                       s_rc[r].x0, s_rc[r].x1, s_rc[r].y0, s_rc[r].y1);
        }
#endif
    }
#ifdef KOI_HOST_PROBE
    if (s_rtrace_from <= s_step_no && s_step_no <= s_rtrace_to)
        printf("    [trace 帧%d] 鱼循环结束：表内 %d 个（MAX_RECT=%d）\n",
               s_step_no, s_nrect, MAX_RECT);
#endif

    for (int i = s_npel - 1; i >= 0; i--) {               // 饲料下落 / 雨点落水
        pel_t *pe = &s_pel[i];
        if (pe->delay > 0) {
            pe->delay -= dt;
            if (pe->delay > 0) continue;                  /* 还在等：这帧不画也不报 */
            /* ★★ 第 34 轮修正④（金标准判据的第三个漏洞）：**延迟跨过 0 的那一帧**。
               报脏用的是**递减前**的 delay（>0 → continue），而 pellets_draw 判的是
               **递减后**的值（≤0 → 画）。于是"饲料第一次出现"的那一帧：
                 · 它被画出来了（画面真的变了）；
                 · 但它一个像素都没报脏。
               台架读数就是这一帧：「(140,68) 增量=水色 整屏=饲料混合色，盖着: 料1」，
               而且整帧只差 8~9 个像素 —— 正是那颗半透明饲料的大小。
               （首次出现 t=0，alpha = 0.45，必然是"混合色"，不会等于任何调色板原色。）
               修法：跨过 0 的这一刻补报一次当前位置。位置就是 (pe->x,pe->y) = 起始点，
               与 do_feed 里的 sx/sy 一致。**不推进 t**，行为与原来逐帧一致，
               只是把漏掉的那次报脏补上 —— 这条改动不改变任何一帧的动画。 */
            dirty_add_ext(floor_f2i(pe->x) - 3, floor_f2i(pe->y) - 3,
                          (int)ceilf(pe->x) + 4, (int)ceilf(pe->y) + 4);
            continue;
        }
        float px0 = pe->x, py0 = pe->y;
        pe->t += dt / pe->dur;
        if (pe->t >= 1.0f) {
            if (pe->food) {
                if (s_nfood < MAX_FOOD) {
                    s_food_x[s_nfood] = pe->tx; s_food_y[s_nfood] = pe->ty;
                    s_food_age[s_nfood] = 0.0f;                    /* 落水开始计时 */
                    s_nfood++;
                }
                /* ★ 第 41 轮：落水也溅一圈水花（王总要"落水后溅水花然后就消失"）
                   与 DROP_N 的雨点共用 kind=1 通道；SPLASH_PLAN.drop=6 给投喂留足。 */
                if (splash_ok(1, pe->tx, pe->ty, 6))
                    ripple_add(pe->tx, pe->ty, 1, rnd_f(7, 14), rnd_f(0.5f, 0.75f), 0.70f, 1, 1, 0);
            } else if (splash_ok(1, pe->tx, pe->ty, 6)) {
                ripple_add(pe->tx, pe->ty, 1, rnd_f(7, 14), rnd_f(0.5f, 0.75f), 0.70f, 1, 1, 0);
            }
            /* ★★ 第 34 轮修正②（金标准判据指出的第二个漏洞）：
               **落水那一帧，饲料画在落点，不是帧初位置**。
               走进本分支时 pe->t 已经 ≥ 1，pellet 不再按 (pe->x,pe->y) 画 ——
               它变成了 s_food[..]，画在 (pe->tx, pe->ty)。而旧写法只报了帧初位置
               px0/py0 ±3：下落弧线最后一跳就有 5~10px，落点整个在框外 →
               落点**一粒像素都没报到** → 食物落定后根本不在屏上，
               直到某条鱼 / 某个涟漪的脏区顺手扫过它才"偶然"被补上。
               台架金标准判据读到的就是那个窗口期：不一致像素「盖着: 料1 / 料6」。
               （这与上面注释里说的"靠画多了掩盖漏标"是同一类坑，只是换了个角色。）
               正解：报 [帧初位置 ∪ 落点] 的 min/max —— 既擦旧的（帧初位置），
               也画新的（落点）。⚠️ 别只报落点：帧初位置那一坨也要擦掉。 */
            int qx0 = floor_f2i(px0), qy0 = floor_f2i(py0);
            int qx1 = (int)ceilf(pe->tx), qy1 = (int)ceilf(pe->ty);
            if (qx1 < qx0) { int t = qx0; qx0 = qx1; qx1 = t; }
            if (qy1 < qy0) { int t = qy0; qy0 = qy1; qy1 = t; }
            dirty_add_ext(qx0 - 3, qy0 - 3, qx1 + 4, qy1 + 4);
            s_pel[i] = s_pel[--s_npel];
        } else {
            /* ★ 飞行途中也要标脏：不然旧位置的几粒饲料不会被重画，拖出一条"影子"。
               这一条原来是被鱼那个大方框**掩盖**着的（饲料的落点就在鱼附近，
               而鱼框有 60 多像素见方，顺手把饲料扫了进去）—— 属于"靠画多了掩盖漏标"，
               和铁律第 15 条里"脏区框错了却被过大的重绘面积盖住"是同一类坑。 */
            float ex0 = pe->x, ey0 = pe->y;
            float e = pe->t * pe->t;
            pe->x = pe->sx + (pe->tx - pe->sx) * e;
            pe->y = pe->sy + (pe->ty - pe->sy) * e;
            /* ★★ 第 34 轮修正：报脏框要先取 min/max。
               旧写法 `(旧位置-3 … 新位置+3)` **默认这一颗只往右下飞**；
               只要它这一帧是往左或往上走（落点 tx 在上一起点的左边就会），
               x0 > x1 就被 dirty_add_ext 开头的合法性检查整条丢掉 ——
               一个像素都不报，那几粒饲料的飞行轨迹就永远留在屏上。
               台架金标准判据直接把它指出来了：不一致像素"盖着: 料1"。 */
            int rx0 = floor_f2i(ex0), ry0 = floor_f2i(ey0);
            int rx1 = (int)ceilf(pe->x), ry1 = (int)ceilf(pe->y);
            if (rx1 < rx0) { int t = rx0; rx0 = rx1; rx1 = t; }
            if (ry1 < ry0) { int t = ry0; ry0 = ry1; ry1 = t; }
            dirty_add_ext(rx0 - 3, ry0 - 3, rx1 + 3, ry1 + 3);
#ifdef KOI_HOST_PROBE
            if (s_rtrace_from <= s_step_no && s_step_no <= s_rtrace_to) {
                printf("    [trace 帧%d] 料%d 飞行 (%.1f,%.1f)→(%.1f,%.1f) 报 %d,%d-%d,%d → 表内 %d 个\n",
                       s_step_no, i, ex0, ey0, pe->x, pe->y,
                       rx0 - 3, ry0 - 3, rx1 + 3, ry1 + 3, s_nrect);
                for (int r = 0; r < s_nrect; r++)
                    printf("                #%d x[%d..%d] y[%d..%d]\n", r,
                           s_rc[r].x0, s_rc[r].x1, s_rc[r].y0, s_rc[r].y1);
            }
#endif
        }
    }

    for (int i = s_nrip - 1; i >= 0; i--) {
        rip_t *rp = &s_rip[i];
        rp->age += dt;
        float rr = rp->rMax + 8.0f;
        dirty_add_ext((int)(rp->x - rr), (int)(rp->y - rr), (int)(rp->x + rr), (int)(rp->y + rr));
        if (rp->age >= rp->life) s_rip[i] = s_rip[--s_nrip];
    }
    /* 荷叶（含伴生浮萍）标脏 —— 但要过"跨档门"：只有 bob/rot 的量化档位真的跨了一格
       才报脏。原来是无条件每帧报，于是每帧都要重画 6 片叶子（真机 108ms/帧）。
       ★ 浮萍**不单独报脏区** —— 它长在叶缘 WEED_PAD 之内，落进这一个框里，
         而且它跟着荷叶的 bob 走，荷叶跨档时它自然一起被重画。 */
    for (int i = 0; i < s_nlily; i++) {
        lily_t *L = &s_lily[i];
        float qb = lily_bob(L), qr = lily_rot(L);
        if (qb == L->qbob && qr == L->qrot) continue;   /* 档位没跨 → 这帧不重画 */
        L->qbob = qb; L->qrot = qr;
        float ly = L->y + qb;
        float ex = L->r + WEED_PAD + LILY_SLOP;         /* +SLOP 覆盖旧档位，否则留 1~2px 残影 */
        dirty_add_ext(floor_f2i(L->x - ex), floor_f2i(ly - ex),
                      (int)ceilf(L->x + ex), (int)ceilf(ly + ex));
    }
    for (int i = s_nfood - 1; i >= 0; i--) {
        /* 漂浮食物老化：14 秒没被吃掉就散掉（网页版 foods[f].age > 14）。
           ★ 原来这里是"每帧给池底所有食物报一遍脏" —— 食物是静止的，这么报等于
             每帧把每一颗饲料的位置重画一次，而它们根本没变。改成**只在离开数组
             那一刻报脏**：被吃掉（koi_step 里）、或寿命到（这里）。
           ⚠️ 这个循环不抽 rnd，所以与全局随机序列无关，随便改。 */
        s_food_age[i] += dt;
        if (s_food_age[i] < FOOD_LIFE) continue;
        dirty_add_ext(floor_f2i(s_food_x[i]) - 3, floor_f2i(s_food_y[i]) - 3,
                      (int)ceilf(s_food_x[i]) + 4, (int)ceilf(s_food_y[i]) + 4);
        s_food_x[i] = s_food_x[s_nfood - 1];
        s_food_y[i] = s_food_y[s_nfood - 1];
        s_food_age[i] = s_food_age[s_nfood - 1];
        s_nfood--;
    }
}

/* ==========================================================================
   13. 绘制
   ========================================================================== */
/* （分段计时探针已上提到第 2b 节 —— lily_paint / koi_draw 都在本节之前就要用它） */

static int rect_hits(int x0, int y0, int x1, int y1, float bx0, float by0, float bx1, float by1)
{
    return !(bx0 > (float)x1 || bx1 < (float)x0 || by0 > (float)y1 || by1 < (float)y0);
}

/* ★★ 第 34 轮踩坑记录：**"独占矩形去重"这个方案是错的，已撤。**
   想做的事（真机读数 鱼画=7.90/6、荷画=8.23/6，一倍是白烧）本身没错，
   但判据漏了一层：scene_draw 每进一个矩形都先 water_rect **把整块 clip 填成水**。
   于是"在别的矩形里跳过某条鱼/某片荷叶"= 它被水擦掉之后再也没补回来 ——
   鱼的像素直接留在荷叶上，王总看到的就是「荷叶下方有多余的杂颜色」。
   实测证据（台架探针，报坐标+颜色）：荷叶内部某点旧(172,93,115) 新(255,255,131)，
   x 逐帧 +1，速度正是 1px/帧的运动物体 —— 就是鱼。
   正解要么让水**跳过**被遮挡的像素（需要掩码，代价另算），要么就别去重。
   撤销后实测：荷叶内部 200 帧零变化。
   （顺带：老版本真正的"荷叶杂色"是另一回事 —— 旧脏框漏了尾梢，尾梢残影留在
     荷叶上；那个已经由真实 AABB 修掉了，与这里无关。） */

/* ri 参数留着不用（-1 永远成立），只是为了不改调用点形状；去重已撤。 */
static void scene_draw(int x0, int y0, int x1, int y1, int ri)
{
    (void)ri;
    set_clip(x0, y0, x1, y1);
    water_rect(x0, y0, x1, y1);
    PROF_TICK(3);
    for (int i = 0; i < s_nkoi; i++) {
        koi_t *k = &s_koi[i];
        float bx0, by0, bx1, by1;
        koi_bbox(k, &bx0, &by0, &bx1, &by1);   /* 真实 AABB，与报脏同一口径 */
        if (!rect_hits(x0, y0, x1, y1, bx0, by0, bx1, by1)) continue;
#ifdef KOI_HOST_PROBE
        if (s_hide_koi0 && i == 0) continue;   /* ★ 39 轮叶下鱼台架：只掐这一条的绘制 */
#endif
        demo_koi_who(6, i);
        koi_draw(k);
    }
    PROF_TICK(5);
    for (int i = 0; i < s_nrip; i++) {
        float rr = s_rip[i].rMax + 8.0f;
        if (!rect_hits(x0, y0, x1, y1, s_rip[i].x - rr, s_rip[i].y - rr,
                       s_rip[i].x + rr, s_rip[i].y + rr)) continue;
        demo_koi_who(3, i);
        ripple_draw(&s_rip[i]);
    }
    PROF_TICK(6);
    /* 荷叶与浮萍排在涟漪之后、饲料之前（与网页版层序一致）：
       它们浮在水面上，压在涟漪之上；饲料最后撒，压在荷叶之上。
       浮萍先画、荷叶后画 —— 两者都浮在水面，荷叶更大更"高"，压着浮萍。 */
    for (int i = 0; i < s_nlily; i++) {
        const lily_t *L = &s_lily[i];
        float ly = L->y + lily_bob(L);
        float ex = L->r + WEED_PAD;
        if (!rect_hits(x0, y0, x1, y1, L->x - ex, ly - ex, L->x + ex, ly + ex)) continue;
        float dy = ly - L->y;
        /* ★ 归属探针：浮萍按**全局下标 k** 编号（它就是"那一片浮萍"的身份），
           荷叶按荷叶下标编号 —— 判据里一眼就能分清是叶还是萍。 */
        for (int k = L->ws0; k < L->ws0 + L->wsn; k++) {
            demo_koi_who(8, k);
            weed_paint(&s_weed[k], dy);
        }
        demo_koi_who(7, i);
        lily_paint(L, lily_rot(L));    /* 与标脏共用同一个量化角，画/报永远一致 */
    }
    PROF_TICK(7);
    demo_koi_who(4, -1);
    pellets_draw();
    PROF_TICK(8);
}

/* ==========================================================================
   14. 开局三屏（首页 / 选条数 / 分色）
   --------------------------------------------------------------------------
   网页版 scene='setup' 的三步，逐式移植（对应它的 drawSetup + 按键块）：
     0 首页    —— 王总给的整屏设计稿，一张 240x320 位图铺满，**只认 OK**
     1 选条数  —— 7 枚图章（↑ 上一档 / ↓ 下一档），OK 确认
     2 分色    —— 两张卡（↑ 红白+1 / ↓ 红白−1，御黄金反向补齐），OK 开始养
   条数选到 2（金玉成双）时固定"一红白一黄金"，**跳过第 2 屏**直接开养。

   ★ 为什么这三屏在固件里是"静态"的：
     网页版每帧重画水 + 波带 + 波光 + 暗罩（它是 canvas 全量重画，不进脏区账）。
     固件上照抄就是每帧整屏：30.7ms SPI 物理下限 + 十几毫秒 CPU ——
     24fps 的 41.7ms 预算当场击穿。所以这里**换屏那一帧整屏画一次，之后稳态脏区 = 0**：
     一块不动的过场屏，真机确实只该花一帧的成本。
     代价是水面的 44 个波光点不闪烁（网页版会呼吸），这一条写进文档，不当成 bug。

   ★ 资源全部来自 koi_assets.h（PC 端从网页版抽出，调色板已预打包成 RGB565）：
       首页 HM_* / 图章 NM_* / 文字 UI_*。
   ========================================================================== */

/* 状态变量（s_scene / s_setup_step / s_pick_* / PICK_N / default_split / pick_colors）
   在第 5 节 —— pond_init 配鱼色要用，放这里就太晚了。 */

static void setup_draw(void);    /* render() 要用，先声明（定义在本节末尾） */

/* ---- ① 首页整屏位图：索引数组 → 查表写帧缓冲 ------------------------------
   ★ koi_assets.h 里的 HM_IDX 是**解压后的索引数组**（1 B/px，raster 顺序），
     **不是 RLE 流**。RLE 只是网页版把位图塞进那份 HTML 用的容器，这里
     已经在 PC 端解完了 —— 所以既不需要解码、也没有变长循环，一条直写最快。
     整屏都是不透明设计稿，一次查表即可，不需要像素混合。 */
static void home_blit(void)
{
    demo_koi_who(9, -1);                 /* ★ 归属探针（第四处直写 s_fb 的快路径） */
    for (int i = 0; i < HM_PXN; i++) {
#ifdef KOI_HOST_PROBE
        s_who[i] = (unsigned char)s_who_cur;
#endif
        s_fb[i] = HM_PAL[(uint8_t)HM_IDX[i]];
    }
}

/* ---- ② 档位图章：116x116，**索引 0 = 透明** -------------------------------
   透明 = 那个像素不往帧缓冲写。（固件里省的是 CPU；SPI 照样扫过去，
   所以别把"透明"当成省带宽。） */
static void nm_blit(int ki, int cx, int cy)
{
    const nm_asset_t *a = &NM_ASSET[ki];
    const int x0 = cx - NM_W / 2, y0 = cy - NM_H / 2;
    /* 先把可见范围算成整数区间，省掉逐像素四次越界判断 */
    int sx = (x0 < 0) ? -x0 : 0;
    int sy = (y0 < 0) ? -y0 : 0;
    int ex = (x0 + NM_W > KW) ? (KW - x0) : NM_W;
    int ey = (y0 + NM_H > KH) ? (KH - y0) : NM_H;
    for (int y = sy; y < ey; y++) {
        const uint8_t *row = &a->idx[y * NM_W];
        uint16_t *dst = &s_fb[(y0 + y) * KW + x0];
        for (int x = sx; x < ex; x++) {
            uint8_t v = row[x];
            if (v && v < a->nc) dst[x] = a->pal[v];   /* 0 = 透明，跳过 */
        }
    }
}

/* ---- ③ UI 文字小图（RGB565 + A8），锚点 = 图中心 --------------------------
   对齐网页版 textAlign=center / textBaseline=middle。
   暗态另有单独的位图（UI_*_D），**不是**把亮态压暗 —— 网页版换的是另一套颜色，
   生成时已按 dim 色 + globalAlpha 0.44 合成好，这里直接用。 */
static void ui_draw(int id, int cx, int cy)
{
    const ui_img_t *im = &UI_IMG[id];
    int x0 = cx - im->w / 2, y0 = cy - im->h / 2;
    for (int y = 0; y < im->h; y++) {
        int py = y0 + y;
        if (py < s_cy0 || py > s_cy1) continue;
        const uint8_t *arow = &im->al[y * im->w];
        const uint16_t *prow = &im->px[y * im->w];
        for (int x = 0; x < im->w; x++) {
            int px = x0 + x;
            if (px < s_cx0 || px > s_cx1) continue;
            uint8_t a = arow[x];
            if (!a) continue;
            uint16_t v = prow[x];
            int r = (v >> 11) & 31, g = (v >> 5) & 63, b = v & 31;
            px_blend(px, py, (r * 255 + 15) / 31, (g * 255 + 31) / 63, (b * 255 + 15) / 31, a);
        }
    }
}

/* 整屏铺一层半透明暗罩（开局屏专用：字要压在亮水面上，没这层就飘）。
   不走 px_blend 是因为这里是 76800 次混合、每次都要过一遍裁剪判定；
   直接内联展开省掉那几次比较。只在换屏那一帧跑一次。 */
static void veil_full(int r, int g, int b, int a)
{
    demo_koi_who(9, -1);                 /* ★ 归属探针：暗罩（第三处直写 s_fb 的快路径） */
    for (int i = 0; i < NPX; i++) {
#ifdef KOI_HOST_PROBE
        s_who[i] = (unsigned char)s_who_cur;
#endif
        uint16_t d = s_fb[i];
        int dr = s_e5[(d >> 11) & 31];
        int dg = s_e6[(d >> 5) & 63];
        int db = s_e5[d & 31];
        s_fb[i] = pack565(dr + (((r - dr) * a) >> 8),
                          dg + (((g - dg) * a) >> 8),
                          db + (((b - db) * a) >> 8));
    }
}

/* s_pts 里放的本来就是世界坐标 → 直接拷给 stroke_pts 要的 s_wf。
   ⚠️ 不能借用 pt_xf_f：那个套的是**当前荷叶**的旋转/平移（s_lrx/s_lca…），
      开局屏上它是上一次 lily_paint 留下的残值，会把卡片摆到别处去。 */
static void pts_to_wf(int n)
{
    for (int i = 0; i < n * 2; i++) s_wf[i] = s_pts[i];
}

/* 圆角矩形点列（世界坐标，直接进 s_pts）。四个角各 seg 段，顺时针，
   共 4*(seg+1) 个点 —— 首尾不重合，所以描边时要 closed=1 把它接上。
   ★ seg 必须 ≥ 10：stroke_line 把每段画成**方头四边形**，相邻段在顶点处
     方向一变就叠出一个楔形（最宽 ≈ 段长·sin(半转角)）。seg=6 时每段转 15°、
     段长 3.65px → 楔形宽近 1px，叠在 1px 描边上就是"圆角又粗又亮"：
     实测固件圆角 5~6px 厚，而网页版全程 1px（直边是轴对齐的所以没事）。
     seg=14 时段长 1.57px、每段转 6.4° → 楔形收窄到 0.17px，基本看不出来。
   ⚠️ 上限：4*(seg+1) 必须 ≤ NPTS(64)，即 **seg ≤ 15**。超了 pt_push 会
     **静默**截断 → 圆角少一段，屏上表现为"缺个角"，极难定位。 */
static void rr_pts(float x, float y, float w, float h, float r, int seg)
{
    static const float CXK[4] = {1.0f, 1.0f, 0.0f, 0.0f};   /* 右上 / 右下 / 左下 / 左上 */
    static const float CYK[4] = {0.0f, 1.0f, 1.0f, 0.0f};
    static const float A0K[4] = {-1.5707963f, 0.0f, 1.5707963f, 3.1415927f};
    s_npts = 0;
    for (int k = 0; k < 4; k++) {
        float ccx = x + r + CXK[k] * (w - 2.0f * r);
        float ccy = y + r + CYK[k] * (h - 2.0f * r);
        for (int i = 0; i <= seg; i++) {
            float a = A0K[k] + 1.5707963f * (float)i / (float)seg;
            pt_push(ccx + fcos_t(a) * r, ccy + fsin_t(a) * r);
        }
    }
}

/* 卡片上的鱼图标 —— 不是"另画一条鱼"，而是把**池子里那条鱼**在静止姿态下画一遍：
   与网页版 koiIcon() 同思路（它也是调 drawKoiBones 画到离屏画布）。
   好处是图标和池里的鱼永远是同一条：以后改鱼身造型、改尾鳍比例，图标自动跟着变。
   王总原话："鱼的图标用咱们的真鱼图标啊，怎么画了一个啊"（上一版是手搓的简笔鱼）。 */
static void setup_koi_icon(int pat, float cx, float cy2)
{
    /* 固定斑型：图标不能每次都不一样（网页版 ICO_SPOT） */
    static const float ICO_SPOT[3][5] = {
        {1.0f, 0.25f, 0.125f, 0.42f, -0.08f},   /* 两大一小，中间留白 —— */
        {2.0f, 0.60f, 0.060f, 0.26f, -0.16f},   /* 斑块连成带就成"红底白边"了，正好反 */
        {3.0f, 0.50f, 0.105f, 0.40f,  0.10f},
    };
    static koi_t ik;                             /* static：不占栈（koi_t 有 sp[4][5]） */
    const float L  = 32.0f;
    float Wd = L * 0.175f;
    /* 画布尺寸按"游姿伸出去的最远处"留（与网页版同式）：
       胸鳍会扇到 ±1.5 倍体半宽，尾楔往后 0.26L */
    float w = ceilf(L * 1.45f) + 8.0f;
    float h = ceilf(Wd * 4.2f) + 6.0f;

    memset(&ik, 0, sizeof(ik));
    ik.x = cx - w * 0.5f - L * 0.552f;           /* 0.552 让头尾构图在画布里居中 */
    ik.y = cy2 - h * 0.5f;
    ik.headA = KOI_PI;                           /* 头朝左（与网页版图标同向） */
    /* 姿态取"游到一半"的微 S 形，但把尾鳍横摆相位顶到 0：
       尾鳍的横向偏移是 sin(phase − KSEG×KPHASE − 0.85)，取这个相位正好归零 ——
       不归零的话，静止图标上尾鳍会和尾柄错开一道缝，看着像"尾鳍掉了"。 */
    ik.phase = (float)KSEG * KPHASE + 0.85f;
    ik.curv = 0.0f;
    ik.grow = 1.0f;
    ik.L = L;
    ik.waveAmp = 0.30f;
    ik.pat = (uint8_t)pat;
    ik.eat = 0.0f;
    if (pat != 1) {
        ik.ns = 3;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 5; j++) ik.sp[i][j] = ICO_SPOT[i][j];
    }
    koi_draw(&ik);
}

/* 第 2 屏：两张卡（红白 / 御黄金）。两个数字之和恒等于总条数 ——
   上键加红白就自动减御黄金，所以"分配不满"的非法状态在结构上不存在，
   既不需要校验也不需要提示。
   RR_SEG 为什么是 14 而不是 6：见 rr_pts 的注释（方头四边形在顶点叠楔形，
   6 段会让 1px 描边在圆角处胖到 5~6px）。14 是 NPTS 允许的最大档之一。 */
#define RR_SEG 14
static void setup_cards(void)
{
    static const uint8_t C_CARD[3] = {16, 74, 72};      /* rgba(16,74,72,0.60) */
    static const uint8_t C_ON[3]   = {186, 255, 230};   /* rgba(186,255,230,0.95) */
    static const uint8_t C_OFF[3]  = {170, 228, 214};   /* rgba(170,228,214,0.34) */
    const int cw = 108, chh = 114, cg2 = 12;
    int cy  = (int)((float)KH * 0.46f);
    int cx0 = (KW - (cw * 2 + cg2)) / 2;
    int cy0 = cy - chh / 2;

    for (int c = 0; c < 2; c++) {
        int num = c ? (s_pick_n - s_split_kh) : s_split_kh;
        int bx  = cx0 + c * (cw + cg2);
        int on  = (num > 0);
        int d   = on ? 0 : UI_DIM_OFF;                  /* 暗态位图的索引偏移 */

        /* 半透明底：字要压在亮水面上。别做成纯黑半透明 —— 那会让卡片读成
           "屏幕上挖了个洞"，用同色系的深青，保持"这片界面也是水的一部分"。 */
        rr_pts((float)bx, (float)cy0, (float)cw, (float)chh, 14.0f, RR_SEG);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, C_CARD, NULL, (int)(0.60f * 256.0f));

        /* 外沿 1px 描边：亮卡亮边、暗卡灰边。
           几何与网页版逐字对齐：`rr(ctx, bx+0.5, cy0+0.5, cw-1, chh-1, 14)`
           —— 内缩 0.5、**半径仍是 14**（不是 13.5；改小了圆角会瘦一圈）。 */
        rr_pts((float)bx + 0.5f, (float)cy0 + 0.5f,
               (float)cw - 1.0f, (float)chh - 1.0f, 14.0f, RR_SEG);
        pts_to_wf(s_npts);
        if (on) stroke_pts(s_wf, s_npts, 1, 1.0f, C_ON,  (int)(0.95f * 256.0f));
        else    stroke_pts(s_wf, s_npts, 1, 1.0f, C_OFF, (int)(0.34f * 256.0f));

        /* 图标 / 名字 / 数字三行居中 —— "御黄金"是三个字，横排必然要挤图标 */
        setup_koi_icon(c, (float)(bx + cw / 2), (float)(cy0 + 26));
        ui_draw(UI_CN_KH + c + d,     bx + cw / 2, cy0 + 52);
        ui_draw(UI_NUM0 + num + d,    bx + cw / 2, cy0 + 84);
    }
}

/* 开局屏整屏绘制。只在 s_full（开机 / 换屏那一帧）被调用 ——
   稳态下 render() 根本不进这里，脏区因此是 0。 */
static void setup_draw(void)
{
    set_clip(0, 0, KW - 1, KH - 1);

    /* 第 0 屏：整屏就是这一张设计稿，水面/波光/暗罩/文字全都不用画 ——
       画了也被盖住（固件里就是白烧一次 SPI）。 */
    if (s_setup_step == 0) { home_blit(); return; }

    /* 第 1/2 屏：开局和池塘是同一池水，不换皮肤 */
    water_rect(0, 0, KW - 1, KH - 1);

    /* 波光点：网页版逐帧呼吸（sin(time*1.7 + phase)），这里按固定相位烘一次 */
    for (int i = 0; i < 44; i++) {
        const spark_t *sp = &s_spark[i];
        int a = (int)((0.10f + 0.17f * (0.5f + 0.5f * fsin_t(sp->phase))) * 256.0f);
        if (a <= 0) continue;
        demo_koi_who(2, i);
        const uint8_t *cc = s_pal[PI_SPARK];
        for (int y = 0; y < sp->size; y++)
            for (int x = 0; x < sp->size; x++)
                px_blend((int)sp->x + x, (int)sp->y + y, cc[0], cc[1], cc[2], a);
    }

    /* 压一层暗罩：字要压在亮水面上，没有这层就飘。别压狠 ——
       水色本身是这场景最好看的部分，压重了整屏发闷。 */
    veil_full(8, 44, 46, (int)(0.17f * 256.0f));

    if (s_setup_step == 1) {
        nm_blit(s_pick_idx, KW / 2, (int)((float)KH * 0.46f));
        ui_draw(UI_TIP_PICK, KW / 2, KH - 38);
    } else {
        setup_cards();
        ui_draw(UI_TIP_SPLIT, KW / 2, KH - 38);
    }
}

static void render(void)
{
    PROF_START();
#ifdef KOI_HOST_PROBE
    /* ★ 归属图按"一次 render"为界清空：于是"本帧没有任何人写过这个像素"
       读出来就是"无"（= 0），正是判"增量 pass 有没有补上这一笔"要的那个值。 */
    demo_koi_who_reset();
#endif
    /* 开局三屏：**静态画面**。只有 s_full（开机 / 换屏那一帧）才真的画，
       其余帧一个像素都不动 —— 稳态脏区因此是 0，SPI 一点不占。
       （网页版这三屏每帧全量重画，固件照抄就是每帧 30.7ms SPI，见第 14 节头注释。） */
    if (s_scene == 0) {
        if (s_full) setup_draw();
        PROF_TICK(2);
        return;
    }
    if (s_full) {
        scene_draw(0, 0, KW - 1, KH - 1, -1);
        PROF_TICK(2);
        return;
    }
    /* 波带删掉后这里就干净了：一帧 = 逐个脏矩形画整栈，没有"另一层"要单独拼。
       （原先是"波带条只画水+波带、对象矩形再画整栈"—— 那套是为了压层序才存在的。）
       ★ 第 34 轮试过"给每条鱼/每片荷叶找独占矩形、只画一次"，实测**不成立**：
         scene_draw 每进一个矩形先 water_rect 把整块 clip 填成水，"在别的矩形里跳过"
         = 被水擦掉后不补回来 → 鱼的像素留在荷叶上。已撤，见 scene_draw 上面的注释。 */
    for (int i = 0; i < s_nrect; i++)
        scene_draw(s_rc[i].x0, s_rc[i].y0, s_rc[i].x1, s_rc[i].y1, -1);
    PROF_TICK(2);          // 2 = render 总（含上面所有 scene_draw）
}

/* ==========================================================================
   15. 定时器 / 生命周期
   ========================================================================== */
static lv_obj_t   *s_scr;
static lv_obj_t   *s_canvas;
static lv_timer_t *s_timer;
static int64_t     s_t_last, s_sum_us, s_max_us, s_dirty_acc;
static int64_t     s_koi_acc, s_lily_acc;          // 重复绘制计数的窗口累计
static int         s_frames, s_log_acc;

static void tick_cb(lv_timer_t *t)
{
    (void)t;
    int64_t t0 = esp_timer_get_time();
    PROF_START();

    build_palette(s_night);
    PROF_TICK(0);

    s_nrect = 0;
    s_full = 0;                 // 由 step() 决定（开机第一帧 / 昼夜过渡期 → 整屏）
    s_koi_n = 0; s_lily_n = 0;  // 本帧的重复绘制计数，从 0 起
    if (s_scene == 0) {
        /* 开局三屏：不跑物理，只有"换屏请求"那一帧整屏重画一次，之后零脏区 */
        build_palette(s_night);
        PROF_TICK(0);
        if (s_setup_dirty) { s_full = 1; s_setup_dirty = 0; }
    } else {
        step(1.0f / (float)FPS);
        PROF_TICK(1);
        /* ★★ 第 34 轮修正③：**palette 换档与整屏重画必须同帧**。
           原来 build_palette 挂在 step **之前**，于是"night 前进"与"调色板重建"
           差了一帧。夜过渡结束那一帧就露出来了：
             · 帧 N：build_palette(旧 night) → step 把 night 推到 target 并置 s_full=1
                     → 本帧整屏重画，用的却还是**上一档**调色板；
             · 帧 N+1：night 已等于 target（不再前进）→ s_full=0 走脏区；而
                     build_palette 这时才重建（水色整屏变暗）→ 只有那几个脏矩形
                     里的水被更新成新亮度，其余水面全停在旧亮度。
           台架金标准判据把它量出来了：第 886 帧 增量≠整屏 **44300 像素**，
           漏画点 (0,0)…(7,0) 且「盖着: （无对象）」—— 差异像素就是水面本身，
           不属于任何鱼/荷叶/饲料，所以按对象找永远找不到。
           修法：把 build_palette 挪到 step 之后，并且**只要它真的重建了就整屏重画**。
           于是不变量变成定义式的："调色板变了 ⇒ 整屏重画"，与 night 在不在动无关。
           ⚠️ 别再退回"在 step 里判断 night != target"那种按状态推的写法 —— 那就是
              这次漏掉一帧的原因（状态对了，但派生数据（调色板）晚了一帧才跟上）。 */
        if (build_palette(s_night)) s_full = 1;
        PROF_TICK(0);
    }
    render();                   // 内部自己 PROF_TICK(2)/(3..8)
    s_koi_acc += s_koi_n; s_lily_acc += s_lily_n;

    if (s_full) {
        lv_obj_invalidate(s_canvas);
    } else {
        for (int i = 0; i < s_nrect; i++) {
            lv_area_t a;
            a.x1 = (int32_t)s_rc[i].x0; a.y1 = (int32_t)s_rc[i].y0;
            a.x2 = (int32_t)s_rc[i].x1; a.y2 = (int32_t)s_rc[i].y1;
            lv_obj_invalidate_area(s_canvas, &a);
        }
    }

    int64_t us = esp_timer_get_time() - t0;
    int64_t dpx = 0;
    if (s_full) dpx = (int64_t)NPX;
    else {
        for (int i = 0; i < s_nrect; i++)
            dpx += (int64_t)(s_rc[i].x1 - s_rc[i].x0 + 1) * (s_rc[i].y1 - s_rc[i].y0 + 1);
    }

    s_frames++; s_sum_us += us; s_dirty_acc += dpx;
    if (us > s_max_us) s_max_us = us;
    if (++s_log_acc >= FPS * 2) {
        int64_t now = esp_timer_get_time();
        double n = (double)s_log_acc;
        ESP_LOGI(TAG, "%s 渲染 avg=%.2fms max=%.2fms 脏区=%.1f%% rect=%d 鱼=%d 料=%d 食=%d 波=%d 帧=%lld",
                 s_night > 0.5f ? "夜" : "昼",
                 (double)s_sum_us / 1000.0 / n,
                 (double)s_max_us / 1000.0,
                 100.0 * (double)s_dirty_acc / (n * (double)NPX),
                 s_nrect, s_nkoi, s_npel, s_nfood, s_nrip,
                 (long long)(s_frames * 1000000 / (now - s_t_last + 1)));
        /* ★ 临时诊断：每帧各段耗时(ms)。pal 调色板 / step 物理 / 渲染 render 总
           （含下面所有 scene_draw）；水/鱼/涟/荷/料 = scene_draw 内部，按调用次数累加。
           ★ 波带删掉后槽位 4 已废，不再打印。 */
        ESP_LOGI(TAG, "  剖 pal=%.2f step=%.2f 渲染=%.2f | 水=%.2f 鱼=%.2f 涟=%.2f 荷=%.2f 料=%.2f",
                 (double)s_prof[0] / 1000.0 / n, (double)s_prof[1] / 1000.0 / n,
                 (double)s_prof[2] / 1000.0 / n, (double)s_prof[3] / 1000.0 / n,
                 (double)s_prof[5] / 1000.0 / n, (double)s_prof[6] / 1000.0 / n,
                 (double)s_prof[7] / 1000.0 / n, (double)s_prof[8] / 1000.0 / n);
        /* 鱼的内部拆分（每帧 5 条鱼累加后的均值） */
        ESP_LOGI(TAG, "  鱼剖 spine=%.2f 影=%.2f 身=%.2f 斑=%.2f 光=%.2f 尾=%.2f 胸=%.2f",
                 (double)s_prof2[0] / 1000.0 / n, (double)s_prof2[1] / 1000.0 / n,
                 (double)s_prof2[2] / 1000.0 / n, (double)s_prof2[3] / 1000.0 / n,
                 (double)s_prof2[4] / 1000.0 / n, (double)s_prof2[5] / 1000.0 / n,
                 (double)s_prof2[6] / 1000.0 / n);
        /* ★ 重复绘制：每帧实际调了几次绘制（分母是"应该"的次数）。
           比 1 大多少，就是被 scene_draw 的多个矩形重复画了多少遍。 */
        ESP_LOGI(TAG, "  重复 鱼画=%.2f/%d 荷画=%.2f/%d",
                 (double)s_koi_acc / n, s_nkoi,
                 (double)s_lily_acc / n, s_nlily);
        s_log_acc = 0; s_sum_us = 0; s_max_us = 0; s_dirty_acc = 0;
        s_frames = 0; s_t_last = now;
        s_koi_acc = 0; s_lily_acc = 0;
        for (int i = 0; i < 10; i++) s_prof[i] = 0;
        for (int i = 0; i < 8; i++) s_prof2[i] = 0;
    }
}

void demo_koi_enter(void)
{
    s_scr = lv_obj_create(NULL);
    lv_obj_remove_flag(s_scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_scr, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_scr, 0, 0);
    lv_obj_set_style_pad_all(s_scr, 0, 0);

    s_canvas = lv_canvas_create(s_scr);
    lv_canvas_set_buffer(s_canvas, s_fb, KW, KH, LV_COLOR_FORMAT_NATIVE);
    lv_obj_set_pos(s_canvas, 0, 0);
    lv_screen_load(s_scr);

    sin_init();
    unitcircle_init();      /* ★ 第 36 轮：常量单位圆表（必须在 sin_init 之后） */
    expand_init();
    /* （dx2_init 第 34 轮删 —— 水面光晕没了的直接结果，不再需要预计算 dx² 表） */
    s_pal_night = -1;
    day_pal_sync();          /* ★ 37 轮：先把"观感档"灌进 s_day，再建调色板 */
    build_palette(0.0f);

    /* 开机从**首页**开始（网页版 scene='setup' / setupStep=0 / pickIdx=0）。
       池子在这时候先按默认档建好，正式开养时 start_pond() 会按玩家选的条数重建。 */
    s_scene       = 0;
    s_setup_step  = 0;
    s_pick_idx    = 0;
    s_pick_n      = PICK_N[0];              /* 独占鳌头 */
    s_split_kh    = default_split(s_pick_n);
    s_setup_dirty = 1;

    pond_init();
    s_nrip = 0; s_npel = 0; s_nfood = 0;
    s_time = 0; s_shakeT = 0;
    s_night = 0; s_nightTarget = 0; s_night_log = 0;   /* ★ 37 轮：逻辑值一起归零 */
    s_satiety = 0.5f;
    s_nrect = 0; s_full = 1; s_first_frame = 1;
    s_frames = 0; s_log_acc = 0; s_sum_us = 0; s_max_us = 0; s_dirty_acc = 0;
    s_t_last = esp_timer_get_time();

    ESP_LOGI(TAG, "锦鲤池进入：画布 %dx%d RGB565 = %u B，目标 %d fps；开机=首页，%d 档条数可选",
             KW, KH, (unsigned)sizeof(s_fb), FPS, N_PICK);

    s_timer = lv_timer_create(tick_cb, 1000 / FPS, NULL);
}

void demo_koi_exit(void)
{
    if (s_timer) { lv_timer_delete(s_timer); s_timer = NULL; }
    if (s_scr) { lv_obj_delete(s_scr); s_scr = NULL; s_canvas = NULL; }
}

/* 开局屏 → 进池子：定条数、按分色重建池子、请求整屏重画一次。 */
static void start_pond(void)
{
    s_scene = 1;
    s_nkoi  = s_pick_n;
    pond_init();
    s_nrip = 0; s_npel = 0; s_nfood = 0;
    s_time = 0; s_shakeT = 0;
    s_satiety = 0.55f;
    s_night = 0; s_nightTarget = 0; s_night_log = 0;   /* ★ 37 轮：逻辑值一起归零 */
    s_first_frame = 1;                       /* 进池第一帧整屏画一次 */
    s_full = 1;
    ESP_LOGI(TAG, "开养：%d 条（红白 %d / 御黄金 %d）",
             s_pick_n, s_split_kh, s_pick_n - s_split_kh);
}

/* 长按 OK：清空当前池子并回到首页。事件由 BSP 按键组件直接上送，
   不依赖 main.c 拦截，这样单页锦鲤固件也能可靠复位。 */
static void reset_to_home(void)
{
    s_scene = 0;
    s_setup_step = 0;
    s_pick_idx = 0;
    s_pick_n = PICK_N[0];
    s_split_kh = default_split(s_pick_n);
    s_nrip = s_npel = s_nfood = 0;
    s_night = s_nightTarget = 0.0f;
    s_night_log = 0.0f;                      /* ★ 37 轮：量化档也要跟着归零 */
    s_shakeT = 0.0f;
    s_first_frame = 1;
    s_full = 1;
    s_setup_dirty = 1;
}

/* 按键分派。逐条对齐网页版的 fire(name)：feed(↑) / tap(↓) / night(OK)。
   ★ OK 在开局屏是"确定"、在池内是"昼夜切换" —— 同一个物理键，两屏语义不同。
   ★ OK 长按直接清档回首页。 */
void demo_koi_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    if (btn == BSP_BTN_OK && ev == BSP_BTN_LONG) {
        reset_to_home();
        return;
    }
    if (ev != BSP_BTN_CLICK) return;

    if (s_scene == 0) {
        if (s_setup_step == 0) {
            /* 首页只有一颗「锦池初启」。↑↓ 在这屏没有活干 ——
               保持亮着、按了就是不动（把它涂灰会被读成"这个键坏了"，网页版栽过一次）。 */
            if (btn == BSP_BTN_OK) { s_setup_step = 1; s_setup_dirty = 1; }
            return;
        }
        if (s_setup_step == 1) {
            if (btn == BSP_BTN_UP)        s_pick_idx = (s_pick_idx - 1 + N_PICK) % N_PICK;
            else if (btn == BSP_BTN_DOWN) s_pick_idx = (s_pick_idx + 1) % N_PICK;
            else if (btn == BSP_BTN_OK) {
                s_pick_n   = PICK_N[s_pick_idx];
                s_split_kh = default_split(s_pick_n);
                /* 金玉成双（2 条）固定"一红白一黄金" —— 跳过分色直接开养 */
                if (s_pick_n == 2) { start_pond(); return; }
                s_setup_step = 2;
            }
            /* 翻档之后同步条数与默认分色（网页版在 fire 末尾无条件做这一次） */
            if (s_setup_step == 1) {
                s_pick_n   = PICK_N[s_pick_idx];
                s_split_kh = default_split(s_pick_n);
            }
            s_setup_dirty = 1;
            return;
        }
        /* 第 2 屏：↑ 给红白加一条、↓ 减一条，御黄金反向补齐 ——
           两者之和恒等于总数，所以"分配不满"这个状态在结构上就不存在，
           既不需要校验也不需要提示。想让御黄金多一条？把红白减一条就行，完全等价。 */
        {
            int m = s_pick_n + 1;
            if (btn == BSP_BTN_UP)        s_split_kh = (s_split_kh + 1) % m;
            else if (btn == BSP_BTN_DOWN) s_split_kh = (s_split_kh - 1 + m) % m;
            else if (btn == BSP_BTN_OK) { start_pond(); return; }
            s_setup_dirty = 1;
        }
        return;
    }

    /* ---- 池内 ---- */
    if (btn == BSP_BTN_UP)        do_feed();
    else if (btn == BSP_BTN_DOWN) do_tap();
    else if (btn == BSP_BTN_OK) {
        s_nightTarget = (s_nightTarget > 0.5f) ? 0.0f : 1.0f;
        s_night_log   = s_night;      /* ★ 37 轮：起跑点对齐，否则量化档下会先"跳"一下 */
    }
}

/* ==========================================================================
   H. 主机台架探针（**只在 -DKOI_HOST_PROBE 时编译**，固件里一行都不存在）
   --------------------------------------------------------------------------
   抓王总第 34 轮那句「经过荷叶下方会有不知道什么多余的杂颜色出来」。

   ★★ 第 34 轮最终结论：**"叶身实体逐帧恒定"这条软判据整体摘除了**。
   它的三次失败很有教育意义，按顺序是：
     ① 第一版假设"荷叶 = 实心圆盘"，可荷叶身本来有个 V 形缺口（lily_body_pts
        从叶缘切进来、顶点收在 0.34r）—— 鱼从叶子下面游过时会从缺口透出来，
        **这是对的**。判据把合法渲染判成 bug（同铁律 11）。
     ② 加进缺口（lily_solid_at）后仍报 ±1 LSB —— 因为**边界像素本来就是部分覆盖**，
        它按覆盖比例露出下面的水或鱼，也合法。于是又缩到"叶心区"（lily_core_at）。
     ③ 缩到叶心区后仍报 124/200 帧有变化。查下去发现：它为了绕开"荷叶自己跨
        bob/rot 档"这种**合法**变动，自己另存了一份"上次 bob/rot" ——
        与 step() 里的跨档门成了**两套并行状态**；两套状态一旦错位，基准就永久偏。
        铁证：它报出的差异点里，有相当一部分坐标（如荷叶#2 的 y=178/179）
        **根本不在任何上报矩形内** —— 没被重画的像素不可能自己变。
        **判据坏了，被读成"画面坏了"。**

   ⇒ 于是换成两条**不依赖任何手写几何模型**的判据，它们才是本轮的真正产出：
     · **脏区覆盖**：本帧真实变化的像素必须落在上报矩形内。
       若某帧把鱼色误画在叶面上、之后没人重画那块 → 那个像素在**污染发生的那一帧**
       就是"变化且不在矩形内" → 必被抓住。这条把③想验的事**完备**覆盖了。
     · **金标准（脏区渲染）**：增量合成出来的帧 ≡ 强制整屏重画出来的帧，逐像素相等。
       它把"重画一次该得到什么"当唯一真值，一次覆盖鱼的 AABB / 荷叶的量子门 /
       水面填充 / 层序 / 混合 —— 不依赖"这个像素应该是什么"的任何推测。

   保留的探针（仍在本文件末尾，仍只在 KOI_HOST_PROBE 下编译）：
     demo_koi_dump_state / demo_koi_dump_lily / pal_char / lily_solid_at
     —— 排障时把"池子精确状态"和"一块区域的像素分类字符图"直接打出来，
        别靠截图猜颜色（铁律 11 的老教训）。
   ========================================================================== */
#ifdef KOI_HOST_PROBE

/* ★★ 第 37 轮：观感档位的运行时入口（**只给台架用**）。
   为什么不让固件也带：固件里没有设置界面，一个"没人调"的 setter 就是死代码。
   台架用它把同一份固件的 N 个档各出一遍图，拼成「选择卡」给王总挑 ——
   挑中的那个档，下一轮直接改成 XXX_DEF 宏的默认值。
   ⚠️ 换档必须让调色板真的重建（day_pal_sync 里的 s_pal_night = -1 就是干这个），
      然后置 s_full = 1 整屏重画 —— 否则新档只在"恰好碰到的脏区"里生效，
      画面一半新一半旧（铁律 13 那一族：状态变了但没人重建）。
   ⚠️ night 档切换时 s_night_log 要跟着对齐，不然量化档会先蹦一下。 */
void demo_koi_set_look(int lily, int bgd, int spark, int tail, int night)
{
    s_look_lily  = lily;
    s_look_bgd   = bgd;
    s_look_spark = spark;
    s_look_tail  = tail;
    s_look_night = night;
    s_night_log  = s_night;
    day_pal_sync();
    build_palette(s_night);
    s_full = 1;
    s_first_frame = 1;
}

int demo_koi_get_look(int which)
{
    switch (which) {
    case 0:  return s_look_lily;
    case 1:  return s_look_bgd;
    case 2:  return s_look_spark;
    case 3:  return s_look_tail;
    default: return s_look_night;
    }
}

/* ★★ 第 39 轮：叶下鱼台架的"这一帧不画鱼 0"开关。
   只掐绘制、不碰模拟 —— 见 s_hide_koi0 的注释。 */
void demo_koi_hide_koi0(int on)
{
    s_hide_koi0 = on;
    s_full = 1;
}

    /* ★★ 第 39 轮「叶下鱼台架」—— 把鱼 0 摆到荷叶 0 的**正中心**，量"荷叶盖不盖得住鱼"。
       王总：「荷叶需要做成不能透视的，不能透看到下面的鱼」。

       ★ 判据怎么来的（换过两版，前两版都坏）：
         ✗ 第一版：`demo_koi_dump_lily` 的"认调色板原色"。帧缓冲是 **RGB565**，
           写下去就被量化，实测叶内 1102 px **全被判成 '?'**（连水面都不被认成纯色）
           —— 它认不出任何纯色，"没看到鱼色"根本不是证据（铁律 11）。
         ✗ 第二版：对照组把鱼 0 挪到屏外。差异包围盒 x[17..141] y[16..177] 远大于叶盘
           —— 因为**鱼 0 一挪走，其他鱼的同类避让行为全变了**，整池模拟都动了，
           读到的差异里绝大部分跟荷叶无关（判据被污染）。
         ✓ 第三版（现在这个）：**鱼 0 的位置/模拟一帧都不动**，只加一个"这一帧不画鱼 0"
           的开关。于是两组**唯一**的差别就是"鱼 0 画了没画"：
             差异 = 0  ⇔ 荷叶把鱼完全盖住了（画不画都一样）
             差异 = N  ⇔ 有 N 个像素的鱼从荷叶（或叶影/缺口）里露出来

       ⚠️ 与叠鱼台架同样两条纪律：
         ① 报脏要把新位置的保守框**并上旧框**（否则旧位置没人擦）；
         ② 只改位置与姿势，**不取随机数**（碰 rnd 会让下游全错位，铁律 8）。 */
void demo_koi_force_underlily(void)
{
    if (s_scene != 1 || s_nkoi < 1 || s_nlily < 1) return;
    const lily_t *L = &s_lily[0];
    float lx = L->x, ly = L->y + lily_bob(L);
    float ang = lily_rot(L);
    koi_t *k = &s_koi[0];
    float ox0 = k->ax0, oy0 = k->ay0, ox1 = k->ax1, oy1 = k->ay1;
    int   had = (k->ax1 >= k->ax0);

    /* 鱼朝 +x（headA 减去叶子的自转角 → 与世界 x 轴对齐，与叶瓣的相对角度可预测） */
    k->headA = -ang;
    k->grow  = 0.50f;            /* < 0.56 → 走**非 fine 档**（就是"半边鱼半透明"那条路） */
    k->L     = 62.0f;
    k->eat   = 0.0f;
    k->x     = lx;
    k->y     = ly;

    float r = k->L * k->grow * 0.85f + 12.0f;
    float nx0 = k->x - r, ny0 = k->y - r, nx1 = k->x + r, ny1 = k->y + r;
    if (had) {
        if (ox0 < nx0) nx0 = ox0;
        if (oy0 < ny0) ny0 = oy0;
        if (ox1 > nx1) nx1 = ox1;
        if (oy1 > ny1) ny1 = oy1;
    }
    k->ax0 = nx0; k->ay0 = ny0; k->ax1 = nx1; k->ay1 = ny1;
    k->bx0 = nx0; k->by0 = ny0; k->bx1 = nx1; k->by1 = ny1;
    s_full = 1;
}

/* ★★ 第 37 轮「叠鱼台架」—— 把前两条鱼摆到几乎同一处，专门看尾鳍会不会透出下面那条。
   为什么必须专门搭：真场景里两条鱼重叠的像素极少（实测 40_pond 一帧只有 **22px**），
   在整屏图上肉眼根本分不出两档的差别 —— 正是铁律 14 那条
   （"10px 的差别在真场景里看不出来，必须自己搭台架"）。
   ⚠️ 只在 KOI_HOST_PROBE 下编译，固件里一行都不存在。 */
void demo_koi_force_overlap(void)
{
    if (s_scene != 1 || s_nkoi < 2) return;
    /* 位置是量出来的：让**上面那条鱼的尾鳍**正好压在**下面那条的鱼身**上。
       鱼朝右（headA=0）→ 尾鳍在身后（左侧）→ 所以上面那条要摆在右边一点，
       它的尾巴才伸到下面那条的背上。
       ⚠️ grow 取 0.55 = **贴着 fine 阈值 0.56 的下沿**：
          再小一点鱼就太小、重叠面积不够；=0.56 就升到 fine 档（不透明）而看不到毛病了。 */
    static const float ox[2] = {110.0f, 121.0f};
    static const float oy[2] = {160.0f, 160.0f};
    for (int i = 0; i < 2; i++) {
        koi_t *k = &s_koi[i];
        /* ★ 先把"它原来在哪"记下来再搬 —— 报脏用的是**上帧真实 AABB**，
           直接把 AABB 清空的话，旧位置那一片就没人擦，会留一条鱼影
           （铁律 10 的同一族：位置变了要先分清"谁负责把旧的擦掉"）。
           正解：把新位置的保守框 **并上** 旧框，一次报脏把两头都盖住。 */
        float ox0 = k->ax0, oy0 = k->ay0, ox1 = k->ax1, oy1 = k->ay1;
        int   had = (k->ax1 >= k->ax0);

        k->x = ox[i]; k->y = oy[i];
        k->headA = 0.0f;                 /* 都朝右，姿势一致才好比 */
        k->grow  = 0.55f;                /* < 0.56 → 走非 fine 档（就是半透明那条路） */
        k->L     = 58.0f;                /* 两条一样大、都取大一点 —— 重叠面才够看得出来 */
        k->eat   = 0.0f;

        float r = k->L * k->grow * 0.85f + 12.0f;      /* 与 koi_bbox 的保守框同式 */
        float nx0 = k->x - r, ny0 = k->y - r, nx1 = k->x + r, ny1 = k->y + r;
        if (had) {
            if (ox0 < nx0) nx0 = ox0;
            if (oy0 < ny0) ny0 = oy0;
            if (ox1 > nx1) nx1 = ox1;
            if (oy1 > ny1) ny1 = oy1;
        }
        k->ax0 = nx0; k->ay0 = ny0; k->ax1 = nx1; k->ay1 = ny1;
        k->bx0 = nx0; k->by0 = ny0; k->bx1 = nx1; k->by1 = ny1;
    }
    s_full = 1;
}

/* 该像素是否落在荷叶的**实体**里：圆盘内、且不在 V 形缺口扇区里。
   缺口朝局部 +y（与 lily_body_pts 的顶点 (0, r*LILY_GAP_IN) 一致），半角 = L->gap。
   局部 = 世界绕 -th 转回来（lily_xf 是绕 +th 转出去）。 */
static int lily_solid_at(const lily_t *L, int px, int py)
{
    float dx = (float)px + 0.5f - L->x;
    float dy = (float)py + 0.5f - (L->y + lily_bob(L));
    float d2 = dx * dx + dy * dy;
    if (d2 > L->r * L->r) return 0;                    /* 圆盘外 */
    float ri = L->r * LILY_GAP_IN;
    if (d2 <= ri * ri) return 1;                       /* 缺口顶点以内 = 实心 */
    float la = atan2f(dy, dx) - lily_rot(L);
    return fabsf(angdiff(la, KOI_PI * 0.5f)) >= L->gap;
}

/* ★ 已删：lily_core_at / s_bad_lily / demo_koi_bad_lily / demo_koi_lily_diff /
   demo_koi_lily_crossed —— 它们只服务于"叶身实体逐帧恒定"那条软判据，而那条判据
   在第 34 轮被证明**自己就是坏的**（见文件头 H 节的最终结论），已整体摘除。
   lily_solid_at 保留：demo_koi_dump_lily 的字符图还在用它。 */

/* 不变量：span 覆盖率越界次数（必须恒为 0）。判据可自校验 —— 见 s_cov_over 的声明。 */
long demo_koi_cov_over(void) { return s_cov_over; }

/* 预测脏盒的最大欠缺（px）。≤0 = 盒子够大；>0 = 有残留，且这个数就是要补的量。 */
float demo_koi_bbox_short(void) { return s_bbox_short; }

/* 把一个像素分三类：
     '.'  = 正好等于该行纯净水色        （s_water_row[y]）
     '0'..'9','A'..'E' = 正好等于某个调色板项的原色（PI_* 的顺序，见上面 enum）
     '?'  = 两者都不是 → 混合色（边界像素 / 带 alpha 的笔画 / 水下投影）
   纯色判据可以自校验：一片荷叶的**内部**绝大多数像素必须是纯色（'A'），
   整片纯色区里冒出一串 '?' 就是混合算法坏了。 */
static char pal_char(uint16_t v, int y)
{
    if (v == s_water_row[y]) return '.';
    for (int i = 0; i < PI_NPAL; i++)
        if (v == pack565(s_pal[i][0], s_pal[i][1], s_pal[i][2]))
            return (char)(i < 10 ? ('0' + i) : ('A' + i - 10));
    return '?';
}

/* ==========================================================================
   ★★ 金标准判据：**增量合成的帧 ≡ 整屏重画的帧**（第 34 轮）
   --------------------------------------------------------------------------
   上面那两条"荷叶内部该不该变"的判据，本质都是我自己拿几何模型去推
   "这个像素应该是什么" —— 而模型一次次比现实简单（先是漏了 V 形缺口，
   又漏了"边界像素是部分覆盖"）。**判据越像手写模型，越容易把自己判错。**

   这条不靠模型，靠定义：脏区渲染器的正确性 = "只重画变化的那几块" 与
   "整屏重画" 必须得到**同一张图**。于是：
       ① 拿当前 s_fb（增量合成出来的）存一份；
       ② 强制 s_full=1 再 render() 一次（= 整屏重画）；
       ③ 逐像素相减。差一个像素就是脏区渲染错了。
   它一次能覆盖鱼的 AABB、荷叶的量子门、水面填充、层序、混合 —— 全部。
   ⚠️ 代价是每帧多一次整屏重画（主机 ~3ms，只在台架里跑，固件一行不编）。

   顺带：这条判据还能**自己证明自己没坏** —— 把 water_rect 的裁剪/覆盖率算法
   改回旧版（KOI_SRC 指向临时副本），它必须变红。见 TOOLS.md。
   ========================================================================== */
int demo_koi_full_redraw_diff(uint16_t *ref)
{
    /* ★ 把这次"额外整屏重画"对整个统计体系**完全隐藏**：
       否则重复绘制计数（鱼画/荷画）会翻倍、分段计时会多算一帧 ——
       等于探针自己把性能读数搞坏（这台架就是用来出那些数的）。
       掩掉三样：帧计数 / 分段计时 / PROF 时钟。 */
    int save_full = s_full, save_nrect = s_nrect;
    int save_koi_n = s_koi_n, save_lily_n = s_lily_n;
    int64_t sp[10], sp2[8];
    for (int i = 0; i < 10; i++) sp[i] = s_prof[i];
    for (int i = 0; i < 8; i++)  sp2[i] = s_prof2[i];
    int64_t spt = s_pt, spt2 = s_pt2;

    s_full = 1;
    render();                        /* 整屏重画到 s_fb */

    s_full = save_full;
    s_nrect = save_nrect;
    s_koi_n = save_koi_n; s_lily_n = save_lily_n;
    for (int i = 0; i < 10; i++) s_prof[i] = sp[i];
    for (int i = 0; i < 8; i++)  s_prof2[i] = sp2[i];
    s_pt = spt; s_pt2 = spt2;

    int bad = 0, fx = -1, fy = -1;
    for (int y = 0; y < KH; y++) {
        for (int x = 0; x < KW; x++) {
            uint16_t a = s_fb[y * KW + x], b = ref[y * KW + x];
            if (a == b) continue;
            if (fx < 0) { fx = x; fy = y; }
            if (bad < 8) {
                int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
                int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
                /* ★ 顺手报出"这个像素此刻被谁盖着" —— 光看颜色只能猜，
                   直接说"鱼2 / 荷1 / 料3 / 波0 / 无"就不用猜了。 */
                /* ★ 第 37 轮：把"谁画的"直接打出来（见文件头像素归属图的说明）。
                   读法：整屏=? 回答"这一笔本应是谁"；增量=? 回答"增量 pass 有没有补上这一笔"
                   （"无" = 本帧没有任何人写过它 → 那块的像素是旧的）。
                   ⚠️ 括号里的"盖着:"只是**包围盒**的猜测，与这两个字段冲突时以字段为准。 */
                printf("    增量/整屏不一致 (%3d,%3d) 增量(%3d,%3d,%3d)='%c' "
                       "整屏(%3d,%3d,%3d)='%c'  【整屏画笔=%s 增量画笔=%s】"
                       " [scene=%d full=%d nrect=%d] 盖着: ",
                       x, y, br * 255 / 31, bg * 255 / 63, bb * 255 / 31,
                       pal_char(b, y),
                       ar * 255 / 31, ag * 255 / 63, ab * 255 / 31, pal_char(a, y),
                       demo_koi_who_at(x, y), demo_koi_who_snap_at(x, y),
                       s_scene, save_full, save_nrect);
                int who = 0;
                for (int q = 0; q < s_nkoi; q++) {
                    float qx0, qy0, qx1, qy1;
                    koi_bbox(&s_koi[q], &qx0, &qy0, &qx1, &qy1);
                    if ((float)x >= qx0 - 1 && (float)x <= qx1 + 1 &&
                        (float)y >= qy0 - 1 && (float)y <= qy1 + 1) {
                        printf("鱼%d ", q); who++;
                    }
                }
                for (int q = 0; q < s_nlily; q++) {
                    const lily_t *L = &s_lily[q];
                    float ex = L->r + WEED_PAD + LILY_SLOP;
                    float ly = L->y + lily_bob(L);
                    if ((float)x >= L->x - ex && (float)x <= L->x + ex &&
                        (float)y >= ly - ex && (float)y <= ly + ex) { printf("荷%d ", q); who++; }
                }
                for (int q = 0; q < s_nrip; q++) {
                    float rr = s_rip[q].rMax + 8.0f;
                    if ((float)x >= s_rip[q].x - rr && (float)x <= s_rip[q].x + rr &&
                        (float)y >= s_rip[q].y - rr && (float)y <= s_rip[q].y + rr) {
                        printf("波%d ", q); who++;
                    }
                }
                for (int q = 0; q < s_npel; q++)
                    if (s_pel[q].delay <= 0.0f &&
                        fabsf(s_pel[q].x - (float)x) <= 4.0f &&
                        fabsf(s_pel[q].y - (float)y) <= 4.0f) { printf("料%d ", q); who++; }
                for (int q = 0; q < s_nfood; q++)
                    if (fabsf(s_food_x[q] - (float)x) <= 4.0f &&
                        fabsf(s_food_y[q] - (float)y) <= 4.0f) { printf("食%d ", q); who++; }
                /* 波光点：44 个固定位置，也是"只有进矩形才画"的 —— 全屏重画会补上它 */
                for (int q = 0; q < 44; q++)
                    if ((float)x >= s_spark[q].x && (float)x < s_spark[q].x + s_spark[q].size &&
                        (float)y >= s_spark[q].y && (float)y < s_spark[q].y + s_spark[q].size) {
                        printf("光%d ", q); who++;
                    }
                if (!who) printf("（无对象）");
                printf("\n");
            }
            bad++;
        }
    }
    /* ★★ 第 37 轮：**首帧不一致的现场放大**。
       归属探针能说"是荷3画的"，但**说不出它画成了什么形状** ——
       而"那一笔到底是叶身、浮萍还是描边弧度"正是分叉点（三者修法完全不同）。
       所以这里把两个缓冲区在同一个 48x19 小块上打成字符图：
       判据不再是"数值差多少"，而是"这一笔的形状长什么样"。
       ⚠️ 判"是不是纯净水"用的就是 s_water_row[yy]（每行一个常数）——
          水面是**逐行常量色**，所以任何"行内不一致"都直接意味着**有对象画在那儿**；
          反过来，"两行同色"也可能是量化平台的正常现象，别当成证据。 */
    static int g_map_once;
    if (fx >= 0 && g_map_once == 0) {
        g_map_once = 1;
        /* 窗口开得比"怀疑对象"大一圈：既要包住凸出来的那一笔，也要包住叶身本身，
           否则"这一笔在叶子的哪一侧、离叶心多远"就无从量起。 */
        int wx0 = fx - 42, wx1 = fx + 58, wy0 = fy - 16, wy1 = fy + 16;
        if (wx0 < 0) wx0 = 0;
        if (wy0 < 0) wy0 = 0;
        if (wx1 > KW - 1) wx1 = KW - 1;
        if (wy1 > KH - 1) wy1 = KH - 1;
        printf("  ── 首帧不一致现场 x[%d..%d] y[%d..%d]\n", wx0, wx1, wy0, wy1);
        printf("     ① 是不是水：'.'=该行纯净水  '#'=别的色        （左=增量  右=整屏）\n");
        for (int yy = wy0; yy <= wy1; yy++) {
            printf("   y%3d  增量 ", yy);
            for (int xx = wx0; xx <= wx1; xx++)
                putchar(ref[yy * KW + xx] == s_water_row[yy] ? '.' : '#');
            printf("   整屏 ");
            for (int xx = wx0; xx <= wx1; xx++)
                putchar(s_fb[yy * KW + xx] == s_water_row[yy] ? '.' : '#');
            printf("\n");
        }
        /* ② 画出"每一笔各自落在哪" —— 这才是能直接对上半径的那张图。
              ⚠️ 只画整屏 pass（增量 pass 里没被重画的像素读作空白，正好用来对照）。 */
        {
            const char *lab[26]; int nl = 0;
            for (int yy = wy0; yy <= wy1; yy++)
                for (int xx = wx0; xx <= wx1; xx++) {
                    const char *nm = demo_koi_who_at(xx, yy);
                    if (strcmp(nm, "无") == 0 || strcmp(nm, "水") == 0) continue;
                    int dup = 0;
                    for (int q = 0; q < nl; q++) if (strcmp(lab[q], nm) == 0) { dup = 1; break; }
                    if (!dup && nl < 26) lab[nl++] = nm;
                }
            printf("     ② 每一笔的落点（图例见下）　'.'=水　' '=没人画过（增量）\n");
            for (int yy = wy0; yy <= wy1; yy++) {
                printf("   y%3d  ", yy);
                for (int xx = wx0; xx <= wx1; xx++) {
                    const char *nm = demo_koi_who_at(xx, yy);
                    if (strcmp(nm, "无") == 0)      { putchar(' '); continue; }
                    if (strcmp(nm, "水") == 0)      { putchar('.'); continue; }
                    for (int q = 0; q < nl; q++)
                        if (strcmp(lab[q], nm) == 0) { putchar('A' + q); break; }
                }
                printf("\n");
            }
            printf("     图例：");
            for (int q = 0; q < nl; q++) printf(" %c=%s", 'A' + q, lab[q]);
            printf("\n");
        }
    }
    memcpy(ref, s_fb, (size_t)NPX * sizeof(uint16_t));
    return bad;
}

/* ★ 已删：demo_koi_lily_diff（"叶身实体逐帧恒定"软判据）。
   摘除理由见文件头 H 节。它留下的坑值得记一句：
   那条判据为了绕开"荷叶自己跨 bob/rot 档"这种合法变动，自己另存了一份"上次 bob/rot"
   —— **与 step() 里的跨档门成为两套并行状态**。两套状态一旦错位，基准就永久偏，
   于是它报出的差异里有相当一部分坐标**根本不在任何上报矩形内**
   （没被重画的像素不可能自己变）—— 判据坏了，被读成"画面坏了"。
   这正是铁律 11/12 的同一族：**判据自己不可靠时，它给出的数字比没有更坏。** */

/* 池子当前的精确状态（鱼 / 荷 / 脏矩形 / 覆盖率越界计数）。
   ★ 靠截图猜"这条竖道是什么颜色"太慢，直接把真值打出来（铁律 11）。 */
void demo_koi_dump_state(void)
{
    printf("    状态: 鱼=%d 荷=%d 料=%d 食=%d 波=%d rect=%d full=%d  覆盖率越界(cov>256)=%ld\n",
           s_nkoi, s_nlily, s_npel, s_nfood, s_nrip, s_nrect, s_full, s_cov_over);
    for (int i = 0; i < s_nkoi; i++) {
        const koi_t *k = &s_koi[i];
        printf("      鱼%d x=%6.2f y=%6.2f head=%6.3f L=%5.2f grow=%4.2f pat=%d "
               "aabb=[%.0f,%.0f .. %.0f,%.0f] 报脏盒=[%.0f,%.0f .. %.0f,%.0f]\n",
               i, k->x, k->y, k->headA, k->L, k->grow, (int)k->pat,
               k->ax0, k->ay0, k->ax1, k->ay1,
               k->bx0, k->by0, k->bx1, k->by1);
    }
    for (int i = 0; i < s_nlily; i++) {
        const lily_t *L = &s_lily[i];
        printf("      荷%d x=%.1f y=%.1f r=%.1f gap=%.2f bob=%.1f rot=%.3f\n",
               i, L->x, L->y, L->r, L->gap, lily_bob(L), lily_rot(L));
    }
    for (int i = 0; i < s_nrect; i++)
        printf("      rect#%d x[%d..%d] y[%d..%d]\n",
               i, s_rc[i].x0, s_rc[i].x1, s_rc[i].y0, s_rc[i].y1);
}

/* 把一块区域打成字符图。每像素分三类：
     '.'  = 正好等于该行纯净水色        （s_water_row[y]）
     '0'..'9','A'..'E' = 正好等于某个调色板项的原色（PI_* 的顺序，见上面 enum）
     '?'  = 两者都不是 → 混合色；**在一片荷叶内部，'?' 只该出现在叶脉/描边/缺口边**
            （那些是带 alpha 的笔画）。整片纯色区里冒出一串 '?' 就是混合算法坏了。 */
void demo_koi_dump_lily(int i)
{
    if (i < 0 || i >= s_nlily) return;
    const lily_t *L = &s_lily[i];
    int cx = (int)(L->x + 0.5f), cy = (int)(L->y + lily_bob(L) + 0.5f);
    int rr = (int)(L->r + 4.0f);
    int x0 = cx - rr < 0 ? 0 : cx - rr, x1 = cx + rr > KW - 1 ? KW - 1 : cx + rr;
    int y0 = cy - rr < 0 ? 0 : cy - rr, y1 = cy + rr > KH - 1 ? KH - 1 : cy + rr;
    int cnt[128]; long rich = 0, solid = 0;
    for (int k = 0; k < 128; k++) cnt[k] = 0;

    printf("    ── 荷叶#%d 字符图  x[%d..%d] y[%d..%d]  r=%.1f gap=%.2f rot=%.3f\n",
           i, x0, x1, y0, y1, L->r, L->gap, lily_rot(L));
    printf("      实体遮罩: '#'=叶面实体  '-'=缺口/圆盘外（不算数）\n");
    printf("           ");
    for (int x = x0; x <= x1; x++) printf("%c", (char)('0' + (x / 10) % 10));
    printf("\n           ");
    for (int x = x0; x <= x1; x++) printf("%c", (char)('0' + x % 10));
    printf("\n");
    for (int y = y0; y <= y1; y++) {
        printf("      %3d  ", y);
        for (int x = x0; x <= x1; x++)
            printf("%c", lily_solid_at(L, x, y) ? '#' : '-');
        printf("\n");
    }
    printf("      像素分类（. 水 / 调色板原色 / '?' 混合色）:\n");
    printf("           ");
    for (int x = x0; x <= x1; x++) printf("%c", (char)('0' + (x / 10) % 10));
    printf("\n");
    for (int y = y0; y <= y1; y++) {
        printf("      %3d  ", y);
        for (int x = x0; x <= x1; x++) {
            char c = pal_char(s_fb[y * KW + x], y);
            printf("%c", c);
            cnt[(int)(unsigned char)c] = cnt[(int)(unsigned char)c] + 1;
            if (c == '?' && lily_solid_at(L, x, y)) rich++;
            if (lily_solid_at(L, x, y)) solid++;
        }
        printf("\n");
    }
    printf("      统计: 叶内实体像素=%ld  其中 '?' 混合色=%ld (%.1f%%)   '.'=%d\n",
           solid, rich, solid ? 100.0 * (double)rich / (double)solid : 0.0, cnt['.']);
    printf("      各纯色计数:");
    for (int k = 0; k < PI_NPAL; k++) {
        char c = (char)(k < 10 ? ('0' + k) : ('A' + k - 10));
        if (cnt[(int)(unsigned char)c]) printf(" %c=%d", c, cnt[(int)(unsigned char)c]);
    }
    printf("\n      图例: 0=水顶 1=水底 2=波光 3=鱼身 4=鱼斑 5=鱼金 6=鳍 7=暗线 "
           "8=涟漪 9=饲料 A=叶面 B=叶缘/叶芯 C=叶脉 D=浮萍 E=浮萍浅\n");
}

/* ★★ 第 39 轮：荷叶#0 实体遮罩内的像素，**最后一笔**是谁画的？
   —— "荷叶透出鱼"只有两种可能，修法完全不同，所以必须先分清：
        · WHO 前缀是「鱼」 ⇒ 鱼在荷叶之后落笔（**层序错**：谁后画谁在上面）
        · WHO 是「荷…」    ⇒ 荷叶确实画了，那差异只能来自颜色 / alpha（**混合错**）
   用现成的分笔探针（kind 6=鱼、7=荷、10..16=荷影/荷身/荷脉/荷芯/荷斑/荷光/荷缺）——
   它挂在唯一的写像素出口上，比在 koi_draw / lily_paint 里插 printf 可信得多。 */
void demo_koi_who_lily0(void)
{
    if (s_nlily < 1) return;
    const lily_t *L = &s_lily[0];
    int cx = (int)(L->x + 0.5f), cy = (int)(L->y + lily_bob(L) + 0.5f);
    int rr = (int)(L->r + 6.0f);
    long solid = 0, nfish = 0, nwater = 0, none = 0, other = 0;
    long n_shadow = 0, n_body = 0, n_vein = 0, n_core = 0;
    long n_spot = 0, n_glow = 0, n_gap = 0, n_plain = 0;
    for (int y = cy - rr; y <= cy + rr; y++)
        for (int x = cx - rr; x <= cx + rr; x++) {
            if (x < 0 || x >= KW || y < 0 || y >= KH) continue;
            if (!lily_solid_at(L, x, y)) continue;
            solid++;
            const char *w = demo_koi_who_at(x, y);
            if      (!strncmp(w, "荷影", 6)) n_shadow++;
            else if (!strncmp(w, "荷身", 6)) n_body++;
            else if (!strncmp(w, "荷脉", 6)) n_vein++;
            else if (!strncmp(w, "荷芯", 6)) n_core++;
            else if (!strncmp(w, "荷斑", 6)) n_spot++;
            else if (!strncmp(w, "荷光", 6)) n_glow++;
            else if (!strncmp(w, "荷缺", 6)) n_gap++;
            else if (!strncmp(w, "荷",   3)) n_plain++;
            else if (!strncmp(w, "鱼",   3)) nfish++;
            else if (!strncmp(w, "水",   3)) nwater++;
            else if (!strncmp(w, "无",   3)) none++;
            else                             other++;
        }
    printf("  ★★ 荷叶#0 实体内 %ld px 的**最后一笔**分笔统计：\n", solid);
    printf("      荷身(α256)=%ld   荷影(α56)=%ld   荷脉=%ld   荷芯=%ld   荷斑=%ld   "
           "荷光(α33)=%ld   荷缺=%ld   荷(未细分)=%ld\n",
           n_body, n_shadow, n_vein, n_core, n_spot, n_glow, n_gap, n_plain);
    printf("      鱼=%ld  水=%ld  无=%ld  其他=%ld\n", nfish, nwater, none, other);
    printf("      ★ 判读：「荷身」若明显少于实体总数 ⇒ 叶身 α256 那一笔**没填满**，"
           "底下是半透明的影/光 ⇒ 鱼就从那儿透出来；\n");
    printf("              若「荷身」≈总数 ⇒ 叶身填满了，差异只能来自 α<256 的笔画（脉/光）叠在鱼上。\n");
}
#endif
