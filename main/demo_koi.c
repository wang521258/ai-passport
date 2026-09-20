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
#include "nvs_flash.h"
#include "nvs.h"
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
/* ★★ 第 44 轮：鱼池背景 = 王总给的水下光斑图（240x320 RGB565，153,600 B）。
   由 _tools/bgbake43.py 烘焙，**勿手改**。它是**静态底图**：
   进池子那一帧整屏铺一次，之后每次重画脏矩形 = 从这张图里把那一段 memcpy 回来
   —— 也就是"背景只画一次、之后只刷新鱼动过的那几块"。
   ⚠️ 别把它当"每帧重画整张"：water_rect/bg_rect 只在**脏矩形**里被调用。
   ⚠️ 它进 .rodata（flash），**不占 RAM** —— C3 只有 ~320KB DRAM，而 s_fb 已经吃掉 153,600 B
      （铁律 21）。所以"背景备份一份到 RAM"这条路是死的，必须从 flash 读。 */
#include "koi_bg.h"
/* ★★ 第 45 轮：**夜间**底图 = 王总给的平静水面（240x320 RGB565，153,600 B）。
   由 _tools/bgbake45.py 烘焙，**勿手改**。

   ■ 为什么要有第二张图
     王总原话：「给你的图片是我需要在昼夜替换的时候晚上换成给你的这个图片」——
     夜里要换成这张（没有光斑的平静水面），不是把白天那张调暗。

   ■ ★ 为什么这张图是**预先压暗**好的（这是本轮的核心修正）
     王总原话：「并且是当晚上时候整个屏幕暗 / 现在是按昼夜晚上的时候只是单独鱼和荷叶
               变暗了 这个不对的」。
     根因：第 44 轮把水面从"程序化纵向渐变（走调色板 PI_WTOP/PI_WBOT，夜里自动乘 k）"
           换成"照片（bg_rect 里直接 memcpy）"之后，**底图就绕过了调色板** ——
           build_palette 只改 s_pal[]，而 bg_rect 一个字节都不看 s_pal。
           于是夜里：鱼、荷叶、涟漪、波光都暗了，**水面没暗**。
     修法两条路，选了后者：
       (甲) bg_rect 逐像素乘 k —— 每像素 3 次拆位/乘/回装，整屏 76800 px 约 15~20ms，
            而且脏矩形每帧都要重付；在 160MHz 无 FPU 的 C3 上是白送的开销。
       (乙) 离线把 k 烘进图里 —— 运行时还是**一条 memcpy**，零额外开销（本方案）。
     ⚠️ 因此：**改固件的 NIGHT_DIM 就必须重跑 _tools/bgbake45.py**，否则背景与
        鱼/荷叶的亮度对不上（背景按旧 k 压的，鱼按新 k 压的）。
        烘焙脚本会把当时的 NIGHT_DIM/k 写进头文件注释里，便于对账。
     ⚠️ 压暗系数必须与 build_palette 里**同一个式子** `(v * k) >> 8`，
        k = 256 - (int)((1-NIGHT_DIM)*256+0.5)。别在这里"顺手"写成浮点乘。 */
#include "koi_bg_night.h"

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
    /* ★★ 第 53 轮：御黄金专用的一套**金属金层次**（王总给的五个色）。
       原来只有一个 PI_KGOLD 平涂，所以金鱼读起来是"一片黄纸"，没有金属感。
       ⚠️ 只给 pat==1（御黄金）用；红白那套（PI_KBODY/KSPOT）一个字节都不动。 */
    PI_KGOLD_DK,     /* 阴影   #B96812 = 185,104,18 —— 腹部 / 尾根压暗 */
    PI_KGOLD_LT,     /* 亮部   #FFD34E = 255,211,78 —— 头部略亮 */
    PI_KGOLD_HI,     /* 高光   #FFF0A0 = 255,240,160 —— 背部中央那条**很窄**的带 */
    PI_KSCALE,       /* 鳞片点 #FFF7D1 = 255,247,209 —— 背上几粒很小的点 */
    PI_KGOLD_TL,     /* 尾梢浅金 #FFE28C —— 尾鳍"金黄→半透明浅金"的**那一端** */
    /* ★★ 第 54 轮：红白鲤的**立体化**（王总：「不要像红白两色的平面纸片。
         先塑造鱼身体的体积，再绘制红色花纹」）
         白身从**背中央**向**两侧**四档：亮白 → 暖白 → 灰白 → 稍深阴影
         红斑三档：受光亮朱红 → 主体朱红 → 两侧/边缘深红
       ⚠️ 与御黄金那套**互不干扰**：这套只给 pat != 1 用。 */
    PI_KBODY_HI,     /* 亮白   #FFFDF5 = 255,253,245  背中央受光（非常窄的柔和高光） */
    PI_KBODY_SIDE,   /* 浅灰白 #D9DDD8 = 217,221,216  身体两侧 */
    PI_KBODY_DK,     /* 稍深灰 #BFC6C3 = 191,198,195  腹 / 尾根 / 鳍根阴影 */
    PI_KSPOT_LT,     /* 亮朱红 #F25A45 = 242, 90, 69  红斑受光处 */
    PI_KSPOT_DK,     /* 深红   #A92320 = 169, 35, 32  红斑靠两侧 / 边缘 */
    PI_NPAL
};

static const uint8_t DAY_PAL[PI_NPAL][3] = {
    { 44, 126, 112}, { 17,  70,  70}, {220, 254, 240},
    /* ★★ 第 47 轮：PI_KGOLD 由 {253,216,124}（米黄/柠檬黄）改成 {255,200,92}（橙金）。
       王总原话"黄金锦鲤要有金属金/橙金的层次感，而不是纯黄色"。
       原来那档 R≈G（253 vs 216）读出来就是"浅黄"，色相 ~48°；
       新值 R−G = 55，色相压到 ~40°，明显偏橙，配合背脊那条暗带（见 koi_draw ②段）
       就有了"背暗腹亮"的金属反射层次。
       ⚠️ 这一项只被金鱼身与尾鳍淡色表用（build_tail_pale），
          水面/涟漪/波光各有自己的槽位（PI_RIPPLE 等），不受影响。
       ⚠️ 改这一项会让所有历史"金鱼颜色"截图作废 —— 是有意的（王总要的就是改它）。 */
    /* ★★ 第 54 轮：白身主色 {250,246,238} → **暖白 #F4F1E8 = 244,241,232**
       （王总："白色身体不要使用纯白色整块填充。身体主色使用暖白 #F4F1E8"）。
       旧值 R>G>B 差得少（250/246/238）读出来偏"冷白/纸白"；暖白的 B 再降 6，
       色温往米黄走一点点，才有"活鱼的体色"而不是"复印纸"。
       ⚠️ PI_KBODY 还被 build_tail_pale 用作尾鳍淡色的基色 ⇒ 尾鳍跟着变暖，是有意的。
       ★★ 红斑主色 {232,90,38} → **锦鲤朱红 #E5392D = 229,57,45**
       （王总："红斑主体使用锦鲤朱红 #E5392D"）。旧值偏橙（G=90），新值 G=57 更抳红、
       B 38→45 让它不那么"火"，是锦鲤品种里标准的"绯"色。 */
    {244, 241, 232}, {229,  57,  45}, {245, 166,  35}, {252, 204, 176},
    {158,  78,  38}, {232, 255, 248}, {246, 208, 138},
    /* 荷叶 / 浮萍（网页版 DAY.lilyFill / lilyEdge / lilyVein / weed / weedPale） */
    { 52, 156,  88}, { 22,  94,  54}, {104, 210, 140}, {116, 192,  96},
    {164, 226, 118},
    /* ★★ 第 53 轮：御黄金的金属金层次（王总指定色值，一个都不要改）
         PI_KGOLD    #F5A623 = 245,166,35   主色（原橙金 255,200,92 作废）
         PI_KGOLD_DK #B96812 = 185,104,18   阴影
         PI_KGOLD_LT #FFD34E = 255,211,78   亮部
         PI_KGOLD_HI #FFF0A0 = 255,240,160  高光
         PI_KSCALE   #FFF7D1 = 255,247,209  鳞片点
         PI_KGOLD_TL #FFE28C = 255,226,140  尾梢浅金（**派生色**：介于 LT 与 HI 之间，
                                            只用于尾鳍末端向"半透明浅金"过渡的那一层）
       ⚠️ 主色比原橙金**暗且更橙**（R−G = 79，色相 ~36°），这是"金属金"的读感来源；
          五档之间跨度要够大，否则叠出来的层次会被 RGB565 的量化吃掉。 */
    {185, 104,  18}, {255, 211,  78}, {255, 240, 160}, {255, 247, 209},
    {255, 226, 140},
    /* ★★ 第 54 轮：红白鲤立体化的五个色（王总指定，一个都不要改）
         PI_KBODY_HI   #FFFDF5 = 255,253,245  背中央受光·亮白
         PI_KBODY_SIDE #D9DDD8 = 217,221,216  身体两侧·浅灰白
         PI_KBODY_DK   #BFC6C3 = 191,198,195  腹 / 尾根 / 鳍根·稍深灰
         PI_KSPOT_LT   #F25A45 = 242, 90, 69  红斑受光处·较亮朱红
         PI_KSPOT_DK   #A92320 = 169, 35, 32  红斑靠两侧 / 边缘·深红
       ⚠️ 三档白的**跨度只有 64 级**（255→191），RGB565 的 R 只有 32 级
          （= 8 级/档）—— 这就是为什么要靠**渐变**而不是再分层：
          硬分层在小鱼身上会变成一圈圈的色带，渐变才是"圆鼓鼓"。 */
    {255, 253, 245}, {217, 221, 216}, {191, 198, 195},
    {242,  90,  69}, {169,  35,  32},
};

/* 夜间亮度 = 白天的 42%。挑这个数不是拍的：再低鱼红就开始并档（232,90,38 乘到
   0.30 只剩 70,27,11，与体缘暗线 158,78,38 的暗版撞车，红白鲤会糊成一条褐鱼）。 */
#define NIGHT_DIM  0.42f
/* night=1 时要减掉的量（0..255）。拆成常量是为了让 build_palette 里只剩一次整数乘。 */
#define NIGHT_DROP ((int)((1.0f - NIGHT_DIM) * 256.0f + 0.5f))

static uint8_t s_pal[PI_NPAL][3];
static int     s_pal_night = -1;    // 已建立对应的 night×255；-1 = 还没建过
static int     s_pal_ver   = 0;     // ★ 52 轮：调色板版本号（荷叶快照的失效开关）

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
#define LOOK_BGD_DEF    2    /* ★ 第 43 轮：3→2（王总要"水池背景像平面"—— 档 3 ΔG=0 太纯，加点 subtle 纵深；
                                                选档 2 而非档 1 是因为 ΔG=16（6 条暗带）当年就被说"有点像草坪割草纹"。
                                                ΔG=10（≈5 级台阶 / 每 64 行一跳）应远低于"足球场"阈值。 */
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
    /* 档 2 极弱   ΔG= 8 →  7 级 / 每 40 行；肉眼基本看不出带
       ★ 第 43 轮调色：原本 (33,100,93)→(27,92,87) 太青、太暗 —— 王总要"水池颜色暗 也不绿"。
       新配色：上 (42, 118, 76) 下 (34, 108, 68)，更绿更亮，ΔG=10 / ΔB=8，subtle 纵深感。
       比例上 B/G ≈ 0.64（r40 是 0.89，确实"绿"了），ΔG=10 几乎看不出带。 */
    {{ 42, 118,  76}, { 34, 108,  68}},
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
   它就是 KOI_DIRT_MARGIN 的取值依据 —— 换鱼速 / 转角 / 体长参数后重跑，读数会变。
   ★★ 第 46 轮：光有一个最大值不够用 —— 真机帧长是**变的**（105ms 均值 / 142ms 峰），
      而"缺多少"随帧长怎么变是**量出来的**（扫 KOI_FRAMEMS 得：
      42→1.75 / 105→3.39 / 120→8.00 / 142→10.00，**105~120 之间有个台阶**）。
      台阶说明不只是"形变随 dt 变大"，还有事件型的东西在推。
      所以下面把最大值**发生在哪一帧、哪条鱼、哪一侧**也记下来（只给台架）。 */
static float s_bbox_short = -1e9f;
static int   s_bs_step = -1, s_bs_koi = -1, s_bs_side = -1;
static float s_bs_raw = 0.0f, s_bs_grow = 0.0f;
/* ★ 46 轮 · 活泼度探针累加器（见报脏循环里的注释）。台架最后取平均。 */
static double s_lv_spd, s_lv_turn, s_lv_curv, s_lv_burst, s_lv_n;
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

/* ★★ 第 52 轮：绘制目标可重定向（荷叶 RAM 快照要用）。
   真机一帧要重画 ~6 片荷叶（真机 60ms/帧，比鱼的 40ms 还贵），而荷叶的**样子**
   每秒才变一次：自转角 `L->rot + t·spin`（spin ≤0.07 rad/s）被量化到 0.05 rad
   ⇒ 0.7~1.4 秒才跳一档。也就是说：**60ms 的成本是在反复重画一张 1Hz 才变的图**。
   ⇒ 缓存成一张小图（预乘色 + 覆盖率），之后每帧只做一次贴图。

   ⚠️ 为什么改 px_blend 而不是另写一个"快照专用"像素函数：
      光栅化**只有一个出口**（px_set 已无调用点，是死代码），改一处就够；
      复制一份会留下两份永远对不齐的实现（铁律 6：耦合的量只留一个入口）。
      正常绘制时 s_tgt = s_fb / s_stride = KW ⇒ 与改动前逐位相同。 */
static uint16_t *s_tgt     = s_fb;
static int       s_stride  = KW;
static uint8_t  *s_asnap;                 /* 非 NULL = 快照模式 */
static int       s_aw;                    /* 快照模式下 alpha 的行宽（= 缓冲宽） */

/* alpha 0..256；a=256 时精确等于 src */
static inline void px_blend(int x, int y, int r, int g, int b, int a)
{
    if (a <= 0 || x < s_cx0 || x > s_cx1 || y < s_cy0 || y > s_cy1) return;
    PCNT(px);                            /* ★ 第 35 轮：真正写进去的像素数 */
    WHO_PUT(x, y);                       /* ★ 第 37 轮：像素归属探针（仅台架） */
    /* ★ 快照模式：顺手把**总覆盖率** A 累加出来。
       px_blend 的式子 d+(src−d)·a 展开就是 d·(1−a)+src·a —— dst 初始为 0 时
       得到的正是**预乘色** P = Σ src·a·Π(1−a)，所以"颜色"这一半不用另加逻辑；
       只差一个 A = 1−Π(1−a)，这里补上（8bit，精确，不靠反推）。
       ⇒ 贴图时 out = P + 水色·(1−A)，与直接画出来的结果**逐位相同**。 */
    if (s_asnap) {
        /* ⚠️⚠️ a **可以 > 256**：a = (a_sub·cov)>>8，而抗锯齿的 cov 在边的端点
           处会溢出 256（代码里那个 s_cov_over 计数器就是盯它的）。
           (256-a) 一变负 ⇒ 255 - ((255-A)·负数 >> 8) 会**往上溢出 uint8**，
           A 从 255 直接塌到几十 —— 叶子整体变成"半透明"，与直接画差出上百个色阶。
           ★ 第 52 轮排障记：A/B 对拍最大通道差 140，差异正好 6 个团块（= 6 片叶子）。
           ⇒ 累加前必须把 a 夹到 [0,256]。颜色那一路 (dr += ((r-dr)*a)>>8) 的溢出
             是**改动前就有**的行为，两条路径一致，所以不用动。 */
        int ac = a;
        if (ac > 256) ac = 256;
        int i = (y - s_cy0) * s_aw + (x - s_cx0);
        int A = s_asnap[i];
        s_asnap[i] = (uint8_t)(255 - (((255 - A) * (256 - ac)) >> 8));
    }
    uint16_t d = s_tgt[y * s_stride + x];
    int dr = s_e5[(d >> 11) & 31];
    int dg = s_e6[(d >> 5) & 63];
    int db = s_e5[d & 31];
    dr += ((r - dr) * a) >> 8;
    dg += ((g - dg) * a) >> 8;
    db += ((b - db) * a) >> 8;
    s_tgt[y * s_stride + x] = pack565(dr, dg, db);
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

/* ★★ 第 44 轮：背景源开关。1 = 王总给的照片（koi_bg.h，从 flash 按行拷）；
   0 = 程序化纵向渐变（第 34~43 轮那套，靠 LOOK_BGD 档位调色）。
   默认 1 —— 王总这次的原话是「你把图片做成鱼池背景」。
   ⚠️ 留 0 这条回退路径不是"死代码"：它是**逐字节对拍**用的基线 ——
      台架把 KOI_BGPHOTO=0 跑一遍，能证明"照片这条路只改了底图、没碰别的绘制"。
      这与第 37 轮"档 0 = 改动前逐字节等价"是同一套做法（铁律 14）。 */
static int s_bg_photo = 1;

/* ★★ 第 45 轮：当前**已经铺进 s_fb** 的底图是哪一份。
   0 = 白天 koi_bg，1 = 夜间 koi_bg_night，-1 = 还没铺过（第一帧必然整屏）。
   --------------------------------------------------------------------------
   为什么单独记一个变量，而不是渲染时现算 `s_night > 0.5f`：
     ① s_night 会在**帧外**被按键回调直接改（第 43 轮"OK 键昼夜瞬切"就是
        `s_night = s_nightTarget`，一步到位）。底图该不该换，取决于
        "**上一帧铺的是哪份**"，而不是"s_night 现在是多少"—— 前者才是能比较的实体。
     ② 换底图那一帧**必须整屏重画**，否则会出现"上半屏白天图、下半屏夜间图"
        （脏矩形只会把鱼动过的那几块换成新底图）。这是铁律 13 那一族：
        状态变了但没人重建。
   所以用法固定为：渲染前比对一次 → 不一致就 s_full = 1 并同步过去；
   绝不在别处零散地 `if (s_night > 0.5f) s_full = 1;`（漏一处就是半屏混色）。 */
static int s_bg_drawn = -1;

/* 底图该用哪一份 —— **唯一**的判定口。bg_rect 与"是否要整屏"都走它，
   免得两处各写一遍 `s_night > 0.5f` 然后慢慢跑偏（铁律 8 那一族）。 */
static int bg_want_night(void) { return (s_night > 0.5f) ? 1 : 0; }

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
    s_pal_ver++;        /* ★ 52 轮：调色板一变，荷叶快照必须失效（叶子颜色也在表里） */
    /* 整屏压暗：k = 256（白天）→ 256-NIGHT_DROP（夜里）。
       ★ 必须写成 `(v * k) >> 8` 而**不是** `v * NIGHT_DIM`：
         k == 256 时右移 8 位恒等于原值（IEEE754 下 0 误差），
         白天那一档与旧版逐字节相同 → 历史对照图不作废。
         ⚠️ 这条是第 34 轮特意保的：改配色公式最容易顺手把"白天"也改动 1 个色阶，
            然后就再也对不上之前所有截图了。 */
    int k = 256 - (NIGHT_DROP * n8) / 255;
    s_pal_night = n8;
    for (int i = 0; i < PI_NPAL; i++)
        for (int c = 0; c < 3; c++)
            s_pal[i][c] = (uint8_t)(((int)s_day[i][c] * k) >> 8);
    water_row_build();
    build_tail_pale();                        /* 尾鳍淡色跟着调色板走 */
    return 1;
}

/* 水面（含常驻波光点）填一个矩形 —— ★★ 第 44 轮起这里是**分发口**：
   池子背景改成王总给的照片之后，"填水"有两种实现，靠 s_bg_photo 选：
     · 1 = 照片背景（bg_rect，从 flash 按行 memcpy）—— 第 44 轮起的默认
     · 0 = 程序化纵向渐变（water_rect_proc）—— 回退路径，也是 LOOK_BGD 档位还在的意义
   两个实现都自带 spark_overlay，所以**调用点完全不用知道用的是哪个**。
   ⚠️ 别把分发写进调用点（scene_draw / 开局屏各写一次 if）—— 两处早晚跑偏，
      而且"漏改一处"的表现是"某一屏还是老水色"，很难定位。 */
static void spark_overlay(int x0, int y0, int x1, int y1);   /* 第 44 轮抽出的公共叠加层 */
/* ★ 第 45 轮：安全区多边形推回。定义在 12a 节（第 12 节之前），但出生点
   pond_init() 要用它，所以在这里先声明一次。 */
static int  swim_push(float *px, float *py, float m);
static KOI_HOT void bg_rect(int x0, int y0, int x1, int y1);
static KOI_HOT void water_rect_proc(int x0, int y0, int x1, int y1);

static KOI_HOT void water_rect(int x0, int y0, int x1, int y1)
{
    if (s_bg_photo) { bg_rect(x0, y0, x1, y1); return; }
    water_rect_proc(x0, y0, x1, y1);
}

/* 程序化水面：每行一个颜色（纵向渐变），第 34 轮起就没有 LUT 了。 */
static KOI_HOT void water_rect_proc(int x0, int y0, int x1, int y1)
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
    spark_overlay(x0, y0, x1, y1);
}

/* ★★ 第 44 轮：照片背景填一个矩形 —— 从 flash 里把这一段的像素**按行拷回来**。
   这就是"背景只绘制一次"的全部机制：进池子那一帧整屏铺一遍（s_full），
   之后每次只重画脏矩形，脏矩形里先用这一行把底图恢复，再把鱼/荷叶画上去。
   ⚠️ 用 memcpy 按行拷而不是逐像素：s_fb 与 koi_bg 都是行主序连续，
      x0..x1 在这一行里也是连续的，一行就是一次 memcpy —— 比逐像素少 240 倍的循环开销。
   ⚠️ koi_bg 在 flash（memory-mapped，过 cache），不在 RAM：C3 的 DRAM 装不下第二份 153,600 B
      （铁律 21）。所以这里是"从 flash 读"，不是"从 RAM 备份读"。
   ⚠️ 别把这段搬进"每帧整屏重画"：它只在**脏矩形**里被调用，面积通常 15%~35%。

   ★★ 第 45 轮：这里多了一个"选哪份底图"的分支 —— 白天 koi_bg / 夜间 koi_bg_night。
   两份都是**同一尺寸同一行主序**的 RGB565，所以除了源指针，后面的 memcpy 一模一样；
   夜间那份的压暗已经烘进图里（见 koi_bg_night.h 的说明），运行时**零额外开销**。
   ⚠️ 选源只走 bg_want_night()，别在这儿内联 `s_night > 0.5f` —— 整屏判定要用同一个口，
      两边写两遍迟早对不上（表现就是"换夜那一帧只有脏矩形换了底图"）。 */
static KOI_HOT void bg_rect(int x0, int y0, int x1, int y1)
{
    demo_koi_who(1, -1);                          /* 归属探针：这一片是"背景" */
    const uint16_t *bg = bg_want_night() ? koi_bg_night : koi_bg;
    int n = (x1 - x0 + 1) * 2;
    for (int y = y0; y <= y1; y++) {
        memcpy(&s_fb[y * KW + x0], &bg[y * KW + x0], (size_t)n);
#ifdef KOI_HOST_PROBE
        for (int x = x0; x <= x1; x++) WHO_PUT(x, y);
#endif
    }
    spark_overlay(x0, y0, x1, y1);
}

/* 波光点（44 个，按矩形过滤，代价可忽略）。
   ★ 第 44 轮抽成独立函数：照片背景那条路径也要在同一层叠波光 ——
   复制一份必然两边慢慢跑偏（铁律 8 那一族：draw 与 dump 必须共用一函数）。 */
static void spark_overlay(int x0, int y0, int x1, int y1)
{
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

/* ★★ 第 44 轮：背景源开关在这里（真正那份声明在"6. 水面"节首 —— water_rect
   要用它做分发，所以必须在水面节之前声明）。这里留个索引注释，免得下次在
   "脏区"这节里找它。 */

/* ==========================================================================
   ★★ 第 47 轮：合并阈值重新定档（两个数字都提成宏，好扫档）
   --------------------------------------------------------------------------
   第 34 轮把膨胀阈值从 15% 放宽到 45%，当时的账是：
     "多并一点 → 脏区面积 ↑（多填的水几乎免费）→ 矩形个数 ↓ →
      鱼/荷叶被重复重画的次数 ↓（那才是真机 216ms/frame 的主项）"
   那笔账在当年成立，因为当时水面每像素要算 dx²+dy² 再查两次表 *之外*，
   还有一条 112ms/帧的**波带**横条（第 46 轮已整段删除）。

   第 46 轮把「时间膨胀」修好之后，天平翻了回来，真机读数变了：
     v10 昼 avg=132.96ms  max=162.34ms  脏区=46.7%  **rect=1**
     剖 pal=0.03 step=6.98 | 水=5.64 鱼=53.01 涟=0.02 **荷=66.88** 料=0.04
     重复 鱼画=5.00/6 荷画=4.96/6
   ★ 「rect=1」是这份读数里最刺眼的一个：6 条鱼 + 6 片荷叶的脏框被**级联**
     粘成了一个覆盖 46.7% 屏幕的大包围盒。于是每一帧：
       · 水 memcpy 35866px（本来 6 个小框合计只要 ~4000px）；
       · 那个大框必然命中 5~6 片荷叶 ⇒ **荷=66.88ms 成了最大项**（画一片荷叶
         要比 memcpy 一行水贵两个数量级：叶芯 + 老叶斑 + 叶缘弧 + 伴生浮萍）。
   ★ 也就是说：**第 34 轮要省的那笔"重复重画"，在 rect=1 时已经降到 1 遍/条
     （鱼画=5.00/6、荷画=4.96/6 都 < 1），省无可省**；而它付的代价（大包围盒）
     现在全落在「水 memcpy + 荷叶重画」上。天平翻了，阈值必须跟着往回收。
   收多少 —— 用 `_tools/mergeladder47.py` 扫档决定，结论写在本文件末尾 H 节。 */
#ifndef KOI_MERGE_GROW
#define KOI_MERGE_GROW  145       /* 并集膨胀 ≤ 45% 就并（第 34 轮值，第 47 轮重定） */
#endif
#ifndef KOI_MERGE_OVLP
#define KOI_MERGE_OVLP   40       /* 重叠面积 ≥ 较小者的 40% 就并 */
#endif

/* 合并值不值？ —— 判据一：**并集面积相对"两块各自面积之和"的膨胀率**。
   合并本身是必要的：LVGL 每帧的失效区个数有上限（LV_INV_BUF_SIZE），
   而且小矩形（饲料 8x8、涟漪）本来就该并起来。
   但"碰一下就并"会把散落的鱼 / 荷叶一路粘成一个包围盒 ——
   实测夜态 4 个框粘成 1 个 202x257（= 全屏 68% 脏区），而真正变化的像素只有 6%。
   所以只在「几乎不膨胀」或「本来就大面积重叠」时才并。
   ★ 注意级联合并：A∪B 变大之后，再去并 C 时判据是拿**新框**算的，
     于是"每步只膨胀 45%"可以一路滚成一个覆盖半屏的框 —— 第 47 轮的真因。 */
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
    if (su * 100 <= (sa + sb) * KOI_MERGE_GROW) return 1;   /* 膨胀够小 → 值得并 */

    int ox0 = s_rc[i].x0 > x0 ? s_rc[i].x0 : x0;
    int oy0 = s_rc[i].y0 > y0 ? s_rc[i].y0 : y0;
    int ox1 = s_rc[i].x1 < x1 ? s_rc[i].x1 : x1;
    int oy1 = s_rc[i].y1 < y1 ? s_rc[i].y1 : y1;
    if (ox1 >= ox0 && oy1 >= oy0) {
        long sm = sa < sb ? sa : sb;
        long ov = (long)(ox1 - ox0 + 1) * (oy1 - oy0 + 1);
        if (ov * 100 >= sm * KOI_MERGE_OVLP) return 1;  /* 重叠够多 → 值得并 */
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

/* ★ 52 轮：画叶子时的落点偏移。正常绘制恒为 0（逐位不变）；
   只有建快照时临时设成"把叶子挪到小缓冲中央"的量。 */
static float s_ofx = 0.0f, s_ofy = 0.0f;

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
    /* ★ 52 轮：s_ofx/s_ofy 是**快照模式的落点偏移**（正常绘制恒为 0）。
       建快照时把叶子画到小缓冲的中央，而不是它在池里的真实位置。 */
    s_lrx = L->x - s_ofx; s_lry = L->y + lily_bob(L) - s_ofy;
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

/* ==========================================================================
   7b-2. ★★ 第 52 轮：荷叶 RAM 快照
   --------------------------------------------------------------------------
   【账先算清楚】真机 v17 读数：荷 **60ms** / 鱼 40ms / 水 3.6ms —— 荷叶是单项最大。
   而荷叶每秒被重画 ~42 次（7fps × 6 片），它的**样子**却只有 ~1Hz 才变：
      自转角 = L->rot + t·spin（spin ∈ ±0.07 rad/s），量化步长 LILY_ROT_Q = 0.05
      ⇒ 最快 0.05/0.07 ≈ 0.7s 跳一档，最慢 2.9s。
   也就是说：**每帧的 60ms 都在重画一张 1Hz 才变的图** ⇒ 缓存它。

   【存什么】预乘色 P（RGB565）+ 覆盖率 A（A8），3 B/px。
     · P：px_blend 的式子 d+(src−d)·a = d·(1−a)+src·a，dst 初始为 0 时
       累加出来的正是预乘色；
     · A：在 px_blend 里顺手累加 A = 1−Π(1−a)，8bit 精确（不靠黑白两遍反推）。
     ⇒ 贴图 out = P + 水色·(1−A)，与直接画出来的结果**逐位相同**。

   【为什么不整片叶一次 memcpy】叶子是**半透明**的（根影 0.22 / 叶脉 0.60 /
     叶芯 0.42 / 反光 0.13），底下是水面渐变 + 涟漪 ⇒ 必须带 alpha 合成，
     不能当不透明图块贴。

   【DRAM】每片叶子 (2r+7)² px，r ∈ [13,21] ⇒ 最大 49² = 2401 px。
     池子固定 6 片，按**最大**开槽：6 × 2401 × 3 B = 43,218 B。
     ⚠️ 铁律 21：这片 C3 的 DRAM 本来就贴着边（帧缓冲 153,600 B 是最大单项）。
        如果链接时 DRAM 溢出，先把 KOI_LILY_SNAP 关掉（编译期开关，见下），
        再考虑缩小 LILY_SNAP_W（改成按实际 r 开槽）。
   ========================================================================== */
#ifndef KOI_LILY_SNAP
#define KOI_LILY_SNAP   1
#endif

#define LILY_SNAP_W     (2 * 21 + 7)      /* r 上限 21 → 49 */
#define LILY_SNAP_PX    (LILY_SNAP_W * LILY_SNAP_W)

typedef struct {
    int       w, h;
    uint16_t *c;            /* 预乘色 RGB565 */
    uint8_t  *a;            /* 覆盖率 A8 */
    float     keyRot;       /* 建图时的量化自转角 */
    int       keyPal;       /* 建图时的调色板版本 */
    int       ok;
} lsnap_t;

#if KOI_LILY_SNAP
/* ⚠️ 别叫 s_lsa —— 那已经是荷叶"局部→世界"变换里的 sin 分量（s_lca/s_lsa）。 */
static uint16_t s_lsnap_c[LILY_SNAP_PX * MAX_LILY];  /* 28,812 B */
static uint8_t  s_lsnap_a[LILY_SNAP_PX * MAX_LILY];  /* 14,406 B */
static lsnap_t  s_lsnap[MAX_LILY];
#endif

#if KOI_LILY_SNAP
static void lily_snap_build(int li, float th)
{
    lsnap_t *S = &s_lsnap[li];
    const lily_t *L = &s_lily[li];
    int w = (int)(2.0f * L->r) + 7;
    if (w > LILY_SNAP_W) w = LILY_SNAP_W;
    int h = w;
    int slot = li * LILY_SNAP_PX;

    S->w = w; S->h = h;
    S->c = &s_lsnap_c[slot];
    S->a = &s_lsnap_a[slot];

    /* 预乘色的起点 = 全 0；覆盖率起点 = 全 0 */
    for (int i = 0; i < w * h; i++) { S->c[i] = 0; S->a[i] = 0; }

    uint16_t *sv_tgt = s_tgt; int sv_stride = s_stride; uint8_t *sv_a = s_asnap;
    int sv_cx0 = s_cx0, sv_cx1 = s_cx1, sv_cy0 = s_cy0, sv_cy1 = s_cy1;

    float ly = L->y + lily_bob(L);
    s_tgt = S->c; s_stride = w; s_asnap = S->a; s_aw = w;
    set_clip(0, 0, w - 1, h - 1);
    /* ★★ 亚像素对齐（第 52 轮踩的坑，务必看懂再改）：
       局部像素 i 覆盖世界区间 [L->x + i - w/2, +1)，而世界的像素 p 覆盖 [p, p+1)。
       要让"快照里第 i 个像素"正好等于"世界里某个像素"，**平移量必须是整数** ——
       否则快照的像素网格与世界的像素网格错开 frac(L->x) 个像素，
       贴回去就是一次**亚像素平移**：叶子内部全是细线（11 条叶脉 + 反光弧），
       一平移边缘抗锯齿全变 ⇒ A/B 对拍最大通道差 140、差异正好 6 个团块。
       ⇒ s_ofx/s_ofy 取 **floor**（整数），贴图时用同一个整数偏移。 */
    s_ofx = floorf(L->x) - (float)(w / 2);
    s_ofy = floorf(ly)   - (float)(h / 2);
    lily_paint(L, th);
    s_ofx = 0.0f; s_ofy = 0.0f;

    s_tgt = sv_tgt; s_stride = sv_stride; s_asnap = sv_a;
    set_clip(sv_cx0, sv_cy0, sv_cx1, sv_cy1);

    S->keyRot = th;
    S->keyPal = s_pal_ver;
    S->ok = 1;
}

static void lily_snap_blit(int li, float ly)
{
    const lsnap_t *S = &s_lsnap[li];
    const lily_t  *L = &s_lily[li];
    /* ★ 与建图时同一个 floor 对齐（见 lily_snap_build 里那段注释）：
       整数平移量 ⇒ 快照的像素网格与世界的像素网格严格重合，零亚像素误差。 */
    int ox = (int)floorf(L->x) - S->w / 2;
    int oy = (int)floorf(ly)   - S->h / 2;
    for (int j = 0; j < S->h; j++) {
        int y = oy + j;
        if (y < s_cy0 || y > s_cy1) continue;
        int base = j * S->w;
        uint16_t *dst = &s_fb[y * KW];
        for (int i = 0; i < S->w; i++) {
            int x = ox + i;
            if (x < s_cx0 || x > s_cx1) continue;
            int A = S->a[base + i];
            if (A == 0) continue;
            PCNT(px);
            WHO_PUT(x, y);
            uint16_t P = S->c[base + i];
            uint16_t d = dst[x];
            int k = 255 - A;                      /* out = P + 水色·(1−A) */
            int r = s_e5[(P >> 11) & 31] + (((int)s_e5[(d >> 11) & 31] * k) >> 8);
            int g = s_e6[(P >> 5)  & 63] + (((int)s_e6[(d >> 5)  & 63] * k) >> 8);
            int b = s_e5[P & 31]         + (((int)s_e5[d & 31]         * k) >> 8);
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            dst[x] = pack565(r, g, b);
        }
    }
}

static void lily_snap_draw(int li, float th, float ly)
{
    lsnap_t *S = &s_lsnap[li];
    /* 失效条件三个：没建过 / 自转角跳档 / 调色板重建（叶子颜色在 s_pal 里） */
    if (!S->ok || S->keyRot != th || S->keyPal != s_pal_ver) lily_snap_build(li, th);
    lily_snap_blit(li, ly);
}
#endif   /* KOI_LILY_SNAP */

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

/* ★★ 第 46 轮：头部两个可调量（王总「鱼的最前面 尖尖的 需要稍微打磨下」）
   --------------------------------------------------------------------------
   KOI_KDEPTH0 = 第 0 段（头）的半宽 / 腹部半宽。0.50 = 头只有肚子的一半宽。
   KOI_HEADCAP = 头端圆帽的**伸出量**，单位是头半宽 hw（见 body_pts 的 pt_quad）。
   量一下这两个数各自管什么（二次贝塞尔 A→C→B，A/B 在纵向 0、横向 ±hw，
   C 在纵向 cap·hw、横向 0）：
     · 顶点在 t=0.5 → 纵向伸出 h = cap·hw/2；
     · 顶点曲率半径 ρ = hw²/(2h) = **hw / cap**。
   ⇒ **cap 越大，鼻子越尖**（伸得更远、ρ 更小）；cap 越小，越钝越短。
     ⚠️ 第 41 轮把 cap 从 1.75 加到 2.50 时的注释写的是"让头部更圆" —— 那是**误判**：
        cap 加大只是把头**拉长**（看起来像穹顶），鼻尖本身反而更锐（ρ 从 0.571hw 掉到 0.400hw）。
        本轮把它做成阶梯实测，不再靠推。
   ⚠️ 这两个量都**只影响绘制、不影响物理** —— 所以同一帧号下不同档的鱼位/朝向逐位相同，
      可以直接拿同一块区域做 A/B（`_tools/noseladder46.py` 就是这么出卡的）。
   ⚠️ 调大 KDEPTH0 会**加大 AABB**（第 42 轮的注释：cap 2.50 + 深度 0.55 时满体单条脏盒
      多包几像素、真机退化 ~3ms）。改完必须看台架的「鱼脏盒最大欠缺」与真机帧时间。
   ★★ 第 46 轮定档：王总「鱼的最前面 尖尖的 需要稍微打磨下」⇒ **cap 2.20 → 1.70**。
      只动 cap、不动 KDEPTH0（他说的是"最前面"，那就只修鼻子本身，不把整条鱼改胖）。
      实测（`_tools/noseladder46.py`，同一帧号同一块裁切放大 14 倍）：
        吻尖厚度 4px → 6px（+50%），吻尖位置基本不动（-1px）
      —— 这正是"打磨"：**不缩短、只把尖角磨圆**。
      阶梯卡 `_preview/吻端_档位阶梯46.png` 里 A=2.20(旧) B=1.70(本轮) C=1.10(该构造极限)
      D/E/F=再叠头加宽。王总若要更钝，改这一个数就行。 */
#ifndef KOI_KDEPTH0
#define KOI_KDEPTH0 0.50f
#endif
#ifndef KOI_HEADCAP
#define KOI_HEADCAP 1.70f
#endif

static const float KDEPTH[KSEG + 1] = {KOI_KDEPTH0, 0.90f, 1.00f, 0.78f, 0.55f, 0.30f};
                                       /* ★ 第 43 轮微调（原 {0.50, 0.90, 0.92, 0.78, 0.55, 0.30}）：
                                          腹部 0.92→1.00（王总要"肚子稍微宽点点"；Wd 同步 0.155→0.175）。
                                          头仍 0.50（round 42 的妥协，不动）。比例 belly/head 从 1.84 → 2.00，
                                          比 round 40 原版 2.17 略小，仍然不像 r41 那样"头相对夸张"。
                                          ★ 第 46 轮：头那一项提成宏 KOI_KDEPTH0（值仍是 0.50）。 */
/* ★ 第 38 轮：王总「把初始鱼的大小做成现在的 3 倍」→ 2.0 → 6.0。
   ★ 第 39 轮：王总「把鱼做成现在的大小的一半」→ 6.0 → 3.0
   （= 原基线 2.0 的 1.5 倍，开局体长 17~23 × 3.0 = 51~69px）。
   倍率**必须同时**作用在体长 / 游速 vT / 吃食 / 同类避让 / 边界硬边距 ——
   统一从 kh = L*grow*0.55 推（第 15 轮定下的规矩），所以这里改一个数就够，
   下面每处 `* KOI_SCALE` 与每个从 kh 推的量都会跟着变。 */
/* ★★ 第 44 轮：王总先给了**绝对像素**规格（体长 24~26 / 体宽 12~14 ...），
   按规格做了 4 档尺寸阶梯（_preview/鱼尺寸_档位阶梯44.png，工具 _tools/koiladder44.py）
   给他挑，他的回答是「鱼尺寸都不想要 还回归咱们自己的鱼」
   —— 所以**体量一个数都没动**，回到第 43 轮上板版。
   下面这两个宏留着（值 = 原值），因为：
     · `KOI_WD` 把原来写死在 koi_draw 里的 0.175f 提成了名字 ——
       以后真要调体宽，是一个数的事，不用再去正文里找；
     · 台架的 `KOI_CDEFS=-DKOI_SCALE=... -DKOI_WD=...` 尺寸阶梯工具还能直接用
       （铁律 14：观感类改动先出阶梯让王总挑，别自己拍 —— 这次就是靠它一次问清的）。
   ⚠️ 值必须与第 43 轮逐位相同：3.0f / 0.175f。改一个数就会改画面。

   ★★ 第 48 轮：**3.0 → 2.0（回到网页版基线）**。
      王总第 48 轮把网页版 `锦鲤池_240x320预览.html` 发来，原话
      「这个鱼的 游动 和形状 还有游的动作 都非常的好 都要参考」。
      逐项对了一遍（网页版 1348~1388 / 1505 / 2499 行 vs 固件）：
        KOI_SCALE   网页 **2.0** / 固件 3.0     ← ★ 唯一的实质差异
        KOI_WD      0.175 / 0.175              一致
        KSEG        5 / 5                      一致
        KAMP        [.045 .197 .392 .618 .867] 一致
        KBEND       [.14 .24 .23 .21 .18]      一致
        KPHASE      0.92 / 0.92                一致
        vT 公式     (seek?24:(shake?26:19))×(0.62+0.46·grow)×(0.86+0.28·satiety)
                    ×(0.42+0.58·align)×**KOI_SCALE**   一致
        burst/coast 0.30~0.55 / 0.70~1.50      一致
      ⇒ **KOI_SCALE 一个数同时解释了王总连着两轮的两句抱怨**：
        · 鱼"太大" —— 倍率直接乘体长，3.0 的鱼是网页版的 1.5 倍大；
        · 鱼"游得还是有点快" —— 网页版 vT 公式里 **KOI_SCALE 是乘在速度上的**
          （第 15 轮的规矩：体长翻倍而速度不变，鱼看着就像漂在水里），
          所以 3.0 的鱼速也是网页版的 1.5 倍。
        ⇒ 回到 2.0：形状与速度**同时**回到网页版口径，两个问题一起消。
      ⚠️ 副作用（都是按比例缩的，不是新引入的问题）：
        · 鱼变小 1/3 ⇒ 红斑 8 顶点多边形在小鱼上要重新看（见 KOI_SPOT_SEGS 的图）；
        · 脏区占比下降（鱼小了）⇒ 帧时间会**变快**，是好事；
        · 台架活泼度读数从 27.1px/s 降到约 18.1px/s（27.1 × 2/3），
          这正是网页版的巡游速度 —— 不是"变慢了"，是"回到设计值"。 */
#ifndef KOI_SCALE
#define KOI_SCALE       2.0f
#endif
#ifndef KOI_WD
#define KOI_WD          0.175f
#endif

/* ★★ 第 52 轮：初始体长区间（原写死 rnd_f(17,23)，×KOI_SCALE=2.0 ⇒ 34~46px 基础体长）。
   ⚠️ 为什么只改这里、不改 grow / KOI_SCALE —— 那两个**都进 vT 速度公式**（见 3285 行）：
        vT = (...) × (0.62 + 0.46·k->grow) × (...) × KOI_SCALE × KOI_LV_SPD
      · grow 初始 0.52~0.72 抬到 0.62~0.82 ⇒ 速度跟着 +5%（王总抱怨过"游得快"）；
      · KOI_SCALE 2.0→2.2 ⇒ 速度 +10%。
      而 **k->L 既不在 vT 里、也不在速度上限 `34·KOI_SCALE·KOI_LV_SPD` 里**
      ⇒ 改它 = 纯变大、游速零影响。这是唯一干净的旋钮。
   实测体长（240×320 屏，grow 0.52~0.72 / 满级 1.35）：
      17~23 → 开局 17.7~33.1px / 满级 45.9~62.1px   ← 第 45 轮起的现状
      19~25 → 开局 19.8~36.0px / 满级 51.3~67.5px   ← ★ 第 52 轮现值（+11%）
      21~27 → 开局 21.9~38.9px / 满级 56.7~72.9px   （更明显，但脏区 +48%，先看帧率）
   ⚠️ 台架要复验两项：鱼脏盒最大欠缺（现 −4.00px）、真机帧时间（鱼 47ms / 荷 60ms）。
   ⚠️ 改成宏是为了台架能用 -D 扫档（裸 #define 会被 -D 覆盖后又改回来）。 */
#ifndef KOI_L0_MIN
#define KOI_L0_MIN  19.0f
#endif
#ifndef KOI_L0_MAX
#define KOI_L0_MAX  25.0f
#endif

#define GROW_MAX        1.35f
#define GROW_PER_PELLET 0.018f

/* ==========================================================================
   ★★ 第 46 轮：活泼度档位（王总「鱼活泼感一定要弄出来」）
   --------------------------------------------------------------------------
   五个乘数，全 1.00 = 第 45 轮上板值（逐位不变，历史对照图不作废）；
   本轮把默认值改成 **P1**（见下面的实测表），全 1.00 仍然可复现（命令行 -D 全给 1.00）。
   全部只作用在 `koi_step` 里，且都乘在**已经量出来的标定值**上，
   所以"全 1.00"与原版逐位等价，档位之间是单调的"更活泼"。
     · KOI_LV_SPD  ：巡游/抢食速度 vT（连带 `k->v` 上限）
     · KOI_LV_HZ   ：摆尾频率（滑行 1.5 / 冲刺 2.4 / 抢食 3.0 Hz）
     · KOI_LV_TURN ：转向响应 4.2 与转向速率 2.9（两个一起乘，只改快慢不改形状）
     · KOI_LV_WAND ：漫游推力 1.05 + 换目标间隔 3.4~6.4s 的倒数
                     （越大 = 越主动改方向，是"活泼"最直接的一项）
     · KOI_LV_GAIT ：步态节奏。burst 时长 ×LV、coast 时长 ÷LV
                     （>1 = 冲得多、滑得少 —— "飘逸"就是 coast 占比太高）
   ⚠️ 这几个只改"运动"，不改体量/几何 ⇒ 不需要重跑体量工具。
   ⚠️ 上限：真机一帧 ~105ms ⇒ 每帧推进 105ms（`frame_dt`，无子步量化）。
      速度提到 2 倍以上时，先看台架的「安全区最大越界」与真机帧时间再往上加。

   ★★ 第 46 轮定档：**P1「活泼」**（见下），不是全 1.00。
      为什么不再留全 1.00：全 1.00 = 第 45 轮上板值 = 王总刚说的"不活泼 / 很飘逸"，
      留着等于没改。他这次的原话是「鱼活泼感**一定要**弄出来」，所以给的是结论不是选项。
      ⚠️ 但"不活泼"的**主因不是这几个数太小** —— 是**时间膨胀**（见 frame_dt）：
         物理原来写死 step(1/24s)，真机一帧 105ms ⇒ 鱼速与摆尾频率一起只剩 40%。
         那是先修的（修完时间膨胀比 1.000）。这五个乘数是在"时间已经对得上"之后
         再往上加的主观活泼度。

   ★★ 第 47 轮回调：**P1 → P0.5（增量减半）**。
      王总看完 v10 说「鱼太活跃了 稍微往下减成现在的一半」。
      ⚠️ "减成一半"要减的是**相对 baseline 的增量**，不是把乘数砍到 0.5 倍：
         全 1.00（P0）就是他上一轮嫌"不活泼"的那版，砍到 0.5 倍会比那还慢，
         等于把上一轮的修复又退回去。所以取 P0 与 P1 的**中点**——
         比"不活泼"那版明显活泼，但只有 v10 的一半劲儿。
           SPD  1.35 → 1.18      HZ   1.10 → 1.05
           TURN 1.30 → 1.15      WAND 1.40 → 1.20
           GAIT 1.60 → 1.30      （GAIT 管冲刺占空，是"飘逸感"的主旋钮）

   ★★ 第 48 轮：**P0.5 → P0（全 1.00）**。
      王总看完 v11 说「鱼游的还是有点快活了 减下」。
      ⚠️ ⚠️ **"全 1.00"这个数在第 45 轮与现在含义完全不同** —— 这是最容易看错的一点：
        第 45 轮的 P0：时间膨胀还在（`step(1/24s)`，真机一帧 105ms ⇒ 只推进 41.7ms）
                      ⇒ 鱼速实际只剩 **40%**（那才叫"不活泼"，王总原话）。
        现在的 P0  ：时间膨胀已修（`frame_dt()`，时间膨胀比 1.000）
                      ⇒ 同样的 1.00，**实际速度是第 45 轮的 2.5 倍**。
        所以"退回全 1.00"**不是**退回第 45 轮那个"不活泼"的版本 —— 那版的慢是 bug，
        不是参数。现在这五个乘数才是纯粹的"主观活泼度旋钮"，从 1.00 起算。
      ⇒ 若王总还要再慢，下一档就不是动这五个乘数了，而是动 `koi_step` 里的
        **基础速度 vT**（`19.0f` 巡游 / `24.0f` 抢食 / `26.0f` 抢食冲刺），
        那是另一个量级的手感，改之前先出阶梯。
   --------------------------------------------------------------------------
   台架实测（`_tools/lvsweep46.py`，KOI_FRAMEMS=105，稳态帧·鱼平均）：
     档位        巡游速度   转头速率   |弯曲|   冲刺占空
     P0 全1.00   27.1px/s  0.43rad/s  0.150   29.6%   ← ★ 第 48 轮默认
     P0.5        37.0      0.54       0.162   40.7%   ← 第 47 轮（王总说"还是有点快"）
     P1 活泼     47.0      0.69       0.182   50.2%   ← 第 46 轮（王总说"太活跃"）
     P2 很活泼   58.0      0.82       0.195   57.7%
     P3 只快不冲 45.0      0.62       0.177   29.7%   ← 对照：证明"快"不等于"活泼"，
                                                        冲刺占空不动时照样"飘逸"
   ⚠️ 表里这些数是**物理速度**（台架探针口径 = 帧初帧末位移 ÷ dt），
      与时间膨胀无关，所以第 45 轮与现在**同一档读数相同** ——
      但**真实观感速度**差 2.5 倍（见上面第 48 轮那段）。别拿这两个数直接比。
   四档的安全区越界都是 0.00px、金标准两条判据（脏区渲染 / 脏区覆盖）全 PASS。
   ★ 档位表已备好，王总再要调只需改这五个宏，不用重新找参数。
   ========================================================================== */
#ifndef KOI_LV_SPD
#define KOI_LV_SPD      1.00f     /* ★ 第 48 轮 P0.5(1.18) → P0(1.00) */
#endif
#ifndef KOI_LV_HZ
#define KOI_LV_HZ       1.00f     /* 同上 */
#endif
#ifndef KOI_LV_TURN
#define KOI_LV_TURN     1.00f     /* 同上 */
#endif
#ifndef KOI_LV_WAND
#define KOI_LV_WAND     1.00f     /* 同上 */
#endif
#ifndef KOI_LV_GAIT
#define KOI_LV_GAIT     1.00f     /* 同上 */
#endif
/* ★ 报脏外扩量（第 34 轮）。它要盖住"形状本身的变化"，而不仅仅是位移：
   一帧之内 AABB 还会因为 ① 转身（绕 (x,y) 转 dθ，最远点位移 ≈ R·dθ）
   ② 身体摆动 ③ 吃食长大 而变。位移由"上帧 AABB ∪ 平移副本"覆盖，这三样靠这个 margin。
   值不是拍的：台架里有个自检会逐帧量"真实 AABB 比预测盒超出多少"，
   取全帧最大值 —— 见 KOI_DIRT_MARGIN 的读数（kb_probe_bbox_short）。
   ★★ 第 46 轮：4.0 → **6.0**，起因是**帧长变了**。
     物理从"每帧固定推进 1/24 秒"改成"按真实经过时间推进"（见 frame_dt）之后，
     真机一帧 ~105ms 意味着**摆尾相位、曲率、体量在一帧里走得比原来远 2.5 倍**，
     自检读数从 1.75px 涨到 6.8~14px。
     ■ 先修的是**算法**（不是先加 margin）：报脏改成"绕头按 ΔheadA 与 ΔheadA+Δcurv
       各转一次再求并"—— 因为 koi_spine 里 a += curv·KBEND[i] 且 KBEND 之和 = 1.00，
       尾端绝对角 = headA + curv，旧口径只转了头那一项。这一改把 105ms 的欠缺
       从 2.77px 打到 0.86px、142ms 从 10.00px 打到 2.28px，**脏区占比几乎不变**。
     ■ 剩下的 0.86px 用 margin 兜：4.0 → 6.0 覆盖到 ~115ms 帧长。
     ■ 代价是**量过的**：台架 KOI_FRAMEMS=105 下脏区占比 36.2% → 40.9%，
       但渲染分项几乎不动（水 0.02 鱼 0.18 荷 0.22 → 水 0.02 鱼 0.18 荷 0.23）——
       多出来的像素是**水**（memcpy），几乎免费。真机唯一的代价是 SPI：
       0.4µs/px × 76800 × 4.7% ≈ **1.4ms/帧（105ms 的 1.3%）**。
     ⚠️ 帧长 >115ms 的偶发帧上自检仍会读到 1~3px 欠缺（抢食冲刺那几帧），
        但**金标准两条判据（脏区渲染 / 脏区覆盖）在 42~200ms 全帧长下都 PASS** ——
        没有真实残留（脏矩形合并 + AABB 自带的 −1/+2 余量吃掉了）。
        这是"最坏情况预检"与"真实残留"的差别，别把预检读数当成拖影。 */
#ifndef KOI_DIRT_MARGIN
#define KOI_DIRT_MARGIN  6.0f
#endif
#define KMOUTH  (0.175f * 0.46f * 0.875f)
#define BITE_T    0.42f
#define BITE_SLOW 0.10f

/* ★★ 第 50 轮：吃食涟漪（鱼嘴前那圈"吃到啦"的小圆）的参数**从 BITE_T 里拆出来**。
   王总第 50 轮原话：「鱼吃到食物了，前面有个跟撒饲料一样的特效 感觉没有」。

   ★ 根因**不是**参数抄错了 —— 固件与网页版逐位相同（r0=1、rMax=9、life=0.42、
     a0=0.70，见 koi_step 里那次 ripple_add）。真正的差别是**帧率**：
       · 网页版 60fps：0.42s 的涟漪画 **25 帧**，看得清清楚楚；
       · 真机 7fps（帧长 143ms）：0.42s 只画得到 **2.9 帧** —— 一闪就没了。
     所以这两个数必须**按固件的帧率**重新定，照抄网页版在这里是错的。

   ⚠️ 为什么必须独立成宏、不能接着共用 BITE_T：BITE_T 还是"啄食停顿"（k->biteT），
     它决定鱼吃完在原地停多久。王总这轮明确说「只改波纹，不要改鱼游动」——
     若把涟漪时长塞进 BITE_T，鱼会跟着停 1 秒，那是改了鱼的行为，不是改特效。

   参考系：撒饲料落水那圈 drop 涟漪是 rMax=rnd(7,14)、life=rnd(0.5,0.75)，
   王总说要"跟撒饲料一样"，所以往这两个数靠（见 _tools/ripple50.py 的三档阶梯）。 */
#ifndef EAT_RING_RMAX
#define EAT_RING_RMAX  12.0f     /* 第 50 轮：9 → 12（对齐落水那圈 drop 的中值） */
#endif
#ifndef EAT_RING_LIFE
#define EAT_RING_LIFE  0.85f     /* 第 50 轮：0.42 → 0.85（真机 7fps 下 ≈ 6 帧） */
#endif

/* ★★ 第 51 轮：曾经试过"追食途中也冒水花"，**王总否掉了**（「这个不要啊」）。
     保留这段说明免得以后再提：做法是在 seek 期间每 0.35s 往吻端放一个
     小涟漪（r=5 / life=0.45）。否决理由（推测）：鱼一路冒水花会把"吃到那一下"
     淹掉 —— 到处都是圈，反而看不出哪一下是真吃到了。
   ⇒ **只在真吃到那一刻放一圈**（EAT_RING_RMAX=12 / EAT_RING_LIFE=0.85），
      途中什么都没有。想再试就按上面三个数加回 SEEK_FX_*，别重新设计。 */

/* ★ 第 51 轮：**落水的食物画不画**。
     0 = 不画（王总「显示是隐藏的」）—— 它只作为鱼的追踪/进食目标存在；
     1 = 画出来（第 43 轮之前的口径，池里能看到一颗颗饲料）。
   ⚠️ 这是**纯显示开关**，不影响 s_food 的生命周期与鱼的追食逻辑。 */
#ifndef KOI_FOOD_DRAW
#define KOI_FOOD_DRAW  0
#endif

/* ★ 第 51 轮："嘴够得到"的判定半径里那个**固定余量**（px）。
     旧值 3.0 ⇒ 总半径 = L·grow·0.0805 + 3.0 ≈ **4.9px**（L≈40、grow≈0.6）；
     而真机 7fps 下鱼一帧就走 **3.9px** —— 窗口比步长还窄，鱼经常擦身而过不算吃到，
     这就是王总说的「鱼碰到鱼食的几率很小」。
     放到 6.0 ⇒ ≈7.9px，约等于鱼两帧的步长，够得着了。 */
#ifndef KOI_EAT_REACH
#define KOI_EAT_REACH  10.0f
#endif
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

/* ★★ 第 47 轮：红斑**形状**两个参数（提成宏，供 _tools/spotshape47.py 出阶梯）
   · KOI_SPOT_SEGS：斑的顶点数。14 = 旧值（视觉上就是个圆点）；
                    8 = 本轮默认（明显不规则）。下限 6（再少就成三角形了）。
   · KOI_SPOT_JIT ：顶点径向扰动幅度。0 = 正圆（旧观感）；0.30 = 本轮默认。
   ⚠️ 两个宏只影响"形状"，**不消费 rnd**（扰动是确定性三角函数，见绘制处）——
      所以改档位不会让随机序列错位（铁律 20）。 */
#ifndef KOI_SPOT_SEGS
/* ★ 第 54 轮：8 → 11。王总"红斑不要画成规则圆点…边缘有轻微凹凸"。
   8 个顶点的多边形在 sl≈0.10L（≈5px）的斑上，每个边就 2px 长，
   读出来还是"带棱角的圆"；11 个顶点 + 双频扰动才够碎。
   ⚠️ 上限 12：再多就是纯浪费（11 顶点已经把 5px 的斑切到 1.4px 一段）。 */
#define KOI_SPOT_SEGS   11
#endif
#ifndef KOI_SPOT_JIT
/* ★ 第 54 轮：0.30 → 0.34。扰动现在是**双频叠加**（0.68·cos2.3θ + 0.32·cos3.7θ），
   两个频率同时取到峰的概率很低 ⇒ 实际起伏比单频小，所以要补一点幅度回来。 */
#define KOI_SPOT_JIT    0.34f
#endif

/* ★ 第 51 轮：红斑的**整体尺寸**倍率（王总「红色在鱼身上覆盖的稍微再多点占比」）。
     1.00 = 第 47 轮定下的形状（8 顶点 + 0.30 径向扰动）原尺寸
     1.15 = 面积 +32%（"稍微多点"）
     1.30 = 面积 +69%
   ⚠️ 只乘在 sl/sw 上，**不动 k->sp[][] 的随机系数**（动它们会挪动全局随机序列）。
   ⚠️ 放太大红斑会连成一片、盖住白底 —— 那就不是"红白"了，1.30 是上限。 */
/* ★ 第 51 轮定档 **1.15**（王总「现在鱼的红斑占比要再多点」）。
     阶梯与判据见 `_preview/红斑占比_三档51.png`（整屏统计 红斑/(红斑+白身)）：
       ×1.00 ≈ 22.0%（第 47 轮原尺寸）
       ×1.15 ≈ **27.7%**  ← ★ 现值，面积 +32%，"稍微再多点"
       ×1.30 ≈  36.8%（再大就连成片、白底被盖掉，就不是"红白"了）
   ⚠️ 别搞错顺序：王总原话是「先弄喂食特效」然后「红斑占比要再多点」——
      **两件都要**，不是"做了喂食就不要红斑"。我曾误读成后者回滚过一次。 */
#ifndef KOI_SPOT_SCALE
#define KOI_SPOT_SCALE  1.15f
#endif

/* ★★ 第 53 轮：御黄金金属层次的**四个渐变强度**（0..256）。
   王总给的五个色（见 DAY_PAL 的 PI_KGOLD_* 注释）之外，还得定"各层压多重"。
   这四个是纯观感刻度 —— 王总看完真机要微调就改这几个，别去动几何。
     GOLD_SIDE_A  两侧（腹）压暗   124 ≈ 0.48
     GOLD_BACK_A  背部中央窄高光   116 ≈ 0.45（色本身很亮 #FFF0A0，再高就发白）
     GOLD_HEAD_A  头部略亮          90 ≈ 0.35
     GOLD_TAIL_A  尾根压暗         108 ≈ 0.42
   ★ 为什么从 96/92/72/84 提到 124/116/90/108（第 53 轮第二次调）：
     第一版按"金属反射应该 subtle"取了 0.28~0.38，台架 240×320 **就是真机的分辨率**，
     出图一看：鱼身只有 50px 长、~17px 宽，那么小的面积上 0.3 的压暗
     被 RGB565 的量化（每通道只有 32/64 级）吃掉大半 ⇒ 读出来还是"一片黄纸"。
     ★ 记一条判据：小面积上的渐变，alpha 不够 0.45 就看不见。
   ⚠️ 窄带的**宽度**在 koi_draw 里写死成 ±0.13Wd（一侧 0.42Wd 渐隐）——
      那是"很窄"的几何量化，比 alpha 更影响"是不是一条线"，要改就改那边。 */
#ifndef GOLD_SIDE_A
#define GOLD_SIDE_A    124
#endif
#ifndef GOLD_BACK_A
#define GOLD_BACK_A    116
#endif
#ifndef GOLD_HEAD_A
/* ★ 第 54 轮：90 → 135。王总第 54 轮 v19 后反馈「黄金的鱼，鱼头处颜色有点深了」。
   复盘：③ 头部略亮的 alpha 只 90，比 ① 两侧压暗(124) 弱一头；
   再加上 ④ 尾根压暗的零区 s2=-0.10Hl 离中点很近 ⇒ 中段前部还在压暗，
   两个暗层夹一束弱亮 ⇒ 净效果是头偏暗。
   修：把 HEAD_A 加到 135（与 SIDE/TAIL 同档），并把 ③ 的"渐强起点"前挪到 -0.75Hl、
   把 ④ 的"归零点"推到 +0.25Hl ⇒ 头端只剩 ①+②+③ 三层，无 ④ 干扰。 */
#define GOLD_HEAD_A    135
#endif
#ifndef GOLD_TAIL_A
#define GOLD_TAIL_A    108
#endif
/* ★★ 第 53 轮：尾鳍「从金黄色向半透明浅金色过渡」。
   ⚠️ grad_t **只插值 alpha、不插值颜色**（见 4 节定义），所以"换个颜色渐变"
      做不到单层 —— 正解是**两层渐变交叉淡化**：
        第一层 金黄 PI_KGOLD   alpha 由根到梢 236 → GOLD_TJ_A2（**递减**）
        第二层 浅金 PI_KGOLD_TL alpha 由根到梢 0   → GOLD_TJ_LT2（**递增**）
      合成后：根部是纯金黄、中段两色各半、梢部以浅金为主且总不透明度下降
      ⇒ 读感就是"金黄 → 半透明浅金"。
      梢部残留给背景的比例 = (1−a1)(1−a2)：a1=70/256、a2=176/256 ⇒ 约 21%，
      水色透上来 ⇒ "半透明"。 */
#ifndef GOLD_TJ_A1
#define GOLD_TJ_A1     150      /* 金黄层：中段 alpha */
#endif
#ifndef GOLD_TJ_A2
#define GOLD_TJ_A2      70      /* 金黄层：尾梢 alpha（原 TJ_GG[2] = 82，再降一点） */
#endif
#ifndef GOLD_TJ_LT1
#define GOLD_TJ_LT1     88      /* 浅金层：中段 alpha（开始接手） */
#endif
#ifndef GOLD_TJ_LT2
#define GOLD_TJ_LT2    176      /* 浅金层：尾梢 alpha */
#endif

/* ★★ 第 54 轮：红白鲤立体化的**强度刻度**（0..256）。
   王总给的六个色之外，"各层压多重"是纯观感 —— 全提成宏，看完真机要调就改这几个。
     KBODY_SIDE_A  身体两侧浅灰白   112 ≈ 0.44
     KBODY_DK_A    腹 / 尾根稍深灰   128 ≈ 0.50
     KBODY_BACK_A  背中央亮白窄带    120 ≈ 0.47（“**非常窄**的柔和高光”）
     KBODY_TAIL_A  尾根压暗（前后层次）118 ≈ 0.46
     KSPOT_LT_A    红斑受光亮朱红    150 ≈ 0.59
     KSPOT_DK_A    红斑两侧/边缘深红 140 ≈ 0.55
   ★ 为什么这套数比御黄金（96~124）**整体更重**：
     白身三档之间只差 64/255，而黄金五档差到 70~130 —— 白身的对比天生更弱，
     不加重就完全是一片白（台架 240×320 就是真机分辨率，出图确认过）。
   ⚠️ 几何（窄带宽度 ±0.12Wd 等）在 koi_draw 里写死，比 alpha 更影响
      "是不是一条线"，要改去那边。 */
#ifndef KBODY_SIDE_A
#define KBODY_SIDE_A   112
#endif
#ifndef KBODY_DK_A
#define KBODY_DK_A     128
#endif
#ifndef KBODY_BACK_A
#define KBODY_BACK_A   120
#endif
#ifndef KBODY_TAIL_A
#define KBODY_TAIL_A   118
#endif
#ifndef KBODY_HEAD_R
#define KBODY_HEAD_R   1.25f      /* 头顶那 1 个小高光的半径（px） */
#endif
#ifndef KBODY_HEAD_A
#define KBODY_HEAD_A   205
#endif
#ifndef KSPOT_LT_A
#define KSPOT_LT_A     150
#endif
#ifndef KSPOT_DK_A
#define KSPOT_DK_A     140
#endif

/* 鳞片点 / 背上高光点：半径（px）与 alpha。
   王总原话「1~2 个**很小**的淡金/近白高光点」⇒ 半径必须 ≤1.2px，大了就成疮。 */
#ifndef GOLD_SCALE_R
#define GOLD_SCALE_R   0.85f
#endif
#ifndef GOLD_SCALE_A
#define GOLD_SCALE_A   130
#endif
#ifndef GOLD_HIDOT_R
#define GOLD_HIDOT_R   1.15f
#endif
#ifndef GOLD_HIDOT_A
#define GOLD_HIDOT_A   190
#endif

static void make_spots(koi_t *k)
{
    if (k->pat == 1) { k->ns = 0; return; }               // 黄金鲤不该有红斑
    /* ★★ 第 44 轮：这里曾按王总的规格「花纹：2~3 块大色斑」改成只出大斑
       （原版是大小斑交替 `big = (i % 2 == 0)`）。他看过尺寸阶梯后说
       「鱼尺寸都不想要 还回归咱们自己的鱼」，已**逐字撤回**。
       记一句当时的观察，免得下次重新踩：体长缩到 25px 之后，小斑 sl=0.052~0.072
       只剩 1.3px，在屏上就是一粒噪点 —— 也就是说"小斑"这个设计只在
       L≈40px 以上才读得出来，是**跟体量强耦合**的。真要把鱼缩小，这条必须一起改。 */
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
    k->L = rnd_f(KOI_L0_MIN, KOI_L0_MAX) * KOI_SCALE;   /* ★ 52 轮：17~23 → 19~25 */
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
#if KOI_LILY_SNAP
    /* ★ 52 轮：重建池子 ⇒ 荷叶的 r / gap / 位置全换了，旧快照一律作废。
       ⚠️ 少了这一句，"长按 OK 重置"之后会贴上**上一池**的叶子（同下标不同叶）。 */
    for (int i = 0; i < MAX_LILY; i++) s_lsnap[i].ok = 0;
#endif
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
        float kx = (float)KW * 0.5f + cosf(ia) * ((float)KW * 0.5f - 46.0f) * ir;
        float ky = (float)KH * 0.5f + sinf(ia) * ((float)KH * 0.5f - 64.0f) * ir;
        /* ★★ 第 45 轮：出生点也必须在安全区里 —— 否则第一帧就被 swim_push "啪"地
           推回来，看起来像鱼出生时跳了一下。
           ⚠️ swim_push **不抽随机数**，所以插在这里不会平移后面荷叶/波光点的随机
              序列（这一点很关键，见下面荷叶那段注释）。若哪天它开始抽随机数，
              必须把出生点改成"先算完所有随机数、最后统一推回"。 */
        swim_push(&kx, &ky, 40.0f);      /* 出生时给 40px 余量：鱼还要长大 */
        make_koi(&s_koi[i], kx, ky, rnd_f(0.52f, 0.72f), pats[i]);
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
            _spx[0] + fcos_t(ha) * hw * KOI_HEADCAP, _spy[0] + fsin_t(ha) * hw * KOI_HEADCAP,
            _rx[0], _ry[0]);                                 /* ★ 第 42 轮微调：head cap 2.50→2.20。
                                            第 41 轮 1.75→2.50（当时以为是"让头部更圆"，实为**误判**：
                                            cap 越大鼻子越尖、只是头被拉长 —— 见 KOI_HEADCAP 的注释），
                                            但 +KDEPTH[0]=0.55 一起作用后，满体单条脏盒多包了几像素，
                                            真机退化 ~3ms。本轮 cap 退回 2.20，仍比 1.75 长（多包
                                            hw*0.45 vs hw*0.225），但 AABB 增长砍半。
                                            配合 KDEPTH[0] 0.55→0.50，整体观感仍是"圆头 + 不胖肚"。
                                            ★ 第 46 轮：2.20 提成宏 KOI_HEADCAP（值不变），
                                              供 _tools/noseladder46.py 出档位阶梯。 */
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
    float Wd = L * KOI_WD;                                  /* ★ 第 43 轮：0.155→0.175（王总"鱼身体有点显长 肚子稍微宽点点"——整体加宽，配合 KDEPTH[2] 0.92→1.00 把肚子收回一点，整体比例更接近原 r40 但 belly/head 比从 1.84 改到 2.00）
                                                               ★ 第 44 轮：写死的 0.175 提成 KOI_WD 宏（值仍是 0.175）—— 第 44 轮试过按王总的像素规格改成 0.295，他看过阶梯后说"还回归咱们自己的鱼"，已撤回。 */
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

    /* ★★ 第 53 轮：御黄金的**金属金层次**（只给 pat==1，红白一个字节都不动）
       ⚠️⚠️ 全部用 fill_poly 的 **grad（逐像素插值）** —— 这是第 47 轮否决后、
          注释里明确写下的正解：
            第 47 轮做的是"沿 spine 等距偏移的折线多边形"，第 48 轮被王总否掉
            （"金色鱼背部有条线不对劲 做纯色就行"）。根因：折线的两条边在**弯曲**
            的鱼身上是斜切过身体的直边，叠半透明深褐就成了一条明显的线，
            而"金属感"要的是**明暗过渡**不是轮廓线。
          ⇒ grad 沿方向 u 逐像素插值 alpha，天然是软过渡，不会有硬边。

       四层（王总指定的五个色 + 四条要求）：
         ① 两侧（腹）压暗   PI_KGOLD_DK #B96812   横向 grad，|s|=Wd 处最强、中央 0
         ② 背部中央窄高光   PI_KGOLD_HI #FFF0A0   横向 grad，只在 ±0.13Wd 的窄带里
         ③ 头部略亮         PI_KGOLD_LT #FFD34E   纵向 grad，越靠头越亮
         ④ 尾根压暗         PI_KGOLD_DK #B96812   纵向 grad，尾端最强、中段归零
       ⚠️ 四个 alpha 提成宏：这是纯观感刻度，王总看完可能要微调，别写死。
       ⚠️ 成本：鱼身多边形多填 4 遍（面积 ~300px ⇒ +1200 像素/条）。
          荷叶快照省下来的 ~55ms 就是给这个留的余量。 */
    if (isGold) {
        int   mid = KSEG / 2;
        float ox  = _spx[mid], oy = _spy[mid];           /* 投影原点 = 鱼身中点 */
        float nx  = -fsin_t(_spa[mid]), ny = fcos_t(_spa[mid]);   /* 横向（两侧） */
        float tx  =  fcos_t(_spa[mid]), ty = fsin_t(_spa[mid]);   /* 纵向（头→尾） */
        float Wq  = (Wd > 1.0f) ? Wd : 1.0f;
        float Hl  = L * 0.5f;                            /* 鱼身半长（近似） */
        grad_t g;

        /* ① 两侧压暗：|s| = 1.15Wd 处最强，|s| = 0.45Wd 处归零 */
        g.ux = (int32_t)rne_f2i(nx * 256.0f); g.uy = (int32_t)rne_f2i(ny * 256.0f);
        g.cx = (int32_t)rne_f2i(ox * 256.0f); g.cy = (int32_t)rne_f2i(oy * 256.0f);
        g.s0 = (int32_t)(-Wq * 1.15f * 256.0f); g.a0 = GOLD_SIDE_A;
        g.s1 = (int32_t)(-Wq * 0.45f * 256.0f); g.a1 = 0;
        g.s2 = (int32_t)( Wq * 0.45f * 256.0f); g.a2 = 0;
        g.s3 = (int32_t)( Wq * 1.15f * 256.0f); g.a3 = GOLD_SIDE_A;
        fill_poly(s_poly, s_npts, s_pal[PI_KGOLD_DK], &g, 256);

        /* ② 背部中央**很窄**的高光带：只在 ±0.13Wd 内，两侧到 0.42Wd 渐隐 */
        g.s0 = (int32_t)(-Wq * 0.42f * 256.0f); g.a0 = 0;
        g.s1 = (int32_t)(-Wq * 0.13f * 256.0f); g.a1 = GOLD_BACK_A;
        g.s2 = (int32_t)( Wq * 0.13f * 256.0f); g.a2 = GOLD_BACK_A;
        g.s3 = (int32_t)( Wq * 0.42f * 256.0f); g.a3 = 0;
        fill_poly(s_poly, s_npts, s_pal[PI_KGOLD_HI], &g, 256);

        /* ③ 头部略亮：u 取**指向头**的方向（切向是头→尾，取负）。
              ★★ 第 54 轮修正：渐强起点从 -0.20Hl 提前到 -0.75Hl（从尾后段就开始渐亮），
              满档从 +0.55Hl 提前到 +0.30Hl（更早到满）。原来中段还在压暗，
              头端的"略亮"补不回来 ⇒ 读出来"头偏暗"。 */
        g.ux = (int32_t)rne_f2i(-tx * 256.0f); g.uy = (int32_t)rne_f2i(-ty * 256.0f);
        g.s0 = (int32_t)(-Hl * 1.10f * 256.0f); g.a0 = 0;
        g.s1 = (int32_t)(-Hl * 0.75f * 256.0f); g.a1 = 0;
        g.s2 = (int32_t)( Hl * 0.30f * 256.0f); g.a2 = GOLD_HEAD_A;
        g.s3 = (int32_t)( Hl * 1.10f * 256.0f); g.a3 = GOLD_HEAD_A;
        fill_poly(s_poly, s_npts, s_pal[PI_KGOLD_LT], &g, 256);

        /* ④ 尾根压暗：尾端最强、中段归零。
              ★★ 第 54 轮修正：归零点从 s2=-0.10Hl 推到 s2=+0.25Hl（中点之后才归零），
              渐降区间也从 [-0.55,-0.10] 移到 [-0.30,+0.25] —— 头部完全脱离 ④ 的覆盖，
              让头端只剩 ① 弱 + ② 背中央亮 + ③ 头部亮 三层亮色。 */
        g.s0 = (int32_t)(-Hl * 1.10f * 256.0f); g.a0 = GOLD_TAIL_A;
        g.s1 = (int32_t)(-Hl * 0.30f * 256.0f); g.a1 = GOLD_TAIL_A;
        g.s2 = (int32_t)( Hl * 0.25f * 256.0f); g.a2 = 0;
        g.s3 = (int32_t)( Hl * 1.10f * 256.0f); g.a3 = 0;
        fill_poly(s_poly, s_npts, s_pal[PI_KGOLD_DK], &g, 256);

        /* ★ 鳞片点 + 背上 1~2 个很小的淡金高光点（王总："鱼背上加 1~2 个很小的
             淡金/近白高光点"）。位置**确定性**（由段号推出，不抽 rnd —— 铁律 20，
             动随机序列会把整池鱼的位置/荷叶全挪位）。 */
        static const float SCL_SG[4] = {1.35f, 2.05f, 2.75f, 3.35f};
        static const float SCL_OF[4] = { 0.34f, -0.26f,  0.30f, -0.22f };
        for (int q = 0; q < 4; q++) {
            float sg = SCL_SG[q];
            int   i0 = (int)sg; float tt = sg - i0;
            if (i0 < 0) i0 = 0;
            if (i0 > KSEG - 1) i0 = KSEG - 1;
            float px = _spx[i0] + (_spx[i0 + 1] - _spx[i0]) * tt;
            float py = _spy[i0] + (_spy[i0 + 1] - _spy[i0]) * tt;
            float aa = _spa[i0];
            float ax = -fsin_t(aa), ay = fcos_t(aa);
            float off = SCL_OF[q] * Wd;
            px += ax * off; py += ay * off;
            ipt_t d[8];
            for (int v = 0; v < 8; v++) {
                d[v].x = (int32_t)rne_f2i((px + s_oct_u[v][0] * GOLD_SCALE_R) * 256.0f);
                d[v].y = (int32_t)rne_f2i((py + s_oct_u[v][1] * GOLD_SCALE_R) * 256.0f);
            }
            fill_poly(d, 8, s_pal[PI_KSCALE], NULL, GOLD_SCALE_A);
        }
        /* 背上那 1~2 粒：更靠脊线、更大一点、更亮 */
        static const float HI_SG[2] = {1.70f, 2.60f};
        for (int q = 0; q < 2; q++) {
            float sg = HI_SG[q];
            int   i0 = (int)sg; float tt = sg - i0;
            if (i0 < 0) i0 = 0;
            if (i0 > KSEG - 1) i0 = KSEG - 1;
            float px = _spx[i0] + (_spx[i0 + 1] - _spx[i0]) * tt;
            float py = _spy[i0] + (_spy[i0 + 1] - _spy[i0]) * tt;
            float aa = _spa[i0];
            float ax = -fsin_t(aa), ay = fcos_t(aa);
            px += ax * (-0.08f * Wd); py += ay * (-0.08f * Wd);
            ipt_t d[8];
            for (int v = 0; v < 8; v++) {
                d[v].x = (int32_t)rne_f2i((px + s_oct_u[v][0] * GOLD_HIDOT_R) * 256.0f);
                d[v].y = (int32_t)rne_f2i((py + s_oct_u[v][1] * GOLD_HIDOT_R) * 256.0f);
            }
            fill_poly(d, 8, s_pal[PI_KSCALE], NULL, GOLD_HIDOT_A);
        }
    }

    /* ★★ 第 54 轮：红白鲤的**身体体积**（只给 pat != 1；御黄金走上面那套，互不干扰）
       王总原话：「不要像红白两色的平面纸片。先塑造鱼身体的体积，再绘制红色花纹」
       从**背中央**向**两侧**四档：亮白 → 暖白(底色) → 灰白 → 稍深阴影。

       ⚠️⚠️ 与御黄金同一条路：**只用 fill_poly 的 grad**（沿方向 u 逐像素插值 alpha）。
          第 47 轮"沿 spine 等距偏移的折线多边形"被王总否掉过（"金色鱼背部有条线
          不对劲"）—— 根因是折线的两条边在**弯曲**的鱼身上是斜切过身体的直边，
          叠半透明色就成了**一条线**；而"圆鼓鼓"要的是**明暗过渡**。

       ★ grad 的对称写法（4 个断点做出"中间 0、两侧满"）：
           s ≤ s0 → a0（覆盖外侧整段）· s0→s1 渐降 · s1..s2 恒定 · s2→s3 渐升 ·
           s ≥ s3 → a3（覆盖外侧整段）。
         所以"两侧压暗"= a0/a3 取满、a1/a2 取 0；"背中央窄高光"反过来。
       ⚠️ 满档点必须落在**鱼身内部**：身体半宽 ≈ Wd·KDEPTH[mid] ≈ Wd，
         所以外侧断点取 0.95Wq 而不是 1.15Wq —— 取到 1.15 的话腹侧只吃到
         斜坡的一小截，"腹部压暗"就几乎看不见了。 */
    if (!isGold) {
        int   mid = KSEG / 2;
        float ox  = _spx[mid], oy = _spy[mid];
        float nx  = -fsin_t(_spa[mid]), ny = fcos_t(_spa[mid]);   /* 横向（背↔腹） */
        float tx  =  fcos_t(_spa[mid]), ty = fsin_t(_spa[mid]);   /* 纵向（头→尾） */
        float Wq  = (Wd > 1.0f) ? Wd : 1.0f;
        float Hl  = L * 0.5f;
        grad_t g;
        g.ux = (int32_t)rne_f2i(nx * 256.0f); g.uy = (int32_t)rne_f2i(ny * 256.0f);
        g.cx = (int32_t)rne_f2i(ox * 256.0f); g.cy = (int32_t)rne_f2i(oy * 256.0f);

        /* ① 身体两侧 → 浅灰白 #D9DDD8：|s| 0.70Wd 起满、0.22Wd 归零 */
        g.s0 = (int32_t)(-Wq * 0.70f * 256.0f); g.a0 = KBODY_SIDE_A;
        g.s1 = (int32_t)(-Wq * 0.22f * 256.0f); g.a1 = 0;
        g.s2 = (int32_t)( Wq * 0.22f * 256.0f); g.a2 = 0;
        g.s3 = (int32_t)( Wq * 0.70f * 256.0f); g.a3 = KBODY_SIDE_A;
        fill_poly(s_poly, s_npts, s_pal[PI_KBODY_SIDE], &g, 256);

        /* ② 靠腹 / 更外侧 → 稍深灰 #BFC6C3：|s| 0.95Wd 起满、0.45Wd 归零。
              与 ① 叠起来，腹侧 = 灰白 + 稍深灰 ⇒ 两级下压， outermost 最深。 */
        g.s0 = (int32_t)(-Wq * 0.95f * 256.0f); g.a0 = KBODY_DK_A;
        g.s1 = (int32_t)(-Wq * 0.45f * 256.0f); g.a1 = 0;
        g.s2 = (int32_t)( Wq * 0.45f * 256.0f); g.a2 = 0;
        g.s3 = (int32_t)( Wq * 0.95f * 256.0f); g.a3 = KBODY_DK_A;
        fill_poly(s_poly, s_npts, s_pal[PI_KBODY_DK], &g, 256);

        /* ③ 背中央**非常窄**的柔和高光 → 亮白 #FFFDF5：只在 ±0.12Wd，0.40Wd 渐隐。
              ⚠️ 王总特别强调"非常窄" —— 宽了就是第 41 轮被关掉的那条"脊骨"
                 （「鱼的骨骼显现出来了，需要隐藏」）。0.12Wd ≈ 2px，是柔光不是骨线。 */
        g.s0 = (int32_t)(-Wq * 0.40f * 256.0f); g.a0 = 0;
        g.s1 = (int32_t)(-Wq * 0.12f * 256.0f); g.a1 = KBODY_BACK_A;
        g.s2 = (int32_t)( Wq * 0.12f * 256.0f); g.a2 = KBODY_BACK_A;
        g.s3 = (int32_t)( Wq * 0.40f * 256.0f); g.a3 = 0;
        fill_poly(s_poly, s_npts, s_pal[PI_KBODY_HI], &g, 256);

        /* ④ 尾根压暗 → 稍深灰 #BFC6C3（"使身体和尾巴产生前后层次"）。
              u 取**指尾**方向（切向是头→尾，这里直接用 +tx/+ty）；
              头半段恒 0、从中段开始渐强、尾端满。 */
        g.ux = (int32_t)rne_f2i(tx * 256.0f); g.uy = (int32_t)rne_f2i(ty * 256.0f);
        g.s0 = (int32_t)(-Hl * 1.10f * 256.0f); g.a0 = 0;
        g.s1 = (int32_t)(-Hl * 0.10f * 256.0f); g.a1 = 0;
        g.s2 = (int32_t)( Hl * 0.60f * 256.0f); g.a2 = KBODY_TAIL_A;
        g.s3 = (int32_t)( Hl * 1.10f * 256.0f); g.a3 = KBODY_TAIL_A;
        fill_poly(s_poly, s_npts, s_pal[PI_KBODY_DK], &g, 256);

        /* ⑤ 头顶 1 个小高光（王总："头顶可增加1个小高光"）。
              位置**确定性**（段号推出，不抽 rnd —— 铁律 20）。 */
        {
            float sg = 0.45f;
            int   i0 = 0; float tt = sg;
            float px = _spx[i0] + (_spx[i0 + 1] - _spx[i0]) * tt;
            float py = _spy[i0] + (_spy[i0 + 1] - _spy[i0]) * tt;
            ipt_t d[8];
            for (int v = 0; v < 8; v++) {
                d[v].x = (int32_t)rne_f2i((px + s_oct_u[v][0] * KBODY_HEAD_R) * 256.0f);
                d[v].y = (int32_t)rne_f2i((py + s_oct_u[v][1] * KBODY_HEAD_R) * 256.0f);
            }
            fill_poly(d, 8, s_pal[PI_KBODY_HI], NULL, KBODY_HEAD_A);
        }
    }
    {
        float save[NPTS * 2];
        int n = s_npts;
        for (int i = 0; i < n * 2; i++) save[i] = s_pts[i];
        stroke_pts(save, n, 1, 0.7f, s_pal[PI_KEDGE], 133);  // 0.52
    }
    /* ★★ 第 47 轮加过"金属感背脊暗带"（沿 spine 叠一条 PI_KEDGE 半透明窄带），
       第 48 轮**已整段删除** —— 王总原话"金色鱼背部有条线不对劲 做纯色就行"。
       ⚠️ 记一句机理，免得下次再犯：那条带的两条边是**沿 spine 等距偏移**的折线，
          在弯曲的鱼身上它并不是"贴着背脊"的，而是**斜切过身体**的一条直边；
          再叠上半透明深褐，在橙金底色上就成了一条**明显的直线**（不是渐变）。
          所谓"金属感"要的是**明暗过渡**，而一条硬边折线给的是**轮廓线** —— 观感上
          就是"鱼身上有条线"。真要做金属层次，得用 fill_poly 的 grad（逐像素插值），
          不能用"再叠一个多边形"这种近似。
       ⇒ 金鱼身回到**纯色** fill（与红白同一条路，见下面 ② 段）。 */
    PROF2_TICK(2);                                           /* 2 = ②鱼身 + 体缘暗线 */

    /* ③ 红斑（黄金鲤没有；大正三色另加墨斑）
       ★★ 第 47 轮：14 段椭圆 → **8 顶点径向扰动多边形**。
       王总原话"红斑现在是 ●●● 三个圆点，要改成不规则的 2~3 块色斑"：
       14 段椭圆太圆，看上去就是三粒红点。改成 8 个顶点，每个顶点的径向比 r
       在 [0.70, 1.30] 间起伏，振幅由 cos(ang·2.3 + phase·1.7) 算出来，
       phase = 斑索引 × 1.7 mod 2π → 3 块斑得到 3 种不同形态（确定性、不消费
       rnd，与铁律 20 兼容）。8 顶点比 14 段还便宜（三角函数 14×2 → 8）。
       ⚠️ 报脏没改：AABB 算的是"上帧 AABB ∪ 平移副本"，不关心具体形状，
          r=1.3 偶尔超半轴的像素由 KOI_DIRT_MARGIN=6.0 兜住（台架已 PASS）。 */
    for (int s = 0; s < k->ns; s++) {
        int sg = (int)k->sp[s][0];
        float tt = k->sp[s][1];
        float aa = _spa[sg];
        float ax = _spx[sg] + (_spx[sg + 1] - _spx[sg]) * tt;
        float ay = _spy[sg] + (_spy[sg + 1] - _spy[sg]) * tt;
        float off = k->sp[s][4] * Wd;
        float ph  = (float)s * 1.7f;
        /* ★ 第 51 轮：红斑尺寸整体缩放（王总「红色我需要在鱼身上覆盖的稍微再多点占比」）。
           ⚠️ 只改**尺寸**，不改 k->sp[][] 里那些随机系数 —— 动它们会改随机序列
              （铁律 20），整池鱼的位置/斑的分布全跟着变。
           面积 = π·sl·sw，所以线性放大 1.15 ⇒ 面积 +32%，1.30 ⇒ +69%。 */
        float sl = L * k->sp[s][2] * KOI_SPOT_SCALE;
        float sw = Wd * k->sp[s][3] * KOI_SPOT_SCALE;
        float ca = fcos_t(aa), sa = fsin_t(aa);
        float cax = ax - sa * off, cay = ay + ca * off;
        const int SEGS = KOI_SPOT_SEGS;
        s_npts = SEGS;
        for (int v = 0; v < SEGS; v++) {
            float ang = ph + (float)v * (6.2832f / (float)SEGS);
            /* ★★ 第 54 轮：径向扰动改**双频**（王总："边缘有轻微凹凸"）。
               单频 cos(2.3θ) 只在两个方向上鼓出来，读着还是"橄榄球"；
               叠一个 3.7θ 的高频（权重 0.32）才能出现**细碎的凹凸**，
               像真实绯斑那种不规则的边缘。
               ⚠️ 两个频率**互质**（2.3 / 3.7），否则拍频会让所有斑长一个样。 */
            float r = 1.0f + KOI_SPOT_JIT *
                      (0.68f * fcos_t(ang * 2.3f + ph * 1.7f) +
                       0.32f * fcos_t(ang * 3.7f + ph * 2.9f));
            float rx = fcos_t(ang) * sl * r;
            float ry = fsin_t(ang) * sw * r;
            s_pts[v*2]     = cax + ca * rx - sa * ry;
            s_pts[v*2 + 1] = cay + sa * rx + ca * ry;
        }
        to_q8(s_npts, 0.0f, 0.0f);
        /* ★★ 第 54 轮：红斑**不再是平涂一个色**（王总："不要使用一个颜色。
           红斑应该是不规则自然色块…主体朱红 #E5392D，受光 #F25A45，
           靠身体两侧和边缘 #A92320"）。
           ⇒ 三遍 fill：底（主体朱红）+ 亮（受光）+ 暗（靠两侧/边缘）。
           ⚠️ **两个 grad 用不同的轴**，这是有意的：
             · 亮用**斑自身**的横轴（以斑心为原点、半宽 sw 为单位）
               ⇒ 斑心受光最亮 —— 表现"斑自己是个鼓起来的色块"；
             · 暗用**鱼身**的横轴（以脊线为原点、Wd 为单位）
               ⇒ 长在腹侧的斑整体偏深红 —— 表现"斑贴在曲面上"。
             两条轴叠起来 = 斑既有自己的明暗，又跟着身体的曲面走。
           ⚠️ 边缘的"轻微凹凸、大小不同"由上面的**双频径向扰动**给
              （KOI_SPOT_SEGS 8→11 + JIT 双频），不是靠再叠一圈描边。 */
        fill_poly(s_poly, s_npts, s_pal[PI_KSPOT], NULL, 236);   // 0.92 底：主体朱红

        grad_t gs;
        /* 亮：斑心受光 → 较亮朱红 #F25A45 */
        gs.ux = (int32_t)rne_f2i(-sa * 256.0f);
        gs.uy = (int32_t)rne_f2i( ca * 256.0f);
        gs.cx = (int32_t)rne_f2i(cax * 256.0f);
        gs.cy = (int32_t)rne_f2i(cay * 256.0f);
        gs.s0 = (int32_t)(-0.55f * sw * 256.0f); gs.a0 = 0;
        gs.s1 = (int32_t)(-0.08f * sw * 256.0f); gs.a1 = KSPOT_LT_A;
        gs.s2 = (int32_t)( 0.08f * sw * 256.0f); gs.a2 = KSPOT_LT_A;
        gs.s3 = (int32_t)( 0.55f * sw * 256.0f); gs.a3 = 0;
        fill_poly(s_poly, s_npts, s_pal[PI_KSPOT_LT], &gs, 256);

        /* 暗：斑整体偏在身体哪一侧 → 靠两侧就压深红 #A92320。
           ⚠️ 轴换成**鱼身横轴**（以脊线中段为原点），所以这里要重算 cx/cy。 */
        {
            int   mid2 = KSEG / 2;
            float nx2 = -fsin_t(_spa[mid2]), ny2 = fcos_t(_spa[mid2]);
            float Wq2 = (Wd > 1.0f) ? Wd : 1.0f;
            gs.ux = (int32_t)rne_f2i(nx2 * 256.0f);
            gs.uy = (int32_t)rne_f2i(ny2 * 256.0f);
            gs.cx = (int32_t)rne_f2i(_spx[mid2] * 256.0f);
            gs.cy = (int32_t)rne_f2i(_spy[mid2] * 256.0f);
            gs.s0 = (int32_t)(-Wq2 * 0.92f * 256.0f); gs.a0 = KSPOT_DK_A;
            gs.s1 = (int32_t)(-Wq2 * 0.30f * 256.0f); gs.a1 = 0;
            gs.s2 = (int32_t)( Wq2 * 0.30f * 256.0f); gs.a2 = 0;
            gs.s3 = (int32_t)( Wq2 * 0.92f * 256.0f); gs.a3 = KSPOT_DK_A;
            fill_poly(s_poly, s_npts, s_pal[PI_KSPOT_DK], &gs, 256);
        }
    }
    if (k->pat == 2) {
        static const uint8_t ink[3] = {26, 24, 30};
        /* 大正三色墨斑同形态 —— phase 给 0.85 让它和红斑错开 */
        float ph = 0.85f;
        float ca = fcos_t(_spa[2]), sa = fsin_t(_spa[2]);
        float sl = L * 0.055f, sw = Wd * 0.20f;
        const int SEGS = KOI_SPOT_SEGS;
        s_npts = SEGS;
        for (int v = 0; v < SEGS; v++) {
            float ang = ph + (float)v * (6.2832f / (float)SEGS);
            float r = 1.0f + KOI_SPOT_JIT * fcos_t(ang * 2.3f + ph * 1.7f);
            float rx = fcos_t(ang) * sl * r;
            float ry = fsin_t(ang) * sw * r;
            s_pts[v*2]     = _spx[2] + ca * rx - sa * ry;
            s_pts[v*2 + 1] = _spy[2] + sa * rx + ca * ry;
        }
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
        /* ★★ 第 44 轮：这里曾按王总的规格「尾鳍 7×9 px」把尾鳍放大
           （tl 0.238L→0.365L、tw 从挂在 L 上改成挂在 Wd 上，好让"尾鳍/体宽"
           不随体宽系数变）。他看过尺寸阶梯后说「鱼尺寸都不想要 还回归咱们自己的鱼」，
           已**逐字撤回** —— 注意 tw 必须回到 `L*0.094`（不是等价的 Wd*0.537），
           因为 KOI_WD 已经回到 0.175，两者虽数学等价但浮点路径不同，不是逐位相同。
           下一轮若真要动尾鳍，先跑 _tools/koimeasure44.py 把绝对像素算出来。 */
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
            /* ★★ 第 53 轮：御黄金尾鳍「金黄 → 半透明浅金」。
               红白**一个字节都不动**（仍走下面四条分支里的 TJ_GA）。
               ⚠️ 只加在 `fine` 那条路上（细档鱼才画渐变尾鳍），
                  下面三条 flat 分支保持原样 —— 少动就是少风险。 */
            if (isGold && fine) {
                grad_t g2 = g;                       /* 同方向 u、同原点 c */
                g.a2  = GOLD_TJ_A1;                  /* 金黄：中段开始让位 */
                g.a3  = GOLD_TJ_A2;                  /* 金黄：梢部只剩一点 */
                g2.s0 = 0;          g2.a0 = 0;
                g2.s1 = hold * 256; g2.a1 = 0;
                g2.s2 = mid * 256;  g2.a2 = GOLD_TJ_LT1;
                g2.s3 = nend * 256; g2.a3 = GOLD_TJ_LT2;
                fill_poly(s_poly, n, bodyCol, &g, 256);                 /* 金黄层 */
                fill_poly(s_poly, n, s_pal[PI_KGOLD_TL], &g2, 256);     /* 浅金层 */
            }
            /* ★ 第 39 轮**不动这里**。曾试过把档 2 提到最前当"鱼要实体感"的解，
               但那样等于**擅自改立度**（档 0 的 21° → 档 2 的 30°），超出"就做这两个事"。
               本轮"实体感"由 fill_poly 的根因修复解决（鱼身 α 0.75 → 1.00）。
               遗留（**下一轮单独做**）：`s_look_tail >= 2` 排在 `if (fine)` 之后，
               而 fine = (grow > 0.56)，开局 6 条鱼 grow 0.56~0.68 ⇒ **档 2 从没覆盖过
               细档鱼的渐变尾鳍**（TJ_GA = 256/179/108，尾梢只剩 0.42）—— 真缺陷。 */
            else if (fine) fill_poly(s_poly, n, bodyCol, &g, 256);
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
    /* ⑥ 胸鳍（左右各一）。原 if (fine) —— grow > 0.56 才画，本轮去掉这层限制：
       王总要"初始的所有鱼不管大小都需要有左右的鱼鳍"。几何尺寸都按 Wd/L 比例
       算（r0 = Wd * KDEPTH[1] * 0.66f、pl = L * (0.15 + 0.09 * pad)），
       小鱼自动按比例缩，没问题。⚠️ 性能：每帧多画 2 个 fill_poly（10 个 pt_quad），
       真机 8 条鱼多花 ~3ms（详见 commit message 的预算面板）。 */
    {
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
    if (k >= s_koi && k < s_koi + MAX_KOI && !s_full) {
        /* ★★ 第 46 轮：**整屏重画帧不计**。
           为什么：这条判据问的是"这一帧会不会留下上一帧的残影"，而整屏重画
           （s_full）把 76800 个像素全写一遍 —— 鱼脏盒**根本没被用到**，不可能有残影。
           实测证据：扫 KOI_FRAMEMS 时 120/142/160/200ms 下的最大值都出现在
           **步 5~6（刚进池的头几帧，正是整屏重画）**，原始欠缺 12~15px，
           而同期金标准两条判据（脏区渲染 / 脏区覆盖）全 PASS。
           把整屏帧算进来 = 用一个**用不到的盒子**去否定一个**没被使用的机制**，
           读出来的数还会把稳态的真实读数（6~7px）整个盖住 —— 假 FAIL 比 FAIL 更坏。
           ⚠️ 判据本身没变松：稳态（真正走脏区）的欠缺照样逐帧取最大值。 */
        float d0 = k->bx0 - k->ax0, d1 = k->ax1 - k->bx1;
        float d2 = k->by0 - k->ay0, d3 = k->ay1 - k->by1;
        if (s_rtrace_from <= s_step_no && s_step_no <= s_rtrace_to) {
            printf("    [bs 步%d 鱼%d] 报盒 x[%.1f..%.1f] y[%.1f..%.1f] | "
                   "真AABB x[%.1f..%.1f] y[%.1f..%.1f] | 缺 %.2f/%.2f/%.2f/%.2f "
                   "grow=%.3f v=%.2f curv=%.3f headA=%.3f full=%d\n",
                   s_step_no, (int)(k - s_koi),
                   k->bx0, k->bx1, k->by0, k->by1,
                   k->ax0, k->ax1, k->ay0, k->ay1,
                   d0, d1, d2, d3, k->grow, k->v, k->curv, k->headA, s_full);
        }
        /* ★★ 第 46 轮：记下"最大欠缺"的现场（帧号 / 鱼号 / 哪一侧 / 当时的弯曲与体量），
           否则只有一个数，看不出是"形变随 dt 变大"还是"某个事件推了一把"。 */
        float dd[4] = {d0, d1, d2, d3};
        for (int s = 0; s < 4; s++) {
            if (dd[s] > s_bbox_short) {
                s_bbox_short = dd[s];
                s_bs_step = s_step_no;
                s_bs_koi  = (int)(k - s_koi);
                s_bs_side = s;
                s_bs_raw  = s_bbox_short + KOI_DIRT_MARGIN;   /* 未扣 margin 的原始欠缺 */
                s_bs_grow = k->grow;
            }
        }
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
/* ★ 拍水波列的寿命（秒）。历任取值：
     0.82  网页版原值（RING_LIFE）
     0.40  第 43 轮 —— 王总原话「拍水后水花需要速度快点 散去」
     0.55  ★ 第 50 轮现值 —— 王总改口「拍水按完之后波纹有点太快了 稍微慢一点点」
     0.70  第 50 轮阶梯里更慢的一档（想再慢就换这个）
   ★ 真机实测（ripchk50.py，帧长 143ms = 真机 v13 的 7fps）：
       0.40 → 波纹只存活 **2 帧**（0.29s）  ← 王总觉得"太快"的就是这个
       0.55 → **3 帧**（0.43s）             ← ★ 现值
       0.70 → **4 帧**（0.57s）
     ⚠️ 注意"秒"在这里是个误导单位：真机 7fps 下 0.4 秒只有 2~3 帧，
        观感由**帧数**决定，不是由秒决定 —— 调这个值要看着帧数调。
   ⚠️ 第 50 轮只动这一个数（王总：「只改波纹 不要改鱼游动」）——
      RING_RMAX_V(46) / RING_N_V(3) / RING_GAP_V(13) 都不动，只调快慢。
   ⚠️ 包成 #ifndef 是为了台架能用 -D 扫档（裸 #define 会被 -D 覆盖后又改回来）。
      真机上不传 -D 就是下面这个默认值。 */
#ifndef RING_LIFE_V
#define RING_LIFE_V 0.55f
#endif
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
#define FEED_N      9          /* 一次投喂真正产食物的饲料颗数（王总原话"保持在 8~9 粒"） */
#define DROP_N      7          /* ★ 第 43 轮：4→7（王总要"雨点那 7 颗设计留'下雨感'"） */
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
    /* ★ 第 51 轮：落水的食物**默认不画**（KOI_FOOD_DRAW=0）。
       王总「停留加回来 但是显示是隐藏的」—— 它只作为鱼的追踪/进食目标存在，
       屏上看不见。想 A/B 对照"看得见的版本"就 -DKOI_FOOD_DRAW=1 编一版。 */
#if KOI_FOOD_DRAW
    for (int i = 0; i < s_nfood; i++) {                    // 已落定的食物
        ellipse_pts(s_food_x[i], s_food_y[i], 1.6f, 1.6f, 0.0f, 8);
        to_q8(s_npts, 0.0f, 0.0f);
        fill_poly(s_poly, s_npts, col, NULL, 256);
    }
#endif
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
        /* ★ 第 50 轮：撒点范围 ±40 → **±28**，飞行时长 0.34~0.52s → **0.70~1.00s**。
           为什么：真机 7fps（143ms/帧）下，鱼巡游 ~27px/s = **3.9px/帧**。
             旧参数：饲料只飞 0.4s ≈ **3 帧** ⇒ 鱼最多挪 12px，而饲料撒在 ±40px 处
                     ⇒ 追不上。ripchk50.py 实测 45 帧里 seek 只占 13%、eat 涟漪 0 次。
             新参数：飞行 ≈ 1s ≈ **7 帧** ⇒ 鱼能挪 27px，落点在 ±28px 内
                     ⇒ 大概率够得到，而且"鱼朝饲料冲"这个过程有 7 帧可看。
           ⚠️ 王总没说要改饲料速度，但"鱼要往饲料游"这件事在 3 帧里**物理上做不到**，
              不改这两个数就只能得到"鱼没反应"。飞行慢一点也更像"撒饲料飘落"。 */
        float tx = near ? clampf(near->x + rnd_f(-28, 28), 16, KW - 16) : rnd_f(16, KW - 16);
        float ty = near ? clampf(near->y + rnd_f(-28, 28), 20, KH - 28) : rnd_f(20, KH - 28);
        safe_spot(tx, ty); tx = s_spotX; ty = s_spotY;    // 别撒到荷叶上（撒了看不见也吃不到）
        pel_t *pe = &s_pel[s_npel++];
        pe->sx = tx + rnd_f(-14, 14); pe->sy = ty - rnd_f(46, 80);
        pe->tx = tx; pe->ty = ty;
        pe->x = pe->sx; pe->y = pe->sy;
        pe->t = 0; pe->dur = rnd_f(0.70f, 1.00f); pe->delay = rnd_f(0, 0.55f);
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

/* ==========================================================================
   12a. ★★ 第 45 轮：鱼可游区域 Safe Swim Polygon
   --------------------------------------------------------------------------
   王总第 45 轮原话：
     「鱼可游区域 Safe Swim Polygon：(42, 20) (198, 20) (211, 48) (216, 90)
       (213, 145) (218, 205) (210, 263) (192, 298) (48, 298) (29, 267)
       (23, 220) (27, 170) (23, 115) (29, 62)
       我给到你的数据是鱼可有用的区域」

   这 14 个点是**王总直接给的数值**，不是从图上描的 —— 所以它是最权威口径，
   下面 SWIM_PT[] 与 _tools/swimpoly45.py 的 POLY **逐点一致**（那边是唯一真源）。
   坐标口径已核对为 **240x320 屏幕像素**：关于 x=120 近似镜像（240-42=198、
   240-29=211、240-48=192），y 范围 20..298（距上边 20px、下边 22px）。

   ■ 为什么不用"矩形回避带"（第 38~44 轮那套 m2 + clampf）
     旧写法是四条轴向软推力 + 一个 m2 = 26+kh 的回避带，再钳到 0.40*KW。
     它是**矩形**的：池子四角是圆的（石头堆出来的），矩形在角上会把鱼往石头里推；
     而且 m2 钳到 96 之后左右推力在 240px 宽的屏上方向打架，鱼会原地抖
     （第 38 轮为这个打过一次补丁）。王总这次直接把安全区画出来了，就该用他的。

   ■ ★ 这个多边形**不是凸的**（量过：叉积符号集 = {+1, -1}）
     右缘 (216,90)→(213,145)→(218,205) 和左缘 (23,115)→(27,170)→(23,220)
     各有一个 4~5px 的内凹。所以"逐边半平面推回"不能直接套 —— 它对非凸多边形
     不收敛。验证过的替代方案（见 _tools/swimpoly45.py 的 halfplane_region）：
       **取 14 条边各自的内向半平面求交**。交集天然是凸的，而且实测它
       ⊆ 原多边形（数值扫描 194,615 个采样点，落在多边形外的 291 个点
         全部位于边界线上，是射线法在边界上的固有误判），
       代价是两处内凹被切掉 —— 最大收缩 5.5px，而鱼的边界余量是 54px，
       肉眼完全看不出来。**保守方向**（切掉的是多边形内部那一侧，
       绝不会把鱼放到石头上），可以接受。
     ⚠️ 王总若改了这 14 个点，必须重跑 swimpoly45.py 重新验证
        "半平面交 ⊆ 多边形"与"内缩后非空"这两条，别想当然。
     ⚠️ 别为了"用回凸算法"去取凸包 —— 凸包比原多边形**大**，
        会把鱼放进被王总划掉的那两个凹口里。

   ■ 为什么用"逐边投影迭代"而不是"算最近边界点再推"
     交集是凸的 ⇒ 逐边投影（P ⟵ P − d·n，d<0 时）几轮就收敛，只用乘加，无 sqrt；
     而"最近边界点"要先遍历 14 条边算距离再开方，还得分内外，贵且难写对。
     实测 3 轮足够（凸集上每轮至少消掉一条违规边）。 */
#define SWIM_N 14

/* ★ 王总给的 14 点，原文照抄、一个数没动。顺序 = 上边左→右、右缘自上而下、
   下边右→左、左缘自下而上（屏幕坐标下是顺时针）。 */
static const float SWIM_PT[SWIM_N][2] = {
    { 42.0f,  20.0f}, {198.0f,  20.0f}, {211.0f,  48.0f}, {216.0f,  90.0f},
    {213.0f, 145.0f}, {218.0f, 205.0f}, {210.0f, 263.0f}, {192.0f, 298.0f},
    { 48.0f, 298.0f}, { 29.0f, 267.0f}, { 23.0f, 220.0f}, { 27.0f, 170.0f},
    { 23.0f, 115.0f}, { 29.0f,  62.0f},
};

/* 每条边的**内向**单位法线，以及多边形外接框。都是常数，开机建一次。 */
static float SWIM_NRM[SWIM_N][2];
static float SWIM_BB[4];                 /* x0, y0, x1, y1 —— 外接框 */
static int   SWIM_ready = 0;

static void swim_init(void)
{
    if (SWIM_ready) return;
    SWIM_ready = 1;

    SWIM_BB[0] = SWIM_BB[1] =  1e9f;
    SWIM_BB[2] = SWIM_BB[3] = -1e9f;
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < SWIM_N; i++) {
        float x = SWIM_PT[i][0], y = SWIM_PT[i][1];
        cx += x; cy += y;
        if (x < SWIM_BB[0]) SWIM_BB[0] = x;
        if (y < SWIM_BB[1]) SWIM_BB[1] = y;
        if (x > SWIM_BB[2]) SWIM_BB[2] = x;
        if (y > SWIM_BB[3]) SWIM_BB[3] = y;
    }
    cx /= (float)SWIM_N; cy /= (float)SWIM_N;

    /* 内向法线：先取边的一个法线，再用重心（一定在内部）判该不该翻向。
       多边形关于重心是星形的（王总这 14 点如此），所以这个判法够用。 */
    for (int i = 0; i < SWIM_N; i++) {
        const float *a = SWIM_PT[i], *b = SWIM_PT[(i + 1) % SWIM_N];
        float dx = b[0] - a[0], dy = b[1] - a[1];
        float nx =  dy, ny = -dx;
        float L = sqrtf(nx * nx + ny * ny);
        if (L < 1e-6f) { SWIM_NRM[i][0] = 0.0f; SWIM_NRM[i][1] = 0.0f; continue; }
        nx /= L; ny /= L;
        if ((cx - a[0]) * nx + (cy - a[1]) * ny < 0.0f) { nx = -nx; ny = -ny; }
        SWIM_NRM[i][0] = nx; SWIM_NRM[i][1] = ny;
    }
}

/* 把点 (*px,*py) 推回"多边形内缩 m 像素"的区域。返回 1 = 真的推过。
   ⚠️ 只在**凸**的交集上成立；非凸多边形要先用 halfplane 求交（本文件上面那套）。
   ⚠️ 轮数不是拍脑袋的：台架第 45 轮实测（demo_koi_swim_worst，第三条不变量）
        3 轮 → 残差 0.67px（有鱼在角上擦出界 0.67px，亚像素，肉眼看不到但不达标）
        6 轮 → 残差 ≤0（见 _tools/_host45c.log 的「安全区最大越界」读数）
      **别往回调到 3** —— "凸集上 3 轮就够"是估计，不是量出来的，实测不够。
      代价几乎为零：绝大多数调用第一轮就没有违规、直接 break，
      只有真出界的那几次才会跑满 6 轮（6×14 次乘加）。
   ⚠️ 别改成"推一轮就返回"：一轮只消掉当前最违规的那条边，角上要好几轮才收敛。 */
static int swim_push(float *px, float *py, float m)
{
    swim_init();
    float x = *px, y = *py;
    int moved = 0;
    for (int it = 0; it < 6; it++) {
        int hit = 0;
        for (int i = 0; i < SWIM_N; i++) {
            const float *a = SWIM_PT[i], *n = SWIM_NRM[i];
            float d = (x - a[0]) * n[0] + (y - a[1]) * n[1] - m;
            if (d < 0.0f) { x -= d * n[0]; y -= d * n[1]; hit = 1; moved = 1; }
        }
        if (!hit) break;
    }
    if (moved) { *px = x; *py = y; }
    return moved;
}

static void koi_step(koi_t *k, float dt)
{
    float kh = k->L * k->grow * 0.55f;

    float tx = 0, ty = 0;
    int ti = -1;      /* 飞行中的目标在 s_pel 的下标，-1 = 无 */
    int fi = -1;      /* ★ 第 51 轮：落水后的目标在 s_food 的下标，-1 = 无 */
    float best = 1e9f;
    /* ★ 第 43 轮：找食目标从 s_food（落地后的颗粒）改为 s_pel（飞行中的颗粒）。
       王总原话"喂食后不需要把食物留停留在水池 其他不落池"——
       落水那一帧只 ripple_add 不再入 s_food，鱼要吃就在**飞行途中**接住。
    ★★ 第 50 轮补两条（王总：「鱼应该往饲料方向游动 而不是现在的没感觉」）：
       ① **追落点，不追空中当前位置**（tx/ty 而不是 pe->x/pe->y）。
          旧写法鱼追的是"饲料这一帧飘到哪儿"，那是个**移动目标**——
          鱼朝它游，它也在往下掉，鱼永远在追屁股，看着就像"没在追"。
          落点是固定的，鱼冲过去才是王总说的"往饲料方向游动"。
       ② **delay>0 的颗粒也算目标**（旧写法 `pe->delay > 0 → continue`）。
          delay 是 0~0.55s 的"还没落下来"等待期，这段时间鱼**干等着不动**，
          等它开始落，飞行只剩 dur 那么点时间 —— 真机 7fps 下根本来不及。
          让它提前出发，可用的时间 = delay + dur ≈ 1.3s（旧写法只有 dur ≈ 0.4s）。
       ⚠️ 吃食判定**仍然要求 delay ≤ 0**（见下）：可以提前往那儿游，
          但不能在饲料还没出现时就"吃到"。 */
    for (int m = 0; m < s_npel; m++) {
        const pel_t *pe = &s_pel[m];
        if (!pe->food) continue;                     /* 不捕雨点 */
        float dx = pe->tx - k->x, dy = pe->ty - k->y;
        float dd = dx * dx + dy * dy;
        if (dd < best) { best = dd; ti = m; fi = -1; tx = pe->tx; ty = pe->ty; }
    }
    /* ★★ 第 51 轮：**落水后的颗粒也当目标**（s_food）。
       王总：「饲料进水 鱼应该随机往饲料方向游动 这样就有交互感了」
             「停留加回来 但是显示是隐藏的」
       ⇒ 饲料落水后**继续存在**（鱼有东西可追、可吃），但**屏幕上不画**
         （pellets_draw 里那段被 KOI_FOOD_DRAW 关掉了）。
         这样既有"鱼冲过去吃"的交互，又不会看到池底积一堆饲料 ——
         第 43 轮「其他不落池」那条要求保住了。
       ⚠️ 不加这一段的后果：饲料落水即消失，鱼游到落点时那儿已经空了，
         于是"追了半天啥也没吃到" —— 就是王总说的"碰到几率很小"。 */
    for (int m = 0; m < s_nfood; m++) {
        float dx = s_food_x[m] - k->x, dy = s_food_y[m] - k->y;
        float dd = dx * dx + dy * dy;
        if (dd < best) { best = dd; ti = -1; fi = m; tx = s_food_x[m]; ty = s_food_y[m]; }
    }
    if (best > 340.0f * 340.0f) { ti = -1; fi = -1; }    // 感知半径 ≈ 全屏
    k->seek = (ti >= 0 || fi >= 0);

    float vx = fcos_t(k->headA), vy = fsin_t(k->headA);  // 前进惯性项，不能省
    if (ti >= 0) {
        float d = hypotf(tx - k->x, ty - k->y);
        if (d < 1.0f) d = 1.0f;
        vx += (tx - k->x) / d * 2.2f;
        vy += (ty - k->y) / d * 2.2f;
    } else {
        k->wanderT -= dt;
        if (k->wanderT <= 0) {
            /* ★ 第 46 轮：换目标间隔 ÷KOI_LV_WAND（越大越勤换 = 越活泼）。 */
            k->wanderT = rnd_f(3.4f, 6.4f) / KOI_LV_WAND;
            for (int tr = 0; tr < 8; tr++) {
                /* ★★ 第 45 轮：漫游目标从"以屏幕中心为心的椭圆"改成"安全区里的随机点"。
                   旧椭圆 (KW/2−44−kh, KH/2−54−kh) 是**矩形回避带**时代的残留：
                   池子四角是圆的（石头堆的），椭圆在角上照样能把目标甩到石头里。
                   新做法三步，全部只用乘加：
                     ① 在安全区**外接框**里均匀取一点；
                     ② swim_push 把它拉进安全区（交集是凸的 ⇒ 3 轮投影必收敛）；
                     ③ 与当前位置做一次随机**凸组合** —— 凸集内的凸组合仍在集合内，
                        所以目标点 100% 落在安全区里；而且不会老贴在边界上。
                   ⚠️ 步骤③不能省。省了的话：②的结果一定落在**内缩后的边界上**，
                      于是目标永远是"贴着某条边的一个点"，鱼就变成朝最近的边界冲刺、
                      到边掉头 —— 看起来像在池子里来回弹，而不是在游。 */
                float txx = SWIM_BB[0] + rnd_f(0.0f, 1.0f) * (SWIM_BB[2] - SWIM_BB[0]);
                float tyy = SWIM_BB[1] + rnd_f(0.0f, 1.0f) * (SWIM_BB[3] - SWIM_BB[1]);
                swim_push(&txx, &tyy, kh + 3.0f);
                float tt = rnd_f(0.30f, 1.0f);
                k->wx = k->x + (txx - k->x) * tt;
                k->wy = k->y + (tyy - k->y) * tt;
                if (hypotf(k->wx - k->x, k->wy - k->y) >= 48.0f &&
                    fabsf(angdiff(atan2f(k->wy - k->y, k->wx - k->x), k->headA)) <= 1.92f) break;
            }
        }
        float wdx = k->wx - k->x, wdy = k->wy - k->y;
        float wdd = hypotf(wdx, wdy);
        if (wdd < 36.0f) k->wanderT = 0.0f;              // 到点换目标，别在原点绕圈
        if (wdd < 0.6f) { wdx = 1.0f; wdy = 0.0f; wdd = 1.0f; }
        /* ★ 第 46 轮：漫游推力 ×KOI_LV_WAND。
           1.05 是相对"前进惯性项 fcos(headA)（模长 1.0）"的比值 ——
           它决定"想去哪"能在多大程度上压过"现在朝哪"。调大 = 更愿意拐弯。 */
        vx += wdx / wdd * 1.05f * KOI_LV_WAND;
        vy += wdy / wdd * 1.05f * KOI_LV_WAND;
    }

    if (s_shakeT > 0) {                                   // 受惊四散
        float sd = hypotf(k->x - s_shakeX, k->y - s_shakeY);
        if (sd < 1.0f) sd = 1.0f;
        vx += (k->x - s_shakeX) / sd * 3.0f;
        vy += (k->y - s_shakeY) / sd * 3.0f;
    }

    /* ★★ 第 45 轮：边界回避带（矩形 m2 + 四条轴向推力）→ 安全区多边形的**软推**。
       做法与下面那次硬约束**共用同一个 swim_push**：把"位置"推回区内，
       推回向量就是转向力（越界越远、推得越狠，天然等价于旧的 m2 比例推力）。
       ⚠️ 与硬约束共用同一个函数是刻意的：两边各写一套的话，迟早出现
          "软推认 A 区、硬夹认 B 区"，表现是鱼贴着某条边原地高频抖。
       ⚠️ 系数 3.2 沿用旧值不动 —— 它是第 38 轮量出来的（太小顶不住 seek 的 2.2
          冲刺，太大会让鱼在边界上抖），没有理由跟着形状一起改。 */
    {
        float bx = k->x, by = k->y;
        if (swim_push(&bx, &by, kh + 3.0f)) {
            float ox = bx - k->x, oy = by - k->y;
            float od = hypotf(ox, oy);
            if (od > 0.6f) { vx += ox / od * 3.2f; vy += oy / od * 3.2f; }
        }
    }

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
    /* ★ 第 46 轮：KOI_LV_TURN 同时乘在"响应速度 4.2"与"转向速率 2.9"上 ——
       只改拐弯的**快慢**，不改拐弯的**形状**（形状由 powf 1.7 / 死区 2.60 / 1.48 定）。
       只乘其中一个会把"转头曲线"掰变形（要么起步拖、要么过冲抖）。 */
    float kk = dt * 4.2f * KOI_LV_TURN; if (kk > 1.0f) kk = 1.0f;
    k->curv += (shaped * 0.40f - k->curv) * kk;
    k->headA += k->curv * 2.9f * KOI_LV_TURN * dt;

    k->gaitTime -= dt;                                    // 步态 burst / coast
    if (k->gaitTime <= 0) {
        /* ★ 第 46 轮：KOI_LV_GAIT —— burst 时长 ×LV、coast 时长 ÷LV。
           原值 burst 0.30~0.55s / coast 0.70~1.50s ⇒ **滑行占了约 70% 的时间**，
           这就是王总说的"很飘逸"：大部分时候鱼在滑、不是在游。
           ⚠️ 除的是**结果**不是 rnd_f 的参数 —— 随机数调用次数与顺序必须保持不变，
              否则整条随机序列平移，荷叶/波光点全跟着变（第 20 轮踩过）。 */
        if (k->burst) { k->burst = 0; k->gaitTime = rnd_f(0.70f, 1.50f) / KOI_LV_GAIT; }
        else          { k->burst = 1; k->gaitTime = rnd_f(0.30f, 0.55f) * KOI_LV_GAIT; }
    }
    int power = k->seek || s_shakeT > 0.0f || fabsf(err) > 1.20f;
    int burst = power || k->burst;
    float ampT = k->seek ? 0.50f : (burst ? 0.38f : 0.24f);
    float wa_t = dt * (burst ? 5.0f : 2.6f); if (wa_t > 1.0f) wa_t = 1.0f;
    k->waveAmp += (ampT - k->waveAmp) * wa_t;
    /* ★ 第 46 轮：摆尾频率 ×KOI_LV_HZ。
       ⚠️ 上限有物理约束：真机 tick 只有 ~8Hz，而摆尾是**振荡**——
          超过 ~3.5Hz 就会因为采样不足而变成"闪"而不是"摆"（Nyquist 那一套）。
          所以这个档位别往 2 倍以上加，真要更活就加 KOI_LV_SPD / KOI_LV_WAND。 */
    float hzT = (k->seek ? 3.0f : (burst ? 2.4f : 1.5f)) * KOI_LV_HZ;
    float hz_t = dt * 3.6f; if (hz_t > 1.0f) hz_t = 1.0f;
    k->hz += (hzT - k->hz) * hz_t;
    k->phase += dt * k->hz * 6.2832f;
    if (!burst) k->phase += dt * 0.6f;                    // 滑行时相位继续推进

    float align = 0.5f + 0.5f * fcos_t(err);
    int biting = (k->biteT > 0);
    /* ★ 第 46 轮：巡游/抢食速度 ×KOI_LV_SPD。 */
    float vT = (k->seek ? 24.0f : (s_shakeT > 0 ? 26.0f : 19.0f))
             * (0.62f + 0.46f * k->grow)
             * (0.86f + 0.28f * s_satiety)
             * (0.42f + 0.58f * align) * KOI_SCALE * KOI_LV_SPD
             * (biting ? BITE_SLOW : 1.0f);
    if (burst) {
        float vk = dt * (biting ? 10.0f : 3.8f); if (vk > 1.0f) vk = 1.0f;
        k->v += (vT - k->v) * vk;
    } else {
        float vk = dt * 0.72f; if (vk > 1.0f) vk = 1.0f;
        k->v -= k->v * vk;
    }
    /* ★ 第 46 轮：上限也必须跟着 KOI_LV_SPD 抬 —— 否则提速度档时会先撞上限，
       表现是"档位调大了但速度没变"（一个很容易看漏的哑档）。
       量过：vT 名义值 54px/s × 1.9 就会顶到原来的 34×3=102 上限。 */
    k->v = clampf(k->v, 0.0f, 34.0f * KOI_SCALE * KOI_LV_SPD);
    if (biting) k->biteT -= dt;

    /* ★★ 第 45 轮：硬约束从"矩形 clampf"换成"安全区多边形推回"。
       clampf 是**矩形**的，而池子四角是圆的（石头堆出来的）—— 鱼在角上会被夹进
       石头里，这正是王总这次把安全区画出来的原因。swim_push 只在越界时才动，
       界内时连一次赋值都没有，所以"鱼本来就在区里"的那些帧**逐位不变**。
       ⚠️ 别再退回 clampf：那等于把安全区当矩形用，四个角全废。 */
    k->x += fcos_t(k->headA) * k->v * dt;
    k->y += fsin_t(k->headA) * k->v * dt;
    swim_push(&k->x, &k->y, kh + 3.0f);

    /* 吃食：判定点 = 吻端 → 吃食圆也落在嘴上（第 18 轮口径）。
       ★★ 第 51 轮：目标**两类都吃**
         ① s_pel[ti] —— 飞行中的颗粒（第 43/50 轮口径，鱼在空中接住）
         ② s_food[fi] —— **落水后漂在水面**的颗粒（第 41 轮口径，本轮加回来）
       ★★ 判定半径也放大（+3.0 → KOI_EAT_REACH 默认 **10.0**）：
         旧值 = L·grow·0.0805 + 3.0 ≈ **4.9px**（L≈40、grow≈0.6），
         而真机 7fps 下鱼一帧就走 **3.9px** —— 判定窗口比步长还窄，
         鱼很容易"擦着饲料游过去"却不算吃到，这正是王总说的
         「鱼碰到鱼食的几率很小」。
         中间试过 6.0（≈7.9px，约等于两帧步长）—— 命中仍然偏少；
         最终定 **10.0**（≈11.9px，约三帧步长）⇒ 一次投喂 9 颗吃到 2~3 颗，
         正好落在王总「命中率不要太百分百 也不要太太低」那档。 */
    float mx = k->x + fcos_t(k->headA) * k->L * k->grow * KMOUTH;
    float my = k->y + fsin_t(k->headA) * k->L * k->grow * KMOUTH;
    float eatR = k->L * k->grow * 0.0805f + KOI_EAT_REACH;
    int got = 0;
    if (ti >= 0 && ti < s_npel && s_pel[ti].food && s_pel[ti].delay <= 0 &&
        hypotf(tx - mx, ty - my) < eatR) {
        /* 从 s_pel 把这颗饲料挖掉（飞行中的颗粒直接消失，不留底）；
           报脏按颗粒当前位置 ≈ (pe->x, pe->y)，下一帧水色把它盖掉。 */
        dirty_add_ext(floor_f2i(s_pel[ti].x) - 3, floor_f2i(s_pel[ti].y) - 3,
                      (int)ceilf(s_pel[ti].x) + 4, (int)ceilf(s_pel[ti].y) + 4);
        s_pel[ti] = s_pel[--s_npel];
        got = 1;
    } else if (fi >= 0 && fi < s_nfood && hypotf(tx - mx, ty - my) < eatR) {
        /* 吃掉水里漂着的那颗：按索引删（尾部搬上来，与 s_pel 同一套）。
           ⚠️ 三个数组要**一起搬**，只搬 x/y 会把 age 张冠李戴，
           于是某颗饲料会"刚落水就到 2 秒"被提前清掉。 */
        dirty_add_ext(floor_f2i(s_food_x[fi]) - 4, floor_f2i(s_food_y[fi]) - 4,
                      (int)ceilf(s_food_x[fi]) + 5, (int)ceilf(s_food_y[fi]) + 5);
        s_food_x[fi]   = s_food_x[s_nfood - 1];
        s_food_y[fi]   = s_food_y[s_nfood - 1];
        s_food_age[fi] = s_food_age[s_nfood - 1];
        s_nfood--;
        got = 1;
    }
    if (got) {
        s_satiety = clampf(s_satiety + 0.05f, 0.0f, 1.0f);
        k->eat = 0.6f;
        k->biteT = BITE_T;                                // 啄食停顿：圆灭之前嘴不离开圆
        k->grow = (k->grow + GROW_PER_PELLET > GROW_MAX) ? GROW_MAX
                                                         : k->grow + GROW_PER_PELLET;
        /* ★ 第 50 轮：rMax / life 改用 EAT_RING_RMAX / EAT_RING_LIFE 两个独立宏
           （原来写死 9 和 BITE_T）。理由见那两个宏上面的长注释：
           —— 帧率 7fps 下 0.42s 的涟漪只画得到 2.9 帧，等于没有特效。
           ⚠️ 别把 life 改回 BITE_T：那是啄食停顿，改它会让鱼吃完停 1 秒。 */
        if (splash_ok(2, mx, my, 3))
            ripple_add(mx, my, 1, EAT_RING_RMAX, EAT_RING_LIFE, 0.70f, 2, 1, 0);
    }

    /* ★★ 第 45 轮：吃食会让 k->grow 变大 ⇒ 安全区余量 kh = L*grow*0.55 也跟着变大。
       上面那次 swim_push 用的是**吃之前**的 kh，所以这里必须按**吃之后**的 kh 再推一次。
       量出来的漏子：GROW_PER_PELLET(0.018) × L_max(69) × 0.55 = **0.68px**，
       正好等于台架第三条不变量「安全区最大越界」报的 0.67px —— 不是迭代没收敛
       （3 轮和 6 轮读数一模一样就是证据），是余量在推完之后又变了。
       ⚠️ 别再想"把上面那次 push 挪到这儿"：上面那次必须留在**吃食判定之前**，
          因为吃食判定用的是推正之后的嘴位置（mx/my）—— 挪下来会让"鱼隔着石头吃到食"。
       ⚠️ 凡是"会改变 kh / k->L 的语句"，都要么放在 push 之前，要么在它后面补一次 push。
          这条不变量现在只靠这一处维持，改 koi_step 时务必看一眼。 */
    swim_push(&k->x, &k->y, k->L * k->grow * 0.55f + 3.0f);
}

/* 需要整屏重画的两种情形：开机第一帧（缓冲还是空的），以及昼夜过渡期
   （每一行的水色都在变，脏矩形帮不上忙）。其余时间一律走脏区。
   ★ 这个标志必须"稳态时归零" —— 第一版只在昼夜过渡里清它，白天稳态进不到那个
     分支，于是标志常年为 1、白天也每帧整屏重推，脏区优化形同虚设。
     是主机台架的日志"脏区=100.0% rect=0"把它揪出来的（不在板子上也看得见）。 */
static int s_first_frame = 1;

/* ==========================================================================
   ★★ 第 52 轮：断电续玩（NVS）
   --------------------------------------------------------------------------
   王总原话：「机器上有个电源键，我希望的是在关机重启后还是在鱼池界面里
              而不是重新开始，除非玩家长按 OK 自己重置」

   为什么必须用 NVS（flash）而不是 RTC memory：
     那个电源键是**硬断电**，没有任何关机回调可以挂钩 —— RTC memory 只活到
     deep-sleep，掉电就没了。所以唯一能活过断电的是 flash；而且因为**没有
     关机事件**，必须**定时自动存**，不能指望"退出时存一次"。

   存什么 / 不存什么：
     存：scene / 条数 / 分色 / 每条鱼的 位置·朝向·体长·成长度·摆尾频率 /
         饱食度 / 昼夜 —— 这些是"玩家的进度"。
     不存：涟漪、飞行中的饲料、池里漂着的饲料、受惊计时 —— 都是瞬时状态，
           恢复出来只会让开机第一屏莫名其妙地有一堆水花。
     ⚠️ 红斑 sp[][] **不存**：它由 pond_init() 里的 make_spots 按 pat 生成，
        而 pat 已由 (pick_n, split_kh) 决定 ⇒ 用存档参数重放 pond_init()
        就能得到**逐位相同**的红斑（荷叶/波光同理）。这样存档只有 272 B，
        flash 磨损才扛得住。

   ★ 恢复的做法（关键）：
       s_pick_n / s_split_kh 先还原 → 调 pond_init()（荷叶/波光/红斑按原参数
       重放，与断电前逐位相同）→ 再把 grow/L/hz/x/y/headA/phase 覆盖回去。
       **不重新发明一套建池流程**，所以荷叶位置不会漂。

   ★ flash 磨损（这条不能不算）：
       NVS 一页 4KB，blob 写入是追加式的，写满才回收（GC = 拷贝 + 擦除）。
       存档 272B ⇒ 一页约 15 次写入就 GC 一次。
       预算：每 30s 检查、且**只有真变了才写** ⇒ 一天最多 2880 次写入
       ÷ 15 ≈ 192 次擦除/天 ⇒ 10 万次擦除寿命 ≈ **1.4 年**。
       ⚠️ "只在真变了才写"是关键：鱼不动时不写。别改成每帧写。

   ⚠️ ⚠️ 绝对不要 nvs_flash_erase()：nvs 分区(0x9000)里还有**原厂配网数据**，
       擦了以后灌回原厂固件要重新配网（koi_flash.py 的注释里记着这笔账）。
       nvs_flash_init() 失败就**降级成不存档**，绝不用擦除来救。
   ========================================================================== */
#define KOI_NS          "koipond"
#define KOI_KEY         "state"
#define KOI_SAVE_MAGIC  0x4B4F4931u     /* "KOI1" */
#define KOI_SAVE_VER    1
#define KOI_SAVE_SEC    30.0f           /* 自动存盘的检查间隔（秒） */

typedef struct { float x, y, headA, phase, L, grow, hz; } koi_save1_t;

typedef struct {
    uint32_t magic, ver;
    uint8_t  scene, nkoi, pick_n, split_kh, night;
    float    satiety;
    koi_save1_t f[MAX_KOI];
} koi_save_t;                            /* 4+4 + 5+3pad + 4 + 9×28 = 272 B */

static int   s_save_ok  = 0;             /* NVS 可用（初始化成功且未降级） */
static float s_save_acc = 0.0f;          /* 距上次检查的秒数（只在池内累加） */
static float s_save_grow0 = -1.0f;       /* 上次落盘时的最大 grow */
static float s_save_sat0  = -1.0f;       /* 上次落盘时的饱食度 */

static void koi_save_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e != ESP_OK) {
        /* ⚠️ 不擦除：nvs 里还有原厂配网数据。降级成"不存档"，鱼照游。 */
        ESP_LOGW(TAG, "NVS 不可用(%s)——本次不存档，断电后会回首页", esp_err_to_name(e));
        s_save_ok = 0;
        return;
    }
    s_save_ok = 1;
}

static void koi_save_write(void)
{
    if (!s_save_ok || s_scene != 1) return;
    koi_save_t sv;
    memset(&sv, 0, sizeof(sv));
    sv.magic    = KOI_SAVE_MAGIC;
    sv.ver      = KOI_SAVE_VER;
    sv.scene    = 1;
    sv.nkoi     = (uint8_t)s_nkoi;
    sv.pick_n   = (uint8_t)s_pick_n;
    sv.split_kh = (uint8_t)s_split_kh;
    sv.night    = (uint8_t)(s_nightTarget > 0.5f ? 1 : 0);
    sv.satiety  = s_satiety;
    for (int i = 0; i < s_nkoi; i++) {
        sv.f[i].x     = s_koi[i].x;
        sv.f[i].y     = s_koi[i].y;
        sv.f[i].headA = s_koi[i].headA;
        sv.f[i].phase = s_koi[i].phase;
        sv.f[i].L     = s_koi[i].L;
        sv.f[i].grow  = s_koi[i].grow;
        sv.f[i].hz    = s_koi[i].hz;
    }
    nvs_handle_t h;
    if (nvs_open(KOI_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, KOI_KEY, &sv, sizeof(sv)) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

/* 返回 1 = 有存档且已恢复到池内（调用方据此跳过开局三屏） */
static int koi_save_read(void)
{
    if (!s_save_ok) return 0;
    nvs_handle_t h;
    if (nvs_open(KOI_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    koi_save_t sv;
    size_t len = sizeof(sv);
    esp_err_t e = nvs_get_blob(h, KOI_KEY, &sv, &len);
    nvs_close(h);
    /* ⚠️ 用 `len < sizeof(sv)` 而不是 `!=`：nvs_get_blob 会把 *len 改成**实际**
       读出的字节数，旧版本存档可能更短；只要够长就认。 */
    if (e != ESP_OK || len < sizeof(sv)) return 0;
    if (sv.magic != KOI_SAVE_MAGIC || sv.ver != KOI_SAVE_VER) return 0;
    if (sv.scene != 1 || sv.nkoi < 1 || sv.nkoi > MAX_KOI) return 0;
    /* 自洽性校验：条数必须是一档合法选项，分色不能超总数 —— 否则 pond_init
       会造出一池跟玩家选的对不上的鱼。 */
    int ok_n = 0;
    for (int i = 0; i < N_PICK; i++) if (PICK_N[i] == sv.pick_n) { ok_n = 1; break; }
    if (!ok_n || sv.split_kh > sv.pick_n) return 0;

    /* ★ 用存档参数重放 pond_init：荷叶/波光/红斑与断电前逐位相同 */
    s_pick_n   = sv.pick_n;
    s_split_kh = sv.split_kh;
    s_nkoi     = sv.nkoi;
    pond_init();
    for (int i = 0; i < s_nkoi; i++) {
        s_koi[i].x     = sv.f[i].x;
        s_koi[i].y     = sv.f[i].y;
        s_koi[i].headA = sv.f[i].headA;
        s_koi[i].phase = sv.f[i].phase;
        s_koi[i].L     = sv.f[i].L;
        s_koi[i].grow  = clampf(sv.f[i].grow, 0.30f, GROW_MAX);
        s_koi[i].hz    = sv.f[i].hz;
        s_koi[i].burst = 1;
        /* 脏区状态必须重置：否则拿断电前的 AABB 去算本帧脏矩形，会漏画/花屏 */
        s_koi[i].bx0 = s_koi[i].bx1 = s_koi[i].x;
        s_koi[i].by0 = s_koi[i].by1 = s_koi[i].y;
        s_koi[i].ax0 = 1e30f; s_koi[i].ax1 = -1e30f;      /* 空 AABB */
        s_koi[i].ay0 = 1e30f; s_koi[i].ay1 = -1e30f;
    }
    s_satiety     = clampf(sv.satiety, 0.0f, 1.0f);
    s_nightTarget = sv.night ? 1.0f : 0.0f;
    s_night       = s_nightTarget;
    s_night_log   = s_nightTarget;                        /* 37 轮：量化档也要跟着走 */
    s_nrip = 0; s_npel = 0; s_nfood = 0;                  /* 瞬时状态不恢复 */
    s_time = 0; s_shakeT = 0; s_save_acc = 0.0f;
    s_scene       = 1;
    s_first_frame = 1;
    s_full        = 1;
    s_bg_drawn    = -1;                                   /* 45 轮：底图重铺 */
    ESP_LOGI(TAG, "续玩：%d 条（红白 %d / 御黄金 %d），饱食 %.2f，%s",
             s_pick_n, s_split_kh, s_pick_n - s_split_kh, s_satiety,
             s_nightTarget > 0.5f ? "夜" : "昼");
    return 1;
}

static void koi_save_clear(void)
{
    if (!s_save_ok) return;
    nvs_handle_t h;
    if (nvs_open(KOI_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, KOI_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static void step(float dt)
{
#ifdef KOI_HOST_PROBE
    s_step_no++;
#endif
    s_time += dt;

    /* ★ 第 52 轮：没有关机回调，只能定时刷盘。且**只在真变了才写**（磨损）。 */
    s_save_acc += dt;
    if (s_save_acc >= KOI_SAVE_SEC) {
        s_save_acc = 0.0f;
        float gm = 0.0f;
        for (int i = 0; i < s_nkoi; i++) if (s_koi[i].grow > gm) gm = s_koi[i].grow;
        if (fabsf(gm - s_save_grow0) > 0.001f ||
            fabsf(s_satiety - s_save_sat0) > 0.01f) {
            s_save_grow0 = gm;
            s_save_sat0  = s_satiety;
            koi_save_write();
        }
    }
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
        float ox = k->x, oy = k->y, oa = k->headA, oc = k->curv;
        koi_step(k, dt);
        float dx = k->x - ox, dy = k->y - oy;
        float da = k->headA - oa;
        float dc = k->curv - oc;
        float x0, y0, x1, y1;
        if (k->ax1 >= k->ax0) {
            /* ★★ 第 46 轮：**按两个角各转一次再求并**。
               起因（量出来的）：真机帧长 ~105ms、峰值 142ms 时，"鱼脏盒最大欠缺"
               从 1.75px（42ms）涨到 6.8~14px。逐帧打现场发现全部落在**弯身**上：
                 步5 鱼2  curv +0.389 → −0.081（一帧内反向）⇒ 真 AABB 的 x1 比报盒多 6.0px
               机理：`koi_spine` 里 `a += curv·KBEND[i]`，而 **KBEND 之和 = 1.00**
                 ⇒ 尾端绝对角 = headA + curv；头端 = headA。
                 所以一帧之内，**头**转了 ΔheadA，**尾**转了 ΔheadA + Δcurv。
                 旧代码只绕 (x,y)（= 头）转 ΔheadA ⇒ 尾端那 Δcurv 整段没人管。
               量级对得上：Δcurv 0.47 × 尾端力臂 ≈ 20px ⇒ 侧向 ~9px，与实测 6~14px 同阶。
               ■ 修法：把上帧 AABB 绕头**分别按 ΔheadA 与 ΔheadA+Δcurv 转**，两个盒取并。
                 鱼体对头是**星形**的（头在体内偏后，鼻子只伸出 hw·cap/2），所以
                 中间角度的姿态夹在这两个极端姿态之间；严格说圆弧还会鼓出
                 r·Δθ²/8 ≈ 50·0.47²/8 = 1.4px，那点由 AABB 自带的 −1/+2 余量吃掉。
               ⚠️ 只做"平移副本"是不够的：**转身**时包围盒会朝侧向长出去，
                  平移并集盖不住 —— 台架实测过，一侧欠缺 176px（那样尾巴会残留）。
               ⚠️ 代价：多 4 个角的旋转（sin/cos 只算两次），脏盒**不会因为这一改就变大**
                  —— 只有真在弯身的那一帧才多包一点，直游时 dc≈0、与旧口径逐位相同。 */
            float bx[4] = {k->ax0, k->ax1, k->ax0, k->ax1};
            float by[4] = {k->ay0, k->ay0, k->ay1, k->ay1};
            x0 = 1e30f; y0 = 1e30f; x1 = -1e30f; y1 = -1e30f;
            for (int pass = 0; pass < 2; pass++) {
                float ang = pass ? (da + dc) : da;      /* pass0 = 头端角，pass1 = 尾端角 */
                float ca = fcos_t(ang), sa = fsin_t(ang);
                for (int c = 0; c < 4; c++) {
                    float px = bx[c] - ox, py = by[c] - oy;
                    float rx = ox + px * ca - py * sa + dx;
                    float ry = oy + px * sa + py * ca + dy;
                    if (rx < x0) x0 = rx;
                    if (rx > x1) x1 = rx;
                    if (ry < y0) y0 = ry;
                    if (ry > y1) y1 = ry;
                }
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
        /* ★★ 第 46 轮 · 活泼度探针（只给台架）。
           王总说「鱼的游动……不活泼」「现在游动很飘逸」—— 这两句都是**时间维**的
           观感，单张图完全看不出来（铁律 14）。而调活泼度的五个乘数（KOI_LV_*）
           里哪一个在起什么作用，也得有数，否则就是"调完看着好像活泼了点"。
           这里逐帧累加四个量（只在有脏区的稳态帧上算，整屏帧不计）：
             · 巡游速度 px/s      —— "游得快不快"
             · 转头速率 rad/s      —— "转得灵不灵"
             · |curv|             —— 身体弯不弯（活泼的鱼一直在扭）
             · burst 占空比        —— "冲得多还是滑得多"（"飘逸" = coast 占比太高）
           口径：速度用**帧初帧末位置差 / dt**（就是报脏用的那个 disp），
                 不是 k->v —— k->v 是一阶滞后的目标值，跟真实位移差一截。 */
        if (!s_full && dt > 0.0f) {
            float sp = sqrtf(dx * dx + dy * dy) / dt;
            s_lv_spd += sp; s_lv_turn += fabsf(da) / dt;
            s_lv_curv += fabsf(k->curv);
            s_lv_burst += k->burst ? 1.0f : 0.0f;
            s_lv_n += 1.0f;
        }
#endif
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
                    /* ★★ 第 51 轮：**落水后入 s_food，但屏幕上不画**（KOI_FOOD_DRAW=0）。
                       王总原话：「停留加回来 但是显示是隐藏的」——
                         要"停留"：鱼才有东西可追、可吃（否则游到落点扑空，
                                   就是他说的"碰到鱼食的几率很小"）；
                         要"隐藏"：屏上不出现留在池里的饲料（第 43 轮「其他不落池」）。
                       这两个要求**不矛盾** —— 存在 ≠ 可见。
                       ⚠️ 有 FOOD_LIFE(=2s) 兜底：2 秒没被吃就自己消失，不会无限堆积。
                       ⚠️ 雨点（food=0）**不入池**，只留一圈水花，保持"下雨"的观感。 */
                    if (s_nfood < MAX_FOOD) {
                        s_food_x[s_nfood]   = pe->tx;
                        s_food_y[s_nfood]   = pe->ty;
                        s_food_age[s_nfood] = 0;
                        s_nfood++;
                    }
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
        /* ★ 52 轮：走快照（默认开）。lily_rot 与标脏共用同一个量化角，画/报永远一致。 */
#if KOI_LILY_SNAP
        lily_snap_draw(i, lily_rot(L), ly);
#else
        lily_paint(L, lily_rot(L));
#endif
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
    /* ★★ 第 44 轮：L 32 曾按"体宽系数变成 0.295"改成 5.6/KOI_WD，好让图标
       （固定尺寸卡片里的一枚装饰）不跟着鱼变胖而长高。体宽撤回 0.175 之后，
       这里逐字回到 32.0f。
       ⚠️ 留一句给下次：**图标的 L 与 KOI_WD 是耦合的** —— 图标全高 = 4×L×KOI_WD，
          卡片留白是按旧值调好的。真要改 KOI_WD，这里必须一起改，
          否则 `ik.y = cy2 − h/2` 会把鱼顶出卡片（32 × 0.295 那版就是 37.8px 高）。 */
    const float L  = 32.0f;
    float Wd = L * KOI_WD;                       /* ★ 第 44 轮：0.175 → KOI_WD（值仍是 0.175）。
                                                    这里只用来算**画布留白**（下面的 w/h），
                                                    而 koi_draw 内部已经用 KOI_WD 画了 ——
                                                    两处必须同一个数，否则图标会被画布裁掉。 */
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
        /* ★★ 第 53 轮：分色屏（进池前最后一屏）下面再加一行**小字**——
           王总原话「下面你给我加个 开启后长按OK键可重置 这个字可以小点 一行显示出来」。
           为什么要加：第 52 轮做了断电续玩，上电会**直接进池、不再是首页**；
           玩家一旦忘了"怎么回首页"就出不来了 —— 这行字就是那个出口的说明。
           ⚠️ 位图是 koi_assets.h 里的 UI_TIP_RESET（13px 146x23，比主提示条的
              18px 小一档）；固件没有字体引擎，一行字 = 一张烘好的点阵。
           ⚠️ 只挂在第 2 屏（分配颜色）—— 那是"开启"动作发生的那一屏，
              挂在第 1 屏（选条数）会让人以为现在就能长按。 */
        ui_draw(UI_TIP_RESET, KW / 2, KH - 14);
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

/* ★★ 第 45 轮：底图换源（白天 koi_bg ⇄ 夜间 koi_bg_night）⇒ 本帧必须整屏。
   **只允许在 render() 正前方调一次**（tick_cb 里那一处），别在帧首再判一遍。
   理由：s_night 有两个来源 ——
     · 帧外：按键回调直接 `s_night = s_nightTarget`（第 43 轮"OK 键瞬切"）；
     · 帧中：step() 里的昼夜 ramp 推进。
   放在 render() 前 = 两种来源在**同一处**被裁决。若改到帧首，只能覆盖帧外那种，
   帧中那种就漏了；而"两处各判一次"迟早会漏一处，表现就是换夜那一帧只有脏矩形
   换了底图（半屏白天图半屏夜间图）—— 铁律 13 那一族（状态变了但没人重建）。 */
static void bg_sync_full(void)
{
    int wb = bg_want_night();
    if (wb != s_bg_drawn) { s_full = 1; s_bg_drawn = wb; }
}

/* ==========================================================================
   ★★ 第 46 轮：把"每帧固定推进 1/24 秒"改成"按**真实经过时间**推进"
   --------------------------------------------------------------------------
   王总：「鱼的游动有点问题 不活泼 … 现在游动很飘逸 微卡顿」。
   量出来的根因是**时间膨胀**（不是速度参数太小）：
     · 物理写死 `step(1.0f/FPS)` = 每帧推进 41.7ms；
     · 但真机一帧要 ~105ms（5 条鱼 + 6 片荷叶），tick 实际只有 ~8Hz。
     ⇒ 物理时间 / 真实时间 = 41.7 / 105 = **40%**。
       表现正好是王总说的那两条：
         · 鱼速只剩设计值的 40%（≈0.45 体长/秒）→「不活泼 / 很飘逸」；
         · 摆尾频率同样打 4 折（1.5~2.4Hz → 0.6~1.0Hz）→ 从"游"变成"滑"。
   而且这条 bug **只在慢帧上才显形** —— 第 43 轮 74ms/帧时是 56% 速度，
   第 44 轮换成照片背景后掉到 6fps，才被王总看出来。

   ■ 修法：`step(dt)` 用**真实经过时间**当步长。
     ⚠️ 试过、并且**否掉**的方案：按真实 dt 决定"这一帧推进几次 1/24 子步"。
        理由是"整套运动参数都在 1/24 上标定的，别动步长"。但实测这个方案有个硬伤：
        105ms ÷ 41.7ms = **2.52 步**，只能取 2 或 3 ⇒ 逐帧在 0.79× 和 1.19× 之间跳，
        **每帧 ±19% 的速度抖动** —— 那正是王总说的"微卡顿"，等于用一个更显眼的毛病
        换掉一个不那么显眼的毛病。除非把子步长减半（1/48），但那又会让一阶滞后项
        的系数从 0.158 掉到 0.079，鱼会变迟钝。**两头都不划算。**
     直接用真实 dt 没有量化误差：每帧推进的时间**恰好**是真实经过的时间，
     时间膨胀比 = 1.00，帧间速度也是均匀的。
     代价是"一阶滞后项"的系数会随帧长浮动（`dt*3.8` 从 0.158 变成 0.40）——
     但它本来就是"指数逼近"，dt 大只是**逼近得快一点**（速度/曲率在 2~3 帧内到位
     而不是 5~6 帧），语义没错、也不会过冲（系数都钳在 ≤1）。
     而这个"更跟手"的副作用，恰好是王总要的"活泼"。
   ■ KOI_DT_MAX = 0.15s：真机 max=142ms 刚好不被钳。再长的帧（整屏重画那类）
     就让它慢一点，这是**故意的** —— 否则一次卡顿会让鱼瞬移一大截。
   ■ KOI_DT_MIN = 1/240s：只用来挡住 dt==0 / NaN，不参与"限速"。
   ⚠️ 台架必须用 `KOI_FRAMEMS` 复现真机帧长，否则 x86 一帧 1~3ms，
      台架看到的鱼速跟板子完全不是一回事（见 host_main.c 的合成时钟）。
   ⚠️ 改这个步长会**改变随机序列的消费节奏**（wanderT 到点才抽随机数），
      所以鱼位会与第 45 轮不同 —— 这是预期的，不是回归。判据看那几条不变量。 */
#define KOI_DT_MIN   (1.0f / 240.0f)
#define KOI_DT_MAX   0.15f
static int64_t s_dt_last;
static float   s_dt_last_sec = 1.0f / 24.0f;   /* 最近一帧真正用的 dt（秒） */

static float frame_dt(void)
{
    int64_t now = esp_timer_get_time();
    if (s_dt_last == 0) { s_dt_last = now; s_dt_last_sec = 1.0f / (float)FPS; return s_dt_last_sec; }
    float dt = (float)(now - s_dt_last) / 1000000.0f;
    s_dt_last = now;
    if (!(dt > 0.0f)) dt = KOI_DT_MIN;          /* 也挡住 NaN */
    if (dt < KOI_DT_MIN) dt = KOI_DT_MIN;
    if (dt > KOI_DT_MAX) dt = KOI_DT_MAX;       /* 见上面注释：上限是故意的 */
    s_dt_last_sec = dt;
    return dt;
}

#ifdef KOI_HOST_PROBE
/* ★ 46 轮新增判据：**时间膨胀比** = 物理推进的秒数 ÷ 真实经过的秒数。理想 = 1.00。
   修之前固定 1/24 步进、真机 8Hz ⇒ 这个数是 **0.40** —— 正是王总说的"不活泼"。
   修完之后它应当 ≈1.00；小于 1 只可能是"帧长超过 KOI_DT_MAX 被钳住"（慢动作兜底）。
   ★ 为什么它必须是独立计数器、而不是"用 s_time 算"：s_time 会被 start_pond 清零、
     而且开局三屏不推进 —— 拿它当分子会得到没有意义的数（判据自己不可靠比没有更坏）。 */
static float s_dil_sim, s_dil_real;
float demo_koi_time_dil(void)
{
    if (s_dil_real < 1e-3f) return 0.0f;
    return s_dil_sim / s_dil_real;
}
#endif

static void tick_cb(lv_timer_t *t)
{
    (void)t;
    int64_t t0 = esp_timer_get_time();
    PROF_START();

    /* ★★ 第 43 轮修正：「调色板变了 ⇒ 整屏重画」这条不变量在**帧首**也必须成立。
       原来这里 `build_palette(s_night);` 的返回值被丢掉，紧接着 `s_full = 0;`
       —— 于是**帧外**改了 s_night 的情况（第 43 轮新加的"OK 键昼夜瞬切"就是
       在 demo_koi_key 里直接 s_night = target）会整帧漏画：
         · 帧 N（按键那一帧）：s_night 已改，但本帧 tick 已经跑完了；
         · 帧 N+1：帧首 build_palette 悄悄把调色板重建了（返回值没人接）→
                    s_full 被清成 0 → step() 里 ramp 认为"已到位"置不置 s_full →
                    render() 只画那几个脏矩形，用的却是**新的夜色调色板**；
                    屏上其余水面还停在白天的亮度 → 一张屏两种水色。
       台架金标准把它量出来了：第 710 帧 增量≠整屏 **58785 像素**，
       「脏区覆盖」漏画 58401 像素、漏点 (0,0)…(7,0) —— 左上角就是水面本身，
       不属于任何鱼/荷叶/饲料，按对象找永远找不到。
       修法：把帧首这次 build_palette 的"是否真的重建了"接住，直接决定本帧整屏。
       与 step() 之后那次（`if (build_palette(s_night)) s_full = 1;`）合起来，
       "帧首改的"和"帧中 ramp 推进的"两边都覆盖到了。 */
    int pal_pre = build_palette(s_night);
    PROF_TICK(0);

    /* ★★ 第 46 轮：这一帧的物理步长 = **真实经过时间**（详见 frame_dt 的注释）。
       每帧都要算（包括开局三屏）—— 开局三屏不跑物理，若那时不更新 s_dt_last，
       从首页进池的第一帧会看到"在首页停了几十秒"的假 dt（会被 KOI_DT_MAX 钳住，
       但仍是白送的一次 0.15s 瞬移）。放帧首 = 无论什么场景时钟都连续走。 */
    float dt = frame_dt();

    s_nrect = 0;
    s_full = pal_pre;           // 帧首调色板已变 ⇒ 整屏；否则由 step() 决定
    s_koi_n = 0; s_lily_n = 0;  // 本帧的重复绘制计数，从 0 起
    if (s_scene == 0) {
        /* 开局三屏：不跑物理，只有"换屏请求"那一帧整屏重画一次，之后零脏区 */
        build_palette(s_night);
        PROF_TICK(0);
        if (s_setup_dirty) { s_full = 1; s_setup_dirty = 0; }
    } else {
        /* ★★ 第 46 轮：`step(dt)` —— 真实步长。
           原来这里是 `step(1.0f / (float)FPS)`（固定 41.7ms），真机 8Hz 时
           物理只跑到真实时间的 40%，鱼看起来又慢又飘（王总「不活泼 / 很飘逸」）。 */
        step(dt);
#ifdef KOI_HOST_PROBE
        /* 时间膨胀比的分子分母都在**池内**才累加（开局三屏不跑物理，算进去会拉低比值）。 */
        s_dil_sim  += dt;
        s_dil_real += s_dt_last_sec;
#endif
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
    /* ★★ 第 45 轮：底图换源（白天 koi_bg ⇄ 夜间 koi_bg_night）⇒ 本帧必须整屏。
       位置很讲究：**必须紧贴 render()**，不能挪到帧首。
         · 帧首 `s_full = pal_pre;` 那行是**赋值**不是置位 —— 写在它前面会被冲掉
           （第 44 轮就是在这一步栽过：pal_pre 的返回值被后面的 s_full = 0 冲掉）；
         · 更要紧的是：s_night 在**帧中**还会被 step() 里的昼夜 ramp 推进，
           帧首判一次、帧中变了就漏了。放在这里 = 帧外（按键瞬切）与帧中（ramp）
           两种来源在**同一处**裁决，不会出现"两处判定、漏一个"。
       ⚠️ 别和上面 pal_pre 合并成一个 if —— "调色板重建"与"底图换源"是两件事，
          恰好同时发生只是通常情形，不是必然（day_pal_sync 会强制重建调色板但底图没换）。 */
    bg_sync_full();
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

    /* ★★ 第 52 轮：先开 NVS，再决定"续玩"还是"从首页开始"。
         · koi_save_read() 成功 = 有存档 ⇒ 直接回池内，**跳过开局三屏**；
         · 只有玩家长按 OK（reset_to_home → koi_save_clear）清过档，才走到 else。
       ⚠️ koi_save_read() 内部会按存档参数重放 pond_init()，荷叶/波光/红斑不漂。
       ⚠️ 夜间续玩不用额外处理：tick_cb 帧首的 build_palette(s_night) 会自己发现
          调色板变了并置 s_full=1（第 43 轮修的就是这个）。 */
    koi_save_init();
    int resumed = koi_save_read();
    if (!resumed) {
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
    }
    s_nrect = 0; s_full = 1; s_first_frame = 1; s_bg_drawn = -1;  /* 45 轮：底图重铺 */
    s_frames = 0; s_log_acc = 0; s_sum_us = 0; s_max_us = 0; s_dirty_acc = 0;
    s_t_last = esp_timer_get_time();
    s_dt_last = 0;              /* ★ 46 轮：子步时钟也从零起（第一帧算 1 子步） */

    if (resumed) {
        ESP_LOGI(TAG, "锦鲤池进入：画布 %dx%d RGB565 = %u B，目标 %d fps；续玩（读回存档，直接进池）",
                 KW, KH, (unsigned)sizeof(s_fb), FPS);
    } else {
        ESP_LOGI(TAG, "锦鲤池进入：画布 %dx%d RGB565 = %u B，目标 %d fps；开机=首页，%d 档条数可选",
                 KW, KH, (unsigned)sizeof(s_fb), FPS, N_PICK);
    }

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
    s_bg_drawn = -1;                         /* ★ 45 轮：底图也跟着重铺 */
    ESP_LOGI(TAG, "开养：%d 条（红白 %d / 御黄金 %d）",
             s_pick_n, s_split_kh, s_pick_n - s_split_kh);
    /* ★ 第 52 轮：选完条数/分色进池的这一刻就落盘 —— 断电后能直接回到这一池。 */
    s_save_grow0 = -1.0f; s_save_sat0 = -1.0f;      /* 强制写一次，别被"没变"挡掉 */
    koi_save_write();
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
    s_bg_drawn = -1;                         /* ★ 45 轮：底图也跟着重铺 */
    /* ★ 第 52 轮：清档。这就是王总说的「除非玩家长按 OK 自己重置」——
       长按 OK 走的就是这个函数，擦掉存档后下次上电回开局三屏。 */
    koi_save_clear();
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
        s_night       = s_nightTarget;                            /* ★ 第 43 轮：按下直接瞬切（王总要"按下直接变暗，再按下直接变亮"）；
                                                旧路径是 1 秒逐帧渐变（line 2634 ramp），现在 ramp 不会跑
                                                （s_night == s_nightTarget），整屏由下面 s_full=1 触发。 */
        s_night_log   = s_night;
        /* 整屏重画**不在这里置** —— s_full 会被下一帧 tick 开头的 `s_full = pal_pre`
           覆盖（帧外改状态本来就该由帧首统一裁决，按键回调里抢着置位是无效的）。
           真正的保证在 tick_cb：`pal_pre = build_palette(s_night)`，
           调色板只要在帧首真的重建了，本帧就一定是整屏。见 tick_cb 的第 43 轮注释。 */
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
/* ★★ 第 44 轮：背景源开关的运行时入口（只给台架用）。
   0 = 程序化渐变（第 43 轮那套），1 = 照片背景。用来做 A/B 与逐字节对拍。 */
void demo_koi_set_bgphoto(int on)
{
    s_bg_photo = on ? 1 : 0;
    s_full = 1;                 /* 换底图必须整屏重画一次，否则半屏照片半屏渐变 */
    s_first_frame = 1;
}

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

/* ★★ 第 46 轮：最大欠缺的现场（帧号 / 鱼号 / 0=x0 1=x1 2=y0 3=y1 / 未扣 margin 的原始欠缺 /
   当时的 grow 与 headA）。用来判断"是形变随 dt 变大"还是"某个事件推了一把"。 */
int   demo_koi_bs_step(void) { return s_bs_step; }
int   demo_koi_bs_koi(void)  { return s_bs_koi; }
int   demo_koi_bs_side(void) { return s_bs_side; }
float demo_koi_bs_raw(void)  { return s_bs_raw; }
float demo_koi_bs_grow(void) { return s_bs_grow; }

/* ★★ 第 46 轮 · 活泼度四项读数（稳态帧平均；见报脏循环里的累加注释）。
   返回 0 表示没有样本。 */
float demo_koi_lv_speed(void)  { return s_lv_n > 0 ? (float)(s_lv_spd  / s_lv_n) : 0.0f; }
float demo_koi_lv_turn(void)   { return s_lv_n > 0 ? (float)(s_lv_turn / s_lv_n) : 0.0f; }
float demo_koi_lv_curv(void)   { return s_lv_n > 0 ? (float)(s_lv_curv / s_lv_n) : 0.0f; }
float demo_koi_lv_burst(void)  { return s_lv_n > 0 ? (float)(s_lv_burst / s_lv_n) : 0.0f; }
double demo_koi_lv_n(void)     { return s_lv_n; }

/* ★★ 第 45 轮：安全区自检 —— 帧内所有鱼"离安全区边界的最大越界深度"（px）。
   ≤0 = 全部在区内（这是第三条不变量，前两条是脏区渲染 / 脏区覆盖）。
   ■ 为什么要单独验：swim_push 只迭代 3 轮（凸集上够用），但"够用"是推的，
     得量。残差一旦 >0，表现是**鱼在某个角上擦着石头过**，肉眼很难发现，
     可一旦发生就是"鱼进了石头里"这种一眼假的画面。
   ■ ★ 判据必须**独立于被测代码**：这里重新逐边算一遍 signed distance，
     绝不复用 swim_push 的返回值/中间量 —— 复用的话它只会重复被测代码的 bug，
     绿得毫无意义（铁律 11/12：判据自己不可靠时，它给出的数字比没有更坏）。
   ■ 量的是"半平面交"而不是原多边形：因为固件实际约束的就是这个凸交集
     （原多边形非凸，见 12a 节）。若要验原多边形，得另写射线法。 */
float demo_koi_swim_worst(void)
{
    if (s_scene != 1) return -1e9f;
    float worst = -1e9f;
    for (int i = 0; i < s_nkoi; i++) {
        const koi_t *k = &s_koi[i];
        float m = k->L * k->grow * 0.55f + 3.0f;
        for (int e = 0; e < SWIM_N; e++) {
            const float *a = SWIM_PT[e], *n = SWIM_NRM[e];
            float d = (k->x - a[0]) * n[0] + (k->y - a[1]) * n[1] - m;
            if (-d > worst) worst = -d;
        }
    }
    return worst;
}

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

/* ★ 第 50 轮：台架查询"当前存活的涟漪数（按 kind）"。
   kind：0 = 拍水 tap / 1 = 落水 drop / 2 = 吃食 eat。
   为什么需要它：ripchk50.py 第一版想用"三档画面差分"量涟漪存续帧数，
   结果 zig cc 对不同 -D 宏的浮点舍入漂移（~3000 像素/帧）把涟漪那点差异
   整个淹没了 —— 差分这条路走不通。涟漪**数量**是固件自己的真值，
   直接读它才是独立于被测代码的判据（铁律 11）。
   ⚠️ 真机不调用，没有开销；只是给台架开的一个读数口。 */
int demo_koi_rip_count_kind(int kind)
{
    int n = 0;
    for (int i = 0; i < s_nrip; i++)
        if (s_rip[i].kind == kind) n++;
    return n;
}

int demo_koi_seek_n(void)
{
    int n = 0;
    for (int i = 0; i < s_nkoi; i++)
        if (s_koi[i].seek) n++;
    return n;
}

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
