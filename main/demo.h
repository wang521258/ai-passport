// main/demo.h —— 每个演示页实现的统一接口。
// 新增一个演示页 = 实现这三个函数 + 在 main.c 的 DEMOS[] 里加一行。
#pragma once

#include <stdint.h>
#include "bsp_button.h"

typedef struct {
    const char *name;
    void (*enter)(void);                          // 建自己的屏并载入
    void (*exit)(void);                           // 删屏、停定时器、释放资源
    void (*key)(bsp_btn_t btn, bsp_btn_ev_t ev);  // 收按键(长按确定已被 main 拦截)
} demo_entry_t;

// 各演示页(定义在各自的 .c 里)
void demo_pixel_pet_enter(void); void demo_pixel_pet_exit(void);
void demo_pixel_pet_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_display_enter(void); void demo_display_exit(void);
void demo_display_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_button_enter(void);  void demo_button_exit(void);
void demo_button_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_audio_enter(void);   void demo_audio_exit(void);
void demo_audio_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_battery_enter(void); void demo_battery_exit(void);
void demo_battery_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_wifi_enter(void);    void demo_wifi_exit(void);
void demo_wifi_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_ble_enter(void);     void demo_ble_exit(void);
void demo_ble_key(bsp_btn_t btn, bsp_btn_ev_t ev);

void demo_low_power_enter(void); void demo_low_power_exit(void);
void demo_low_power_key(bsp_btn_t btn, bsp_btn_ev_t ev);

// 锦鲤池:全屏自绘(不用 LVGL 控件),240x320 ST7789 竖屏。
void demo_koi_enter(void);     void demo_koi_exit(void);
void demo_koi_key(bsp_btn_t btn, bsp_btn_ev_t ev);

/* ★ 第 50 轮：台架专用（KOI_HOST_PROBE 时存在；真机不调，零开销）。
   返回当前存活的涟漪数，按 kind 分开（0 tap / 1 drop / 2 eat）。
   ripchk50.py 用它判定"涟漪在真机帧率下能存续几帧"——绕开了"三档画面
   浮点漂移淹没涟漪差异"那条死路。 */
int  demo_koi_rip_count_kind(int kind);

/* ★ 第 50 轮：当前**正在追食**（k->seek）的鱼有几条。
   王总第 50 轮核心诉求「鱼应该往饲料方向游动 而不是现在的没感觉」——
   "有没有追"这件事必须数出来，不能靠看 GIF 猜。 */
int  demo_koi_seek_n(void);

/* ★ 第 55 轮：离散成长的三个读数（台架判据用，真机零开销）。
     demo_koi_maxgrow()   池里最大的 grow（成长是整池一起长，取最大就够）
     demo_koi_feedcnt()   玩家按喂食键的累计次数（0..FEED_PER_GROW-1）
     demo_koi_ eaten()    ★ 累计"鱼吃到食物"的次数
   为什么三个都要：第 55 轮的口径是「成长只认按键次数，跟吃食无关」，
   判据必须能**同时**看到"吃了很多次但 grow 没动"和"按满 100 次 grow 跳一档"
   这两件事 —— 只看 grow 一个数，分不清是"没吃到"还是"真的解耦了"。 */
float    demo_koi_maxgrow(void);
uint32_t demo_koi_feedcnt(void);
uint32_t demo_koi_eaten(void);
