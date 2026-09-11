// components/bsp/include/bsp_pins.h
// FoloToy AI Passport → 正点原子 ATK-DNESP32S3-BOX 硬件引脚【单一事实来源】。
//
// 板型认定依据(不靠猜):板子原厂小智固件启动日志打印
//     I (233) Board: UUID=... SKU=atk-dnesp32s3-box
// 引脚全部取自小智官方板级定义 main/boards/alientek/atk-dnesp32s3-box/。
//   ⚠ 注意:是 atk-dnesp32s3-box,【不是】atk-dnesp32s3-box3。
//     两者屏接口完全不同:box=8080 并口 8bit / box3=SPI。抄错板型必然黑屏。
//
// 硬件:ESP32-S3 (octal PSRAM 8MB@80MHz, Flash 8MB QIO)
//       屏 ST7789 320x240,8bit 8080 并口(i80)
//       XL9555 扩展 IO(I2C 0x20):背光/功放使能/按键
//       音频 NS4168(无 I2C codec;另有 ES8311 版本,靠 XL9555 P0_5 电平区分)
#pragma once

#include "driver/gpio.h"
#include "driver/i2c_types.h"

// ============================================================================
// 显示:ST7789 320x240,8bit 8080 并口(LCD_CAM/i80)。
//   面板原生 320x240 横屏;开 swap_xy(寄存器 0x36 的 MV 位)后,
//   软件坐标空间变成 240x320 竖屏 —— LVGL 画布即用后者。
// ============================================================================
#define BSP_LCD_PANEL_W      320            // 面板原生宽(横屏)
#define BSP_LCD_PANEL_H      240            // 面板原生高
#define BSP_LCD_W            240            // LVGL 画布宽(竖屏,= PANEL_H)
#define BSP_LCD_H            320            // LVGL 画布高(竖屏,= PANEL_W)

#define BSP_LCD_CS           1
#define BSP_LCD_DC           2
#define BSP_LCD_RD           41             // 读选通(本驱动只写,配成输出常高)
#define BSP_LCD_WR           42             // 写选通
#define BSP_LCD_RST          (-1)           // 复位走 SWRESET 软复位

// 8 位数据总线 D0..D7(顺序即 bus_width=8 的低位到高位,不可乱序)
#define BSP_LCD_D0           40
#define BSP_LCD_D1           39
#define BSP_LCD_D2           38
#define BSP_LCD_D3           12
#define BSP_LCD_D4           11
#define BSP_LCD_D5           10
#define BSP_LCD_D6           9
#define BSP_LCD_D7           46

#define BSP_LCD_PCLK_HZ      (10 * 1000 * 1000)
#define BSP_LCD_SWAP_XY      1
#define BSP_LCD_MIRROR_X     1
#define BSP_LCD_MIRROR_Y     0
#define BSP_LCD_INVERT_COLOR 1              // 本屏需反色(0x21 INVON)
// 背光经 XL9555(P1_0,高有效),不由 MCU 直接驱动。此处仅作日志/占位。
#define BSP_LCD_BL           (-1)

// ============================================================================
// 按键:三键。BOOT 直连 GPIO0;另外两个实体键经 XL9555 输入。
//   【实机实测定位 2026-09-11】串口抓到的 XL9555 输入跳变:
//       0xFFFF -> 0xFFEF  即 bit4 落下  → P0_4
//       0xFFFF -> 0xFFF7  即 bit3 落下  → P0_3
//   (按下=0,松开=1。方向寄存器 0x1B 已把 P0_3/P0_4 配为输入,天然吻合。)
//   ⚠ 上下方向以实机操作为准:v9 首版 P0_4=UP/P0_3=DOWN 被反馈"搞反了",
//     故按用户实测对调 → P0_3=UP、P0_4=DOWN。
//   ⚠ 正点原子通用板位表里 P0_3=BEEP、P0_4=OV_PWDN,但 BOX 版没有蜂鸣器/摄像头,
//     这两脚被改接成实体按键 —— 所以必须以实机日志为准,不能照抄位表。
// ============================================================================
#define BSP_BTN_COUNT        3
#define BSP_BTN_BOOT_GPIO    0              // BOOT 键(GPIO0),低有效 → OK
#define BSP_BTN_K1_XIO       3              // XL9555 P0_3 → UP  (实机对调后)
#define BSP_BTN_K2_XIO       4              // XL9555 P0_4 → DOWN(实机对调后)

// ============================================================================
// I2C:XL9555(扩展 IO);本板 NS4168 无 I2C codec(ES8311 版本才有)
// ============================================================================
#define BSP_I2C_PORT         I2C_NUM_0
#define BSP_I2C_SDA          48
#define BSP_I2C_SCL          45
#define BSP_I2C_ES8311_ADDR  0x18           // 7 位地址(本板若无 ES8311 则无应答)
#define BSP_I2C_CW2017_ADDR  0x63           // 本板无独立电量计,init 失败无害

// ============================================================================
// 音频:I2S → NS4168(纯 I2S DAC,无 I2C 控制口)。本版【暂未适配】,
//   bsp_audio_init() 直接返回 NOT_SUPPORTED,避免误占引脚。
//   (引脚按官方 config.h 预留:MCLK 不用 / BCLK21 / WS13 / DOUT14 / DIN47)
// ============================================================================
#define BSP_AUDIO_ES8311     0              // 1=按 ES8311 全双工驱动(需板上有该 codec)
#define BSP_I2S_PORT         I2S_NUM_0
#define BSP_I2S_MCLK         (-1)           // 本板 codec 不需 MCLK
#define BSP_I2S_BCLK         21
#define BSP_I2S_WS           13
#define BSP_I2S_DOUT         14             // 播放:MCU → 功放
#define BSP_I2S_DIN          47             // 录音
#define BSP_I2S_PA_CTRL      (-1)           // 功放使能走 XL9555 P0_5

// ============================================================================
// XL9555 扩展 IO(I2C 地址 0x20)
//   pin 编号用 16 位线性编号:0..7 = P0_0..P0_7,8..15 = P1_0..P1_7
//   寄存器(TCA9535/XL9555 布局):输入 0x00/0x01 输出 0x02/0x03 方向 0x06/0x07(1=输入)
//   位定义取自正点原子官方 xl9555.h(openedv/ATK-DNESP32S3-Board)。
// ============================================================================
#define BSP_XIO_ADDR           0x20
#define BSP_XIO_BL_PIN         8    // P1_0 LCD_BL     背光(高有效)
#define BSP_XIO_SPK_PIN        5    // P0_5 SPK_CTRL   功放使能/ES8311 探测
#define BSP_XIO_KEY0_PIN       15   // P1_7 KEY0
#define BSP_XIO_KEY1_PIN       14   // P1_6 KEY1
#define BSP_XIO_KEY2_PIN       13   // P1_5 KEY2
#define BSP_XIO_KEY3_PIN       12   // P1_4 KEY3
// 方向:P0=0x1B(P0_0/1/3/4 输入,其余输出) P1=0xFE(P1_0 输出,其余输入)
#define BSP_XIO_DIR_P0         0x1B
#define BSP_XIO_DIR_P1         0xFE
