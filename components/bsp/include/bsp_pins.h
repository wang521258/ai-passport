// components/bsp/include/bsp_pins.h
// FoloToy AI Passport → 正点原子 DN-ESP32-S3-BOX3 硬件引脚【单一事实来源】。
// 板子:ESP32-S3 (octal PSRAM 8MB) / ST7789 320x240 SPI 屏 / AW9523 扩展 IO /
//       ES8311 音频 / 三键(BOOT 直连 + K1/K2 经 AW9523)。
// 引脚全部取自正点原子小智固件 board 定义(atk-dnesp32s3-box3),实测一致。
#pragma once

#include "driver/spi_master.h"
#include "driver/i2c_types.h"
#include "hal/adc_types.h"

// ============================================================================
// 显示:ST7789 320x240,4-line SPI(玻璃为横屏;LVGL 用 swap_xy 转竖屏 240x320)
// ============================================================================
#define BSP_LCD_W            240
#define BSP_LCD_H            320
#define BSP_LCD_SPI_HOST     SPI2_HOST
#define BSP_LCD_MOSI         16
#define BSP_LCD_SCLK         15
#define BSP_LCD_MISO         17
#define BSP_LCD_CS           47
#define BSP_LCD_DC           48
#define BSP_LCD_RST          (-1)   // 复位走 SWRESET 软复位
// 背光经 AW9523 扩展 IO(P0_8,低有效),非直连 LEDC。-1 仅作占位供日志打印。
#define BSP_LCD_BL           (-1)
#define BSP_LCD_PCLK_HZ      (60 * 1000 * 1000)
#define BSP_LCD_SPI_MODE     0
#define BSP_LCD_INVERT_COLOR 1      // 本屏出厂需反色(0x21 INVON)

// ============================================================================
// 按键:三键。BOOT 直连 GPIO0;K1/K2 经 AW9523 扩展 IO(P0_0 / P0_1)。
//   映射:BOOT→OK,K1→UP,K2→DOWN(宠物/菜单逻辑零改动)。
// ============================================================================
#define BSP_BTN_COUNT        3
#define BSP_BTN_BOOT_GPIO    0      // BOOT 键,低有效(按下=0)
#define BSP_BTN_K1_XIO       0      // AW9523 P0_0,低有效(按下=0)
#define BSP_BTN_K2_XIO       1      // AW9523 P0_1,低有效(按下=0)

// ============================================================================
// I2C:ES8311(音频)与 AW9523(扩展 IO)共用一条总线
// ============================================================================
#define BSP_I2C_PORT         I2C_NUM_0
#define BSP_I2C_SDA          3
#define BSP_I2C_SCL          2
#define BSP_I2C_ES8311_ADDR  0x18    // 7 位地址
#define BSP_I2C_CW2017_ADDR  0x63    // box3 无独立电量计,init 失败无害

// ============================================================================
// 音频:ES8311,I2S 全双工(同端口一 tx 一 rx,共用 MCLK/BCLK/WS)
// ============================================================================
#define BSP_I2S_PORT         I2S_NUM_0
#define BSP_I2S_MCLK         21
#define BSP_I2S_BCLK         38
#define BSP_I2S_WS           39
#define BSP_I2S_DOUT         40      // 播放:MCU → codec
#define BSP_I2S_DIN          41      // 录音:codec → MCU
// 功放使能经 AW9523 P0_5,由 bsp_xio 控制;codec 自身不控 PA。
#define BSP_I2S_PA_CTRL      (-1)

// ============================================================================
// AW9523 扩展 IO(I2C 地址 0x59):管背光 / 功放 / K1-K2 键 / 板上多路电源使能
// ============================================================================
#define BSP_XIO_ADDR         0x59
#define BSP_XIO_BL_PIN       8       // P0_8 LCD 背光(低有效)
#define BSP_XIO_PA_PIN       5       // P0_5 音频功放
#define BSP_XIO_K1_PIN       0       // P0_0
#define BSP_XIO_K2_PIN       1       // P0_1
