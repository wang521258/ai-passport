// components/bsp/include/bsp_xio.h
// XL9555 I/O 扩展器封装:背光 + 功放使能 + 按键输入。
// 直接走 bsp_i2c 已建好的 I2C 总线,不引入 esp_io_expander 组件依赖。
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// 初始化 XL9555:配方向、拉高输出、点亮背光。须在 bsp_i2c_init() 之后调用。幂等。
esp_err_t bsp_xio_init(void);

// 背光:P1_0,高有效。on=1 点亮。
void bsp_xio_set_bl(uint8_t on);

// 读扩展 IO 输入 pin(0..15)电平,1 = 高电平。
// 线性编号:0..7 = P0_0..P0_7,8..15 = P1_0..P1_7。
bool bsp_xio_get_key(uint8_t xio_pin);

// 一次读回 16 位输入寄存器(按键诊断用)。成功返回 true。
bool bsp_xio_read_all(uint16_t *in);

// 功放使能:P0_5,高有效。
void bsp_xio_set_pa(uint8_t on);
