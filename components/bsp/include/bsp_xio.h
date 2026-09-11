// components/bsp/include/bsp_xio.h
// AW9523 I/O 扩展器(TCA95xx 兼容寄存器)封装:板上多路电源使能 + 背光 + 功放 + K1/K2 键。
// 直接走 bsp_i2c 已建好的 I2C 总线,不引入 esp_io_expander 组件依赖。
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// 初始化 AW9523:配置 GPIO 模式、方向,并拉高所有电源使能、点亮背光。
// 必须在 bsp_i2c_init() 之后调用。幂等。
esp_err_t bsp_xio_init(void);

// 背光:on=1 点亮(低有效,内部写 P0_8 = 0)
void bsp_xio_set_bl(uint8_t on);

// 读扩展 IO 输入 pin(0..15)电平,1 = 高电平。用于 K1/K2 按键检测。
bool bsp_xio_get_key(uint8_t xio_pin);

// 功放使能:on=1 打开(拉高 P0_5)
void bsp_xio_set_pa(uint8_t on);
