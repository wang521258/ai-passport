/**
 * @file gif_player.h
 * @brief 基于 AnimatedGIF + LVGL canvas 的 GIF 播放器
 */
#pragma once
#include "lvgl.h"
#include <stdint.h>

/**
 * @brief 创建 GIF 播放器（返回 LVGL canvas 对象）
 */
lv_obj_t *gif_player_create(lv_obj_t *parent, int x, int y, int w, int h);

/**
 * @brief 播放指定 GIF 数据（从内存）
 */
void gif_player_play(lv_obj_t *canvas, const uint8_t *data, int len);

/**
 * @brief 停止播放（释放定时器）
 */
void gif_player_stop(lv_obj_t *canvas);
