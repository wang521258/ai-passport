/**
 * @file gif_player.h
 * @brief 基于 AnimatedGIF + LVGL canvas 的 GIF 播放器接口
 *
 * 原工程只提供了 gif_player.c，缺少对应头文件，导致 demo_pet.c 编译时
 * 找不到 gif_player_create / gif_player_play / gif_player_stop 的声明。
 * 本头文件按 gif_player.c 的实际实现补齐。
 */
#ifndef GIF_PLAYER_H
#define GIF_PLAYER_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 GIF 播放画布
 * @param parent 父对象
 * @param x 左上角 X
 * @param y 左上角 Y
 * @param w 画布宽度（像素）
 * @param h 画布高度（像素）
 * @return 画布对象指针（播放器的句柄即挂在它的 user_data 上）
 */
lv_obj_t *gif_player_create(lv_obj_t *parent, int x, int y, int w, int h);

/**
 * @brief 在指定画布上播放内存中的 GIF 数据
 * @param canvas gif_player_create 返回的画布
 * @param data GIF 二进制数据（通常位于 Flash，不会占用 RAM）
 * @param len 数据长度
 */
void gif_player_play(lv_obj_t *canvas, const uint8_t *data, int len);

/**
 * @brief 停止播放并释放解码器
 * @param canvas gif_player_create 返回的画布
 */
void gif_player_stop(lv_obj_t *canvas);

#ifdef __cplusplus
}
#endif

#endif /* GIF_PLAYER_H */
