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
#include <stdint.h>
#include <stdbool.h>

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
/* 0=失败(参数/内存/GIF解析), 1=成功启动 */
int  gif_player_play(lv_obj_t *canvas, const uint8_t *data, int len);

/**
 * @brief 停止播放并释放解码器
 * @param canvas gif_player_create 返回的画布
 */
void gif_player_stop(lv_obj_t *canvas);

/**
 * @brief 彻底销毁单例播放器并释放 GIFIMAGE / 画布缓冲
 *
 * 退出宠物玩法页时调用。播放器是全局单例（GIFIMAGE 约 24KB），
 * 不释放会一直占着 ESP32-C3 本就紧张的动态堆。
 */
void gif_player_destroy(void);

/**
 * @brief 注册"背景取色"回调，消除 canvas 的白底方块
 *
 * LVGL 的 RGB565 canvas 没有 alpha 通道，清空画布只能填一个纯色，
 * 于是宠物周围会出现一块和场景不符的白色方块。
 * 注册该回调后，播放器每帧按画布的屏幕绝对坐标逐像素取场景背景色
 * 填充，宠物就像直接站在背景上，方块消失。
 * @param fn 入参为屏幕绝对坐标 (x, y)，返回 RGB565 颜色
 */
void gif_player_set_bg_fn(uint16_t (*fn)(int x, int y));

/**
 * @brief 开启/关闭宠物上下轻微浮动（呼吸感）
 *
 * 浮动在解码绘制阶段给整只宠物加纵向偏移，背景不动，
 * 幅度 2px、8 帧一个周期，与 GIF 帧率同步，不额外占内存。
 */
void gif_player_set_bob(lv_obj_t *canvas, bool enable);

/**
 * @brief 移动画布（同时更新背景取色的坐标基准）
 */
void gif_player_set_pos(lv_obj_t *canvas, int x, int y);

#ifdef __cplusplus
}
#endif

#endif /* GIF_PLAYER_H */
