/**
 * @file gif_impl.c
 * @brief 编译 gif.inl 为独立目标文件，提供 GIF 解码函数符号
 *
 * gif.inl 第 45 行的条件编译：
 *   #if defined( PICO_BUILD ) || defined( __LINUX__ ) || defined( __MCUXPRESSO )
 * 在 ESP-IDF 环境下这些宏都未定义，导致 GIF_playFrame/GIF_openRAM/GIF_reset
 * 等函数定义被跳过，链接时报 undefined reference。
 *
 * 定义 PICO_BUILD 让第 45-172 行的 C 函数实现被正确编译。
 * PICO_BUILD 在 gif.inl 中仅出现在第 45 行，无其他副作用。
 */
#ifndef PICO_BUILD
#define PICO_BUILD
#endif
#include "gif.inl"
