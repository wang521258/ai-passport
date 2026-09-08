/**
 * @file gif_impl.c
 * @brief 编译 gif.inl 为独立目标文件，提供 GIF 解码函数符号
 *
 * gif.inl 是 AnimatedGIF 库的纯 C 解码核心，原本设计为被 .cpp 包含。
 * 但我们的 gif_player.c 是 C 文件，直接 #include "gif.inl" 会导致
 * 链接阶段找不到函数符号（可能因编译器优化或符号可见性问题）。
 * 通过创建独立的 .c 文件包含 gif.inl，强制编译为独立目标文件。
 */
#ifndef memcpy_P
#define memcpy_P memcpy
#endif
#include "gif.inl"
