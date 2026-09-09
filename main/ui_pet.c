/**
 * @file ui_pet.c
 * @brief 精灵球与像素宠物的实现（ui_pet.h 的接口）
 *
 * 原工程只有 ui_pet.h，缺少对应的 .c 实现，而 demo_pet.c 会调用
 * ui_pixel_ball_create / ui_pixel_ball_shake / ui_pixel_ball_open，
 * 直接编译会链接失败。本文件补齐这些实现。
 *
 * 注意内存：ESP32-C3 仅约 80 KB 动态 RAM，因此这里刻意只用少量
 * block() 色块（每个 block 都是一个 LVGL 对象），不做精细逐像素绘制。
 */
#include "ui_pet.h"
#include "ui_pixel.h"
#include "lvgl.h"
#include <stdint.h>

/* ---------- 内部工具 ---------- */
static void ball_set_x_cb(void *obj, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)obj, v);
}

static void ball_fade_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

static void ball_open_done_cb(lv_anim_t *a)
{
    if (a && a->var) lv_obj_delete((lv_obj_t *)a->var);
}

/* ---------- 精灵球 ---------- */
lv_obj_t *ui_pixel_ball_create(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *ball = lv_obj_create(parent);
    lv_obj_remove_flag(ball, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(ball, x, y);
    lv_obj_set_size(ball, 24, 24);
    lv_obj_set_style_bg_opa(ball, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ball, 0, 0);
    lv_obj_set_style_pad_all(ball, 0, 0);

    /* 记住基准 X，供 shake 反复调用时复位 */
    lv_obj_set_user_data(ball, (void *)(intptr_t)x);

    /* 上半球：红 */
    block(ball, 4, 2, 16, 7, 0xE03A2F);
    block(ball, 2, 4, 20, 5, 0xE03A2F);
    /* 下半球：白 */
    block(ball, 2, 13, 20, 5, 0xF4F4EA);
    block(ball, 4, 16, 16, 4, 0xF4F4EA);
    /* 中间黑带 */
    block(ball, 2, 10, 20, 3, UI_INK);
    /* 中心按钮：白圈 + 黑心 */
    block(ball, 8, 7, 8, 8, 0xF4F4EA);
    block(ball, 10, 9, 4, 4, UI_INK);
    /* 高光 */
    block(ball, 6, 4, 3, 2, 0xFFFFFF);

    return ball;
}

void ui_pixel_ball_shake(lv_obj_t *ball, int level)
{
    if (!ball) return;
    if (level < 1) level = 1;
    if (level > 3) level = 3;

    int base_x = (int)(intptr_t)lv_obj_get_user_data(ball);
    int amp = 2 + level * 2;   /* level 1~3 → 4/6/8 px */

    lv_anim_delete(ball, ball_set_x_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ball);
    lv_anim_set_exec_cb(&a, ball_set_x_cb);
    lv_anim_set_values(&a, base_x - amp, base_x + amp);
    lv_anim_set_duration(&a, 60);
    lv_anim_set_playback_duration(&a, 60);
    lv_anim_set_repeat_count(&a, level);
    lv_anim_set_path_cb(&a, lv_anim_path_step);
    lv_anim_start(&a);
}

void ui_pixel_ball_open(lv_obj_t *ball)
{
    if (!ball) return;

    /* 爆开：淡出后自删（不额外创建动画对象以外的资源） */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ball);
    lv_anim_set_exec_cb(&a, ball_fade_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_duration(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_completed_cb(&a, ball_open_done_cb);
    lv_anim_start(&a);
}

/* ---------- 像素宠物（圆滚滚粉色）----------
 * demo_pet.c 当前版本用 GIF 呈现宝可梦，未调用下面这组接口；
 * 这里保留实现，避免其他页面引用时出现链接错误。
 */
lv_obj_t *ui_pixel_pet_create(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *pet = lv_obj_create(parent);
    lv_obj_remove_flag(pet, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(pet, x, y);
    lv_obj_set_size(pet, 40, 40);
    lv_obj_set_style_bg_opa(pet, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(pet, 0, 0);
    lv_obj_set_style_pad_all(pet, 0, 0);

    /* 身体 */
    block(pet, 4, 6, 32, 28, 0xFFB6C1);
    /* 头顶 */
    block(pet, 8, 2, 24, 6, 0xFFB6C1);
    /* 眼睛 */
    block(pet, 12, 14, 5, 6, UI_INK);
    block(pet, 24, 14, 5, 6, UI_INK);
    /* 腮红 */
    block(pet, 8, 22, 5, 3, 0xFF8FA3);
    block(pet, 28, 22, 5, 3, 0xFF8FA3);
    /* 脚 */
    block(pet, 10, 34, 8, 4, 0xFF8FA3);
    block(pet, 24, 34, 8, 4, 0xFF8FA3);

    return pet;
}

void ui_pixel_pet_jump(lv_obj_t *pet)
{
    if (!pet) return;
    int y = lv_obj_get_y(pet);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, pet);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&a, y, y - 8);
    lv_anim_set_duration(&a, 120);
    lv_anim_set_playback_duration(&a, 150);
    lv_anim_set_path_cb(&a, lv_anim_path_step);
    lv_anim_start(&a);
}

void ui_pixel_pet_set_face(lv_obj_t *pet, pet_face_t face)
{
    if (!pet) return;
    /* 简化实现：通过整体透明度区分睡觉状态，其余表情保持默认笑脸 */
    if (face == PET_FACE_SLEEP) {
        lv_obj_set_style_opa(pet, LV_OPA_60, 0);
    } else {
        lv_obj_set_style_opa(pet, LV_OPA_COVER, 0);
    }
}
