/**
 * @file ui_pet.h
 * @brief 像素画风格电子宠物接口
 */
#pragma once
#include "lvgl.h"

/**
 * @brief 创建像素画宠物（圆滚滚粉色）
 * @param parent 父对象
 * @param x 左上角 X
 * @param y 左上角 Y
 * @return 宠物对象指针
 */
lv_obj_t *ui_pixel_pet_create(lv_obj_t *parent, int x, int y);

/**
 * @brief 宠物跳跃动画
 */
void ui_pixel_pet_jump(lv_obj_t *pet);

/**
 * @brief 表情枚举
 */
typedef enum {
    PET_FACE_NORMAL = 0,
    PET_FACE_HAPPY,
    PET_FACE_CRY,
    PET_FACE_SLEEP,
    PET_FACE_HUNGRY,
    PET_FACE_ANGRY,
} pet_face_t;

/**
 * @brief 设置宠物表情
 */
void ui_pixel_pet_set_face(lv_obj_t *pet, pet_face_t face);

/* ---------- 精灵球 ---------- */
lv_obj_t *ui_pixel_ball_create(lv_obj_t *parent, int x, int y);
void ui_pixel_ball_shake(lv_obj_t *ball, int level);  /* level: 1~3 */
void ui_pixel_ball_open(lv_obj_t *ball);               /* 爆开并自删 */
