/**
 * @file ui_pet.h
 * @brief 像素画风宠物 UI：圆滚滚宠物 + 像素风精灵蛋
 */
#pragma once
#include "lvgl.h"

/* ---------- 像素宠物（粉色圆滚滚，替代品，未实际使用但保留接口） ---------- */
lv_obj_t *ui_pixel_pet_create(lv_obj_t *parent, int x, int y);
void ui_pixel_pet_jump(lv_obj_t *pet);
typedef enum {
    PET_FACE_NORMAL = 0,
    PET_FACE_HAPPY,
    PET_FACE_CRY,
    PET_FACE_SLEEP,
    PET_FACE_HUNGRY,
    PET_FACE_ANGRY,
} pet_face_t;
void ui_pixel_pet_set_face(lv_obj_t *pet, pet_face_t face);

/* ---------- 精灵蛋（宝可梦风，米白椭圆 + 绿色斑点 + 描边） ----------
 * 实现方式：lv_canvas 72×90，RGB565 逐像素填色。
 * 蛋大小：72×90（比原来的 24×24 精灵球大 2.7×，好看很多） */
#define EGG_W 72
#define EGG_H 90

/**
 * @brief 创建像素风精灵蛋
 * @return lv_canvas 对象（按 x,y 摆放, 不占用父对象布局）
 */
lv_obj_t *ui_pixel_egg_create(lv_obj_t *parent, int x, int y);

/**
 * @brief 摇晃动画（破壳点击时调用）
 * @param level 1~3，摇晃幅度递增
 */
void ui_pixel_egg_shake(lv_obj_t *egg, int level);

/**
 * @brief 销毁蛋对象（破壳后调用）
 */
void ui_pixel_egg_destroy(lv_obj_t *egg);