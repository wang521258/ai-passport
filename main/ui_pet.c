/**
 * @file ui_pet.c
 * @brief 像素画风格电子宠物（适配 FoloToy AI Passport / LVGL）
 *
 * 用 block() 拼像素块的方式绘制圆滚滚粉色宠物，
 * 支持眨眼、跳跃动画和多种表情切换。
 */
#include "ui_pixel.h"
#include "ui_pet.h"
#include "lvgl.h"

/* 宠物颜色 */
#define PET_BODY    0xF8BBD0   /* 身体：樱花粉 */
#define PET_BODY_D  0xE89AB8   /* 身体暗色（阴影） */
#define PET_EAR_IN  0xF5A0BC   /* 内耳 */
#define PET_CHEEK   0xFF8FA8   /* 腮红 */
#define PET_INK     0x17202A   /* 黑色轮廓/眼睛 */
#define PET_HI      0xFFFFFF   /* 高光 */
#define PET_NOSE    0xE87090   /* 鼻子 */
#define PET_TONGUE  0xFF6B8A   /* 舌头 */

/* 宠物对象：保存各部位指针以便动画/表情切换 */
typedef struct {
    lv_obj_t *root;
    lv_obj_t *left_eye;
    lv_obj_t *right_eye;
    lv_obj_t *mouth;
    lv_obj_t *left_cheek;
    lv_obj_t *right_cheek;
    lv_obj_t *left_tear;
    lv_obj_t *right_tear;
} pet_obj_t;

/* ---------- 内部辅助 ---------- */

static void pet_blink_cb(void *obj, int32_t value)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

static void pet_start_blink(lv_obj_t *eye)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, eye);
    lv_anim_set_exec_cb(&a, pet_blink_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_10);
    lv_anim_set_duration(&a, 80);
    lv_anim_set_playback_duration(&a, 80);
    lv_anim_set_repeat_delay(&a, 1800);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_step);
    lv_anim_start(&a);
}

static void pet_jump_cb(void *obj, int32_t value)
{
    lv_obj_set_y((lv_obj_t *)obj, value);
}

/* ---------- 公开 API ---------- */

/**
 * @brief 创建像素画宠物
 * @param parent 父对象
 * @param x 左上角 X
 * @param y 左上角 Y
 * @return 宠物对象指针（用于动画/表情）
 */
lv_obj_t *ui_pixel_pet_create(lv_obj_t *parent, int x, int y)
{
    pet_obj_t *p = lv_malloc(sizeof(pet_obj_t));
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(root, x, y);
    lv_obj_set_size(root, 44, 54);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_user_data(root, p);

    /* 耳朵 */
    block(root, 13, 0, 5, 6, PET_INK);
    block(root, 26, 0, 5, 6, PET_INK);
    block(root, 14, 1, 3, 4, PET_BODY);
    block(root, 27, 1, 3, 4, PET_BODY);
    block(root, 15, 2, 1, 2, PET_EAR_IN);
    block(root, 28, 2, 1, 2, PET_EAR_IN);

    /* 身体轮廓（黑色描边 + 粉色填充） */
    /* 用多行 block 拼出椭圆身体 */
    block(root, 10, 6,  24, 2, PET_INK);
    block(root, 6,  8,  32, 2, PET_INK);
    block(root, 4,  10, 36, 22, PET_INK);
    block(root, 6,  32, 32, 2, PET_INK);
    block(root, 10, 34, 24, 2, PET_INK);

    /* 粉色填充 */
    block(root, 11, 7,  22, 1, PET_BODY);
    block(root, 7,  9,  30, 1, PET_BODY);
    block(root, 5,  11, 34, 20, PET_BODY);
    block(root, 7,  31, 30, 1, PET_BODY);
    block(root, 11, 33, 22, 1, PET_BODY);

    /* 头顶高光 */
    block(root, 10, 12, 8, 3, PET_HI);
    block(root, 10, 12, 5, 1, PET_HI);

    /* 腮红 */
    p->left_cheek  = block(root, 7,  22, 5, 3, PET_CHEEK);
    p->right_cheek = block(root, 32, 22, 5, 3, PET_CHEEK);

    /* 眼睛 */
    p->left_eye  = block(root, 12, 18, 5, 6, PET_INK);
    p->right_eye = block(root, 27, 18, 5, 6, PET_INK);
    /* 眼睛高光 */
    block(root, 13, 19, 2, 2, PET_HI);
    block(root, 28, 19, 2, 2, PET_HI);

    /* 鼻子 */
    block(root, 21, 24, 2, 2, PET_NOSE);

    /* 嘴巴（默认微笑） */
    p->mouth = block(root, 19, 28, 6, 2, PET_INK);
    block(root, 19, 28, 1, 1, PET_INK);
    block(root, 24, 28, 1, 1, PET_INK);

    /* 眼泪（默认隐藏） */
    p->left_tear  = block(root, 11, 25, 2, 4, 0x4FC3F7);
    p->right_tear = block(root, 31, 25, 2, 4, 0x4FC3F7);
    lv_obj_add_flag(p->left_tear, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->right_tear, LV_OBJ_FLAG_HIDDEN);

    /* 脚 */
    block(root, 12, 36, 7, 5, PET_INK);
    block(root, 25, 36, 7, 5, PET_INK);
    block(root, 13, 37, 5, 3, PET_BODY);
    block(root, 26, 37, 5, 3, PET_BODY);
    block(root, 14, 39, 3, 1, PET_EAR_IN);
    block(root, 27, 39, 3, 1, PET_EAR_IN);

    /* 启动眨眼 */
    pet_start_blink(p->left_eye);
    pet_start_blink(p->right_eye);

    return root;
}

/**
 * @brief 宠物跳跃动画
 */
void ui_pixel_pet_jump(lv_obj_t *pet)
{
    if (!pet) return;
    int y = lv_obj_get_y(pet);
    lv_anim_delete(pet, pet_jump_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, pet);
    lv_anim_set_exec_cb(&a, pet_jump_cb);
    lv_anim_set_values(&a, y, y - 6);
    lv_anim_set_duration(&a, 120);
    lv_anim_set_playback_duration(&a, 150);
    lv_anim_set_path_cb(&a, lv_anim_path_step);
    lv_anim_start(&a);
}

/**
 * @brief 设置宠物表情
 */
void ui_pixel_pet_set_face(lv_obj_t *pet, pet_face_t face)
{
    if (!pet) return;
    pet_obj_t *p = lv_obj_get_user_data(pet);
    if (!p) return;

    /* 先重置嘴巴 */
    lv_obj_set_pos(p->mouth, 19, 28);
    lv_obj_set_size(p->mouth, 6, 2);
    lv_obj_remove_flag(p->mouth, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->left_tear, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(p->right_tear, LV_OBJ_FLAG_HIDDEN);

    switch (face) {
        case PET_FACE_HAPPY:
            /* 弯眼笑：眼睛变窄 */
            lv_obj_set_size(p->left_eye, 5, 2);
            lv_obj_set_size(p->right_eye, 5, 2);
            lv_obj_set_pos(p->left_eye, 12, 21);
            lv_obj_set_pos(p->right_eye, 27, 21);
            /* 大笑嘴 */
            lv_obj_set_pos(p->mouth, 18, 28);
            lv_obj_set_size(p->mouth, 8, 4);
            break;
        case PET_FACE_CRY:
            lv_obj_set_size(p->left_eye, 5, 1);
            lv_obj_set_size(p->right_eye, 5, 1);
            lv_obj_set_pos(p->left_eye, 12, 21);
            lv_obj_set_pos(p->right_eye, 27, 21);
            /* 倒嘴 */
            lv_obj_set_pos(p->mouth, 19, 30);
            lv_obj_set_size(p->mouth, 6, 1);
            /* 眼泪 */
            lv_obj_remove_flag(p->left_tear, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(p->right_tear, LV_OBJ_FLAG_HIDDEN);
            break;
        case PET_FACE_SLEEP:
            /* 闭眼 */
            lv_obj_set_size(p->left_eye, 5, 1);
            lv_obj_set_size(p->right_eye, 5, 1);
            lv_obj_set_pos(p->left_eye, 12, 21);
            lv_obj_set_pos(p->right_eye, 27, 21);
            /* 平嘴 */
            lv_obj_set_size(p->mouth, 4, 1);
            break;
        case PET_FACE_HUNGRY:
            /* 圆眼 + 馋嘴 */
            lv_obj_set_size(p->left_eye, 5, 6);
            lv_obj_set_size(p->right_eye, 5, 6);
            lv_obj_set_pos(p->left_eye, 12, 18);
            lv_obj_set_pos(p->right_eye, 27, 18);
            /* 舔嘴 */
            lv_obj_set_pos(p->mouth, 19, 28);
            lv_obj_set_size(p->mouth, 6, 2);
            break;
        case PET_FACE_ANGRY:
            lv_obj_set_size(p->left_eye, 5, 4);
            lv_obj_set_size(p->right_eye, 5, 4);
            lv_obj_set_pos(p->left_eye, 12, 20);
            lv_obj_set_pos(p->right_eye, 27, 20);
            /* 波浪嘴 */
            lv_obj_set_pos(p->mouth, 18, 29);
            lv_obj_set_size(p->mouth, 8, 2);
            break;
        default: /* NORMAL */
            lv_obj_set_size(p->left_eye, 5, 6);
            lv_obj_set_size(p->right_eye, 5, 6);
            lv_obj_set_pos(p->left_eye, 12, 18);
            lv_obj_set_pos(p->right_eye, 27, 18);
            lv_obj_set_pos(p->mouth, 19, 28);
            lv_obj_set_size(p->mouth, 6, 2);
    }
}

/* ==================== 精灵球 ==================== */
#define BALL_RED   0xE53935
#define BALL_WHITE 0xFFFFFF
#define BALL_INK   0x17202A

/**
 * @brief 创建像素精灵球（24×24），坐标与 HTML 原型一致
 */
lv_obj_t *ui_pixel_ball_create(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(root, x, y);
    lv_obj_set_size(root, 24, 24);
    lv_obj_set_style_radius(root, 0, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);

    /* 红色上半球 */
    block(root, 8, 1, 8, 1, BALL_RED);
    block(root, 6, 2, 12, 1, BALL_RED);
    block(root, 5, 3, 14, 1, BALL_RED);
    block(root, 4, 4, 16, 1, BALL_RED);
    block(root, 3, 5, 18, 1, BALL_RED);
    block(root, 2, 6, 20, 1, BALL_RED);
    block(root, 2, 7, 20, 1, BALL_RED);
    block(root, 3, 8, 18, 1, BALL_RED);
    block(root, 4, 9, 16, 1, BALL_RED);
    block(root, 5, 10, 14, 1, BALL_RED);
    /* 黑色中缝 + 白色按钮 */
    block(root, 2, 11, 20, 2, BALL_INK);
    block(root, 9, 11, 6, 2, BALL_WHITE);
    /* 白色下半球 */
    block(root, 5, 13, 14, 1, BALL_WHITE);
    block(root, 4, 14, 16, 1, BALL_WHITE);
    block(root, 3, 15, 18, 1, BALL_WHITE);
    block(root, 2, 16, 20, 1, BALL_WHITE);
    block(root, 2, 17, 20, 1, BALL_WHITE);
    block(root, 3, 18, 18, 1, BALL_WHITE);
    block(root, 4, 19, 16, 1, BALL_WHITE);
    block(root, 5, 20, 14, 1, BALL_WHITE);
    block(root, 6, 21, 12, 1, BALL_WHITE);
    block(root, 8, 22, 8, 1, BALL_WHITE);
    /* 黑色外轮廓 */
    block(root, 8, 0, 8, 1, BALL_INK);
    block(root, 7, 1, 1, 1, BALL_INK);  block(root, 16, 1, 1, 1, BALL_INK);
    block(root, 5, 2, 1, 1, BALL_INK);  block(root, 18, 2, 1, 1, BALL_INK);
    block(root, 4, 3, 1, 1, BALL_INK);  block(root, 19, 3, 1, 1, BALL_INK);
    block(root, 3, 4, 1, 1, BALL_INK);  block(root, 20, 4, 1, 1, BALL_INK);
    block(root, 2, 5, 1, 1, BALL_INK);  block(root, 21, 5, 1, 1, BALL_INK);
    block(root, 1, 6, 1, 2, BALL_INK);  block(root, 22, 6, 1, 2, BALL_INK);
    block(root, 2, 8, 1, 1, BALL_INK);  block(root, 21, 8, 1, 1, BALL_INK);
    block(root, 3, 9, 1, 1, BALL_INK);  block(root, 20, 9, 1, 1, BALL_INK);
    block(root, 4, 10, 1, 1, BALL_INK); block(root, 19, 10, 1, 1, BALL_INK);
    block(root, 4, 13, 1, 1, BALL_INK); block(root, 19, 13, 1, 1, BALL_INK);
    block(root, 3, 14, 1, 1, BALL_INK); block(root, 20, 14, 1, 1, BALL_INK);
    block(root, 2, 15, 1, 1, BALL_INK); block(root, 21, 15, 1, 1, BALL_INK);
    block(root, 1, 16, 1, 2, BALL_INK); block(root, 22, 16, 1, 2, BALL_INK);
    block(root, 2, 18, 1, 1, BALL_INK); block(root, 21, 18, 1, 1, BALL_INK);
    block(root, 3, 19, 1, 1, BALL_INK); block(root, 20, 19, 1, 1, BALL_INK);
    block(root, 4, 20, 1, 1, BALL_INK); block(root, 19, 20, 1, 1, BALL_INK);
    block(root, 5, 21, 1, 1, BALL_INK); block(root, 18, 21, 1, 1, BALL_INK);
    block(root, 7, 22, 1, 1, BALL_INK); block(root, 16, 22, 1, 1, BALL_INK);
    block(root, 8, 23, 8, 1, BALL_INK);

    return root;
}

static void ball_shake_cb(void *var, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)var, v);
}

/**
 * @brief 精灵球摇晃动画
 * @param level 1~3，等级越高摇晃幅度越大
 */
void ui_pixel_ball_shake(lv_obj_t *ball, int level)
{
    if (!ball) return;
    int base_x = lv_obj_get_x(ball);
    int amp = 3 + level * 2;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ball);
    lv_anim_set_exec_cb(&a, ball_shake_cb);
    lv_anim_set_values(&a, base_x - amp, base_x + amp);
    lv_anim_set_time(&a, 80 + level * 30);
    lv_anim_set_playback_time(&a, 80 + level * 30);
    lv_anim_set_repeat_count(&a, 2 + level);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

static void ball_open_ready_cb(lv_anim_t *a)
{
    lv_obj_t *ball = a->var;
    if (ball) lv_obj_delete(ball);
}

static void ball_opa_cb(void *var, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)var, (lv_opa_t)v, 0);
}

/* 爆开时通过改 transform_scale 做放大效果（LVGL 9 用 style 实现）*/
static void ball_scale_cb(void *var, int32_t v)
{
    lv_obj_t *obj = (lv_obj_t *)var;
    /* v 是 256..640，对应 scale 256..640（256=1.0x）*/
    lv_obj_set_style_transform_scale(obj, v, 0);
    /* 同时把 pivot 移到中心，保证从中心放大 */
    lv_obj_set_style_transform_pivot_x(obj, 12, 0);
    lv_obj_set_style_transform_pivot_y(obj, 12, 0);
}

/**
 * @brief 精灵球爆开（放大淡出并自删）
 */
void ui_pixel_ball_open(lv_obj_t *ball)
{
    if (!ball) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ball);
    lv_anim_set_exec_cb(&a, ball_scale_cb);
    lv_anim_set_values(&a, 256, 640);
    lv_anim_set_time(&a, 400);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, ball_open_ready_cb);
    lv_anim_start(&a);
    lv_anim_t b;
    lv_anim_init(&b);
    lv_anim_set_var(&b, ball);
    lv_anim_set_exec_cb(&b, ball_opa_cb);
    lv_anim_set_values(&b, 255, 0);
    lv_anim_set_time(&b, 400);
    lv_anim_start(&b);
}
