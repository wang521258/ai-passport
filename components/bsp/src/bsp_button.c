// components/bsp/src/bsp_button.c
// 正点原子 DN-ESP32-S3-BOX3 三键:BOOT(GPIO0 直连) + K1/K2(经 AW9523 扩展 IO)。
// 映射 BOOT→OK、K1→UP、K2→DOWN,宠物/菜单逻辑零改动。
#include "bsp_button.h"
#include "bsp_pins.h"
#include "bsp_xio.h"
#include "iot_button.h"
#include "esp_log.h"

static const char *TAG = "bsp_btn";

static button_handle_t s_btn[BSP_BTN_COUNT];
static bsp_btn_cb_t    s_cb;
static void           *s_user;

// 按键回调运行在 button 组件的任务里;用 usr_data 传"哪个键"(bsp_btn_t)。
static void on_event(void *arg, void *usr_data, bsp_btn_ev_t ev) {
    (void)arg;
    if (!s_cb) return;
    s_cb((bsp_btn_t)(intptr_t)usr_data, ev, s_user);
}
static void cb_press (void *a, void *u) { on_event(a, u, BSP_BTN_PRESS);  }
static void cb_click (void *a, void *u) { on_event(a, u, BSP_BTN_CLICK);  }
static void cb_double(void *a, void *u) { on_event(a, u, BSP_BTN_DOUBLE); }
static void cb_long  (void *a, void *u) { on_event(a, u, BSP_BTN_LONG);   }

// K1/K2 经 AW9523,均为低有效(按下=0),与 BOOT 键一致。
// 若真机上某键"按了没反应/松了才触发",说明该键实际高有效 —— 把对应
// 那行的 "? 1 : 0" 翻成 "? 0 : 1" 即可(一行极性翻转)。
static uint8_t k1_get_key_level(button_driver_t *drv) {
    (void)drv; return bsp_xio_get_key(BSP_BTN_K1_XIO) ? 1 : 0;
}
static uint8_t k2_get_key_level(button_driver_t *drv) {
    (void)drv; return bsp_xio_get_key(BSP_BTN_K2_XIO) ? 1 : 0;
}

static void reg(bsp_btn_t btn) {
    iot_button_register_cb(s_btn[btn], BUTTON_PRESS_DOWN,      NULL, cb_press, (void *)(intptr_t)btn);
    iot_button_register_cb(s_btn[btn], BUTTON_SINGLE_CLICK,    NULL, cb_click, (void *)(intptr_t)btn);
    iot_button_register_cb(s_btn[btn], BUTTON_DOUBLE_CLICK,    NULL, cb_double, (void *)(intptr_t)btn);
    iot_button_register_cb(s_btn[btn], BUTTON_LONG_PRESS_START,NULL, cb_long,  (void *)(intptr_t)btn);
}

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user) {
    s_cb = cb; s_user = user;

    esp_err_t e;

    // BOOT → OK(低有效,直连 GPIO0)
    button_gpio_config_t boot_cfg = {
        .gpio_num = BSP_BTN_BOOT_GPIO,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };
    if ((e = iot_button_new_gpio_device(&(button_config_t){0}, &boot_cfg, &s_btn[BSP_BTN_OK])) != ESP_OK) {
        ESP_LOGE(TAG, "BOOT 键创建失败 (%s)", esp_err_to_name(e));
        return e;
    }

    // K1 → UP(自定义 driver 读 AW9523)
    button_driver_t *k1_drv = calloc(1, sizeof(button_driver_t));
    k1_drv->enable_power_save = false;
    k1_drv->get_key_level = k1_get_key_level;
    if ((e = iot_button_create(&(button_config_t){0}, k1_drv, &s_btn[BSP_BTN_UP])) != ESP_OK) {
        ESP_LOGE(TAG, "K1 键创建失败 (%s)", esp_err_to_name(e));
        return e;
    }

    // K2 → DOWN
    button_driver_t *k2_drv = calloc(1, sizeof(button_driver_t));
    k2_drv->enable_power_save = false;
    k2_drv->get_key_level = k2_get_key_level;
    if ((e = iot_button_create(&(button_config_t){0}, k2_drv, &s_btn[BSP_BTN_DOWN])) != ESP_OK) {
        ESP_LOGE(TAG, "K2 键创建失败 (%s)", esp_err_to_name(e));
        return e;
    }

    reg(BSP_BTN_OK);
    reg(BSP_BTN_UP);
    reg(BSP_BTN_DOWN);

    ESP_LOGI(TAG, "按键就绪:BOOT(OK) + K1(UP) + K2(DOWN)");
    return ESP_OK;
}

// box3 无 ADC 分压键,保留接口返回 -1(demo 的电压显示页会用,宠物页不用)。
int bsp_button_read_mv(void) { return -1; }
