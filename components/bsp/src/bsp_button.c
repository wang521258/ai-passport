// components/bsp/src/bsp_button.c
// 正点原子 DN-ESP32-S3-BOX3 三键:BOOT(GPIO0 直连) + K1/K2(经 AW9523 扩展 IO)。
// 映射 BOOT→OK、K1→UP、K2→DOWN,宠物/菜单逻辑零改动。
//
// 不依赖 esp-button 组件(其 API 在 v4 大改,易编译不过);自行用 FreeRTOS 任务
// 轮询三键电平,做去抖 + 事件派发(PRESS/CLICK/LONG)。应用只需 BSP_BTN_CLICK
// 与 BSP_BTN_LONG,其余事件发出无害。
#include "bsp_button.h"
#include "bsp_pins.h"
#include "bsp_xio.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_btn";

static bsp_btn_cb_t s_cb;
static void        *s_user;

#define BTN_TICK_MS      10
#define BTN_LONG_MS      800
#define BTN_DEBOUNCE     2      // 连续 2 个 tick(20ms)稳定才确认

typedef struct {
    bsp_btn_t   id;
    bool        active_low;      // true: 电平=0 表示按下
    bool      (*read)(void);     // 读物理电平(1=高)
    int         deb;             // 去抖计数器
    bool        is_down;         // 去抖后的当前按下态
    bool        was_down;        // 上一拍按下态(用于边沿判定)
    uint32_t    t_down_ms;       // 按下的系统时间(ms)
    bool        long_fired;
} btn_state_t;

static btn_state_t s_btn[BSP_BTN_COUNT];

static bool read_boot(void) { return (bool)gpio_get_level(BSP_BTN_BOOT_GPIO); }
static bool read_k1(void)   { return bsp_xio_get_key(BSP_BTN_K1_XIO); }
static bool read_k2(void)   { return bsp_xio_get_key(BSP_BTN_K2_XIO); }

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static void btn_tick(btn_state_t *b) {
    bool level = b->read();
    bool down  = b->active_low ? !level : level;

    // 去抖:向下计正、向上计负,稳态窗口内才算确认
    if (down) { if (b->deb < BTN_DEBOUNCE) b->deb++; }
    else      { if (b->deb > 0)            b->deb--; }
    bool pressed = (b->deb >= BTN_DEBOUNCE);

    if (pressed && !b->was_down) {
        // 下降沿:按下
        b->was_down   = true;
        b->t_down_ms  = now_ms();
        b->long_fired = false;
        if (s_cb) s_cb(b->id, BSP_BTN_PRESS, s_user);
    } else if (!pressed && b->was_down) {
        // 上升沿:抬起
        b->was_down = false;
        uint32_t dur = now_ms() - b->t_down_ms;
        if (!b->long_fired && dur < BTN_LONG_MS) {
            if (s_cb) s_cb(b->id, BSP_BTN_CLICK, s_user);
        }
        // long_fired 为真时代表已发过 LONG,这里不再发 CLICK,避免重复触发
    }

    // 长按:按住达到阈值立即发一次
    if (pressed && !b->long_fired) {
        uint32_t dur = now_ms() - b->t_down_ms;
        if (dur >= BTN_LONG_MS) {
            b->long_fired = true;
            if (s_cb) s_cb(b->id, BSP_BTN_LONG, s_user);
        }
    }
}

static void btn_task(void *arg) {
    (void)arg;
    for (;;) {
        for (int i = 0; i < BSP_BTN_COUNT; i++) btn_tick(&s_btn[i]);
        vTaskDelay(pdMS_TO_TICKS(BTN_TICK_MS));
    }
}

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user) {
    s_cb = cb; s_user = user;

    // BOOT(GPIO0) 配输入 + 上拉(板上一般有外部上拉,内部再补一层保险)
    gpio_reset_pin(BSP_BTN_BOOT_GPIO);
    gpio_set_direction(BSP_BTN_BOOT_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BSP_BTN_BOOT_GPIO);

    // AW9523 上的 K1/K2 由 bsp_xio_init 配为输入,这里确保已初始化
    esp_err_t e = bsp_xio_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "AW9523 未就绪,K1/K2 不可用 (%s)", esp_err_to_name(e));
        // 仍允许 BOOT 工作,不致命返回
    }

    s_btn[BSP_BTN_UP].id = BSP_BTN_UP;
    s_btn[BSP_BTN_UP].active_low = true;
    s_btn[BSP_BTN_UP].read = read_k1;

    s_btn[BSP_BTN_DOWN].id = BSP_BTN_DOWN;
    s_btn[BSP_BTN_DOWN].active_low = true;
    s_btn[BSP_BTN_DOWN].read = read_k2;

    s_btn[BSP_BTN_OK].id = BSP_BTN_OK;
    s_btn[BSP_BTN_OK].active_low = true;
    s_btn[BSP_BTN_OK].read = read_boot;

    for (int i = 0; i < BSP_BTN_COUNT; i++) {
        s_btn[i].deb = 0; s_btn[i].is_down = false; s_btn[i].was_down = false;
        s_btn[i].t_down_ms = 0; s_btn[i].long_fired = false;
    }

    if (xTaskCreate(btn_task, "btn_task", 2048, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "按键轮询任务创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "按键就绪:BOOT(OK) + K1(UP) + K2(DOWN)");
    return ESP_OK;
}

// box3 无 ADC 分压键,保留接口返回 -1(demo 的电压显示页会用,宠物页不用)。
int bsp_button_read_mv(void) { return -1; }
