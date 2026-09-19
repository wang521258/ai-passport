// main/main.c —— 锦鲤池专用固件入口（本仓库分支 koi-firmware）
//
// ★ 为什么砍成单页、且关掉无线栈：
//   ESP32-C3 无 PSRAM。原工程（WiFi + NimBLE + 菜单全量 demo）链接时报
//     ld: region `dram0_0_seg' overflowed by 146800 bytes
//   而锦鲤池需要一块 240x320 RGB565 全屏帧缓冲 = 153,600 B。
//   换句话说：这颗片子在带无线协议栈时几乎一字节 DRAM 都不剩，
//   **全屏帧缓冲与无线栈不可兼得**。
//   锦鲤池不需要联网、不需要蓝牙、不需要音频、不需要电量计，于是这里只留：
//   屏 + 按键 + 锦鲤池。这样 DRAM 才够摆帧缓冲。
//   （要跑上游那套 AI Passport 功能，请把原厂 8MB 整片备份刷回去。）
//
// 想换回多页 demo 菜单：本文件的上一版就在 git 历史里
// （分支 koi-firmware 的 koi-v2 提交），取回即可。
//
// 按键（沿用仓库约定）：
//   上 短按 = 投喂 / 下 短按 = 拍水 / 确定 短按 = 昼夜切换
#include "bsp_i2c.h"
#include "bsp_display.h"
#include "bsp_button.h"
#include "bsp_pins.h"
#include "demo.h"
#include "esp_log.h"

static const char *TAG = "koi";

// 按键回调运行在 button 组件自己的任务里；LVGL 不是线程安全的，
// 而本页的 tick 回调跑在 LVGL 任务内（esp_lvgl_port 的 lv_timer_handler 里），
// 所以这里必须拿同一把锁把两边串起来。
static void on_key(bsp_btn_t btn, bsp_btn_ev_t ev, void *user)
{
    (void)user;
    if (!bsp_lvgl_lock(200)) return;
    demo_koi_key(btn, ev);
    bsp_lvgl_unlock();
}

void app_main(void)
{
    ESP_LOGI(TAG, "锦鲤池固件启动");

    bsp_i2c_init();

    if (bsp_display_init() != ESP_OK || !bsp_lvgl_init()) {
        ESP_LOGE(TAG, "显示/LVGL 初始化失败,检查 SPI 接线"
                      "(MOSI=%d SCLK=%d CS=%d DC=%d BL=%d)",
                 BSP_LCD_MOSI, BSP_LCD_SCLK, BSP_LCD_CS, BSP_LCD_DC, BSP_LCD_BL);
        return;
    }
    bsp_display_backlight(100);

    // 按键失败不阻塞：鱼照游，只是投喂/拍水/昼夜按不动。
    if (bsp_button_init(on_key, NULL) != ESP_OK) {
        ESP_LOGW(TAG, "按键初始化失败:画面正常,但三个键无响应");
    }

    if (bsp_lvgl_lock(1000)) {
        demo_koi_enter();
        bsp_lvgl_unlock();
    }

    ESP_LOGI(TAG, "就绪");
}
