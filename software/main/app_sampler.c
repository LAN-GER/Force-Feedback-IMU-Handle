/**
 * @file app_sampler.c
 * @layer APP
 * @brief 单核：1 kHz 采样 IMU + 力反馈环；低优先级轮询 USB / 灯色
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_ws2812.h"
#include "drv/n20_encoder.h"
#include "svc/imu_ff.h"
#include "svc/indicate.h"
#include "svc/ff_impedance.h"
#include "esp_log.h"
#include "app.h"

static const char *TAG_BTN = "btn";

static TaskHandle_t s_sam_task;
static TaskHandle_t s_idle_task;

static void on_sample_timer(void *arg)
{
    (void)arg;
    BaseType_t hp = pdFALSE;
    if (s_sam_task) {
        vTaskNotifyGiveFromISR(s_sam_task, &hp);
        portYIELD_FROM_ISR(hp);
    }
}

static void sampler_loop(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        svc_imu_ff_on_sample();
    }
}

void app_user_btn_poll(void)
{
    /* 简易消抖：按下沿清零编码器（与 USB zero 相同） */
    static bool prev;
    static int stable;
    bool now = bsp_user_btn_pressed();
    if (now == prev) {
        stable = 0;
        return;
    }
    if (++stable < 3) {
        return;
    }
    stable = 0;
    if (now && !prev) {
        drv_n20_encoder_zero();
        ESP_LOGI(TAG_BTN, "user btn: encoder zero");
    }
    prev = now;
}

static void idle_loop(void *arg)
{
    (void)arg;
    for (;;) {
        bsp_console_activity_poll();
        app_user_btn_poll();
        app_indicate_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_indicate_poll(void)
{
    svc_telemetry_t tel;
    svc_imu_ff_get(&tel);
    int mode = FF_IND_DISABLED;
    if (!tel.imu_ok && tel.seq > 10) {
        mode = FF_IND_IMU_FAIL;
    } else if (tel.ff.enabled) {
        mode = tel.ff.limited ? FF_IND_LIMIT : FF_IND_OK;
    }
    svc_rgb_t c = svc_indicate_from_mode(mode);
    bsp_ws2812_set_rgb(c.r, c.g, c.b);
}

void app_sampler_start(void)
{
    xTaskCreatePinnedToCore(sampler_loop, "app_sam", 4096, NULL, 7, &s_sam_task, 0);
    xTaskCreatePinnedToCore(idle_loop, "app_idle", 3072, NULL, 4, &s_idle_task, 0);

    const esp_timer_create_args_t targs = {
        .callback = &on_sample_timer,
        .name = "ff_1khz",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000));
}
