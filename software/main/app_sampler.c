/**
 * @file app_sampler.c
 * @layer APP
 * @brief 1 kHz 采样；空闲任务：USB / 按键（无 RGB）
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "bsp/bsp_gpio.h"
#include "drv/n20_encoder.h"
#include "svc/imu_ff.h"
#include "svc/torque_dob.h"
#include "esp_log.h"
#include "app.h"

static const char *TAG_BTN = "btn";

static TaskHandle_t s_sam_task;
static TaskHandle_t s_idle_task;

static void on_sample_timer(void *arg)
{
    (void)arg;
    if (s_sam_task) {
        /* ESP_TIMER_TASK callback runs in a task, not an interrupt. */
        xTaskNotifyGive(s_sam_task);
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
        svc_dob_reset();
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_indicate_poll(void)
{
    /* 本板无 WS2812 */
}

void app_sampler_start(void)
{
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(sampler_loop, "app_sam", 4096, NULL, 7,
                                          &s_sam_task, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(idle_loop, "app_idle", 4096, NULL, 4,
                                          &s_idle_task, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    const esp_timer_create_args_t targs = {
        .callback = &on_sample_timer,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ff_1khz",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000));
}
