/**
 * @file app_main.c
 * @layer APP
 * @brief ESP32-C3FH4：力反馈 IMU 手柄启动
 */

#include "esp_log.h"
#include "bsp/bsp.h"
#include "bsp/bsp_heater.h"
#include "bsp/bsp_ws2812.h"
#include "drv/bmi088.h"
#include "drv/drv8833.h"
#include "drv/n20_encoder.h"
#include "svc/params.h"
#include "svc/ff_impedance.h"
#include "svc/imu_ff.h"
#include "svc/indicate.h"
#include "app.h"

static const char *TAG = "app";

svc_params_t g_params;
svc_params_t g_params_nvs;

void app_main(void)
{
    ESP_LOGI(TAG, "力反馈 IMU 手柄启动 (ESP32-C3 单核)");

    ESP_ERROR_CHECK(svc_params_init(&g_params_nvs));
    g_params = g_params_nvs;

    ESP_ERROR_CHECK(bsp_init());
    ESP_ERROR_CHECK(drv_bmi088_init());
    ESP_ERROR_CHECK(drv_drv8833_init());
    ESP_ERROR_CHECK(drv_n20_encoder_init(g_params.enc_cpr));
    drv_n20_encoder_set_dir(g_params.enc_dir);

    ESP_ERROR_CHECK(svc_ff_init());
    svc_ff_set_gains(g_params.k, g_params.b, g_params.duty_max);
    svc_ff_set_target(g_params.theta_des);
    svc_ff_set_limits(g_params.theta_min, g_params.theta_max);
    svc_ff_enable(g_params.ff_enable != 0);
    bsp_heater_set_duty(g_params.heater_duty);

    ESP_ERROR_CHECK(svc_imu_ff_init());

    svc_rgb_t c = svc_indicate_from_mode(FF_IND_DISABLED);
    bsp_ws2812_set_rgb(c.r, c.g, c.b);

    app_usb_init();
    app_sampler_start();
}
