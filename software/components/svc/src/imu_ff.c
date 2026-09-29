/**
 * @file imu_ff.c
 * @layer SVC
 * @brief IMU 采样 + 编码器更新 + 阻抗环输出到电机
 */

#include "svc/imu_ff.h"
#include "drv/drv8833.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_mux;
static svc_telemetry_t s_tel;

esp_err_t svc_imu_ff_init(void)
{
    s_mux = xSemaphoreCreateMutex();
    s_tel = (svc_telemetry_t){0};
    return ESP_OK;
}

void svc_imu_ff_on_sample(void)
{
    bmi088_sample_t imu = {0};
    n20_encoder_sample_t enc = {0};
    ff_state_t ff = {0};

    bool imu_ok = (drv_bmi088_read(&imu) == ESP_OK) && imu.ok;
    (void)drv_n20_encoder_update(&enc);

    float duty = svc_ff_step(enc.theta_rad, enc.omega_rad_s, &ff);
    if (ff.enabled) {
        drv_drv8833_enable(true);
        drv_drv8833_set_duty(duty);
    } else {
        drv_drv8833_set_duty(0.0f);
        drv_drv8833_enable(false);
    }

    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(2)) == pdTRUE) {
        s_tel.imu = imu;
        s_tel.enc = enc;
        s_tel.ff = ff;
        s_tel.imu_ok = imu_ok;
        s_tel.seq++;
        xSemaphoreGive(s_mux);
    }
}

void svc_imu_ff_get(svc_telemetry_t *out)
{
    if (!out) {
        return;
    }
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        *out = s_tel;
        xSemaphoreGive(s_mux);
    }
}
