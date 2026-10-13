/**
 * @file imu_ff.c
 * @layer SVC
 * @brief IMU + 编码器 + 多模式电机控制 + DOB
 */

#include "svc/imu_ff.h"
#include "svc/params.h"
#include "svc/imu_kalman.h"
#include "svc/imu_attitude.h"
#include "esp_timer.h"
#include "drv/drv8876.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

/* 由 app 层维护的全局参数 */
extern svc_params_t g_params;

static SemaphoreHandle_t s_mux;
static svc_telemetry_t s_tel;
static float s_openloop_duty;
static float s_duty_for_sign;
static imu_kalman_t s_imu_kalman;
static imu_attitude_t s_attitude;

esp_err_t svc_imu_ff_init(void)
{
    s_mux = xSemaphoreCreateMutex();
    s_tel = (svc_telemetry_t){0};
    s_openloop_duty = 0.0f;
    s_duty_for_sign = 0.0f;
    imu_kalman_reset(&s_imu_kalman);
    imu_attitude_reset(&s_attitude);
    return ESP_OK;
}

void svc_imu_ff_set_openloop_duty(float duty)
{
    if (!isfinite(duty)) {
        duty = 0.0f;
    }
    if (duty > 1.0f) {
        duty = 1.0f;
    }
    if (duty < -1.0f) {
        duty = -1.0f;
    }
    s_openloop_duty = duty;
}

void svc_imu_ff_on_sample(void)
{
    bmi088_sample_t imu = {0};
    n20_encoder_sample_t enc = {0};
    ff_state_t ff = {0};
    pos_pid_state_t pid = {0};
    torque_ctrl_state_t tq = {0};
    dob_state_t dob = {0};

    bool imu_ok = (drv_bmi088_read(&imu) == ESP_OK) && imu.ok;
    float raw[6] = {imu.ax_g, imu.ay_g, imu.az_g, imu.gx_dps, imu.gy_dps, imu.gz_dps};
    float filtered[6];
    bool kf_en = g_params.imu_kf_en != 0;
    int64_t now_us = esp_timer_get_time();
    bool kf_ready = imu_kalman_step(&s_imu_kalman, raw, imu_ok, kf_en,
                                  imu.acc_new, now_us, filtered);
    float quaternion[4];
    bool quaternion_ok = imu_attitude_step(&s_attitude, kf_ready ? filtered : raw,
                                           imu_ok, now_us, quaternion);
    (void)drv_n20_encoder_update(&enc);

    /* 先读电流（用上一步 duty 符号） */
    float i_meas = drv_drv8876_read_current_a(s_duty_for_sign);

    uint8_t mode = g_params.ctrl_mode;
    float applied = 0.0f;
    bool run = false;

    switch (mode) {
    case CTRL_MODE_IMPEDANCE:
        applied = svc_ff_step(enc.theta_rad, enc.omega_rad_s, &ff);
        run = ff.enabled;
        break;
    case CTRL_MODE_POS_PID:
        applied = svc_pos_pid_step(enc.theta_rad, enc.omega_rad_s, 0.001f, &pid);
        run = pid.enabled;
        break;
    case CTRL_MODE_TORQUE:
        applied = svc_torque_ctrl_step(i_meas, 0.001f, &tq);
        run = tq.enabled;
        break;
    default:
        /* OFF：开环 duty */
        svc_ff_step(enc.theta_rad, enc.omega_rad_s, &ff);
        applied = s_openloop_duty;
        run = fabsf(applied) > 1e-4f;
        break;
    }

    if (run) {
        drv_drv8876_enable(true);
        drv_drv8876_set_duty(applied);
    } else {
        drv_drv8876_set_duty(0.0f);
        drv_drv8876_enable(false);
    }
    s_duty_for_sign = applied;

    /* 再采一次电流供遥测/DOB（方向已更新） */
    i_meas = drv_drv8876_read_current_a(applied);
    svc_dob_step(enc.omega_rad_s, applied, i_meas, true, 0.001f, &dob);

    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(2)) == pdTRUE) {
        s_tel.imu = imu;
        for (int i = 0; i < 6; ++i) s_tel.imu_filtered[i] = filtered[i];
        s_tel.imu_kf_en = kf_en;
        s_tel.imu_kf_ready = kf_ready;
        memcpy(s_tel.quaternion, quaternion, sizeof(quaternion));
        s_tel.quaternion_ok = quaternion_ok;
        s_tel.quaternion_kf = quaternion_ok && kf_ready;
        s_tel.enc = enc;
        s_tel.ff = ff;
        s_tel.pid = pid;
        s_tel.tq = tq;
        s_tel.dob = dob;
        s_tel.applied_duty = applied;
        s_tel.i_meas = i_meas;
        s_tel.ctrl_mode = mode;
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
