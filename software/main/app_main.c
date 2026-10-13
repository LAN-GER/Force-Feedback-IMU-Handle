/**
 * @file app_main.c
 * @layer APP
 * @brief ESP32-C3FH4：力反馈 IMU 手柄（DRV8876）
 */

#include "esp_log.h"
#include "bsp/bsp.h"
#include "bsp/bsp_heater.h"
#include "drv/bmi088.h"
#include "drv/drv8876.h"
#include "drv/n20_encoder.h"
#include "svc/params.h"
#include "svc/ff_impedance.h"
#include "svc/motor_pid.h"
#include "svc/motor_torque.h"
#include "svc/torque_dob.h"
#include "svc/imu_ff.h"
#include "app.h"
#include "app_wifi.h"

static const char *TAG = "app";

svc_params_t g_params;
svc_params_t g_params_nvs;
esp_err_t g_imu_init_err = ESP_ERR_INVALID_STATE;

static void apply_dob_from_params(const svc_params_t *p)
{
    dob_params_t d = {
        .vm = p->dob_vm,
        .r = p->dob_r,
        .ke = p->dob_ke,
        .kt = p->dob_kt > 1e-9f ? p->dob_kt : p->kt,
        .j = p->dob_j,
        .b_visc = p->dob_b_visc,
        .tau_c = p->dob_tau_c,
        .fc_hz = p->dob_fc_hz,
        .enable = p->dob_enable,
    };
    svc_dob_set_params(&d);
}

void app_apply_controllers_from_params(void)
{
    svc_ff_set_gains(g_params.k, g_params.b, g_params.duty_max);
    svc_ff_set_target(g_params.theta_des);
    svc_ff_set_limits(g_params.theta_min, g_params.theta_max);

    pos_pid_params_t pid = {
        .kp = g_params.pid_kp,
        .ki = g_params.pid_ki,
        .kd = g_params.pid_kd,
        .duty_max = g_params.duty_max,
        .theta_des = g_params.theta_des,
        .theta_min = g_params.theta_min,
        .theta_max = g_params.theta_max,
    };
    svc_pos_pid_set_params(&pid);

    torque_ctrl_params_t tq = {
        .kp = g_params.tq_kp,
        .ki = g_params.tq_ki,
        .tau_des = g_params.tau_des,
        .kt = g_params.kt,
        .i_max = g_params.i_max,
        .duty_max = g_params.duty_max,
    };
    svc_torque_ctrl_set_params(&tq);

    /* 按模式使能对应环；互斥 */
    svc_ff_enable(g_params.ctrl_mode == CTRL_MODE_IMPEDANCE);
    svc_pos_pid_enable(g_params.ctrl_mode == CTRL_MODE_POS_PID);
    svc_torque_ctrl_enable(g_params.ctrl_mode == CTRL_MODE_TORQUE);

    g_params.ff_enable = (g_params.ctrl_mode == CTRL_MODE_IMPEDANCE) ? 1 : 0;

    drv_n20_encoder_set_cpr(g_params.enc_cpr);
    drv_n20_encoder_set_dir(g_params.enc_dir);
    bsp_heater_set_duty(g_params.heater_duty);
    apply_dob_from_params(&g_params);
}

void app_main(void)
{
    ESP_LOGI(TAG, "力反馈 IMU 手柄启动 (DRV8876 / ESP32-C3)");

    ESP_ERROR_CHECK(svc_params_init(&g_params_nvs));
    g_params = g_params_nvs;

    ESP_ERROR_CHECK(bsp_init());
    esp_err_t imu_err = drv_bmi088_init();
    g_imu_init_err = imu_err;
    if (imu_err != ESP_OK) {
        /* Keep USB diagnostics reachable when the sensor is missing/miswired. */
        ESP_LOGE(TAG, "BMI088 初始化失败 %s；继续 USB/编码器诊断，IMU 标记不可用",
                 esp_err_to_name(imu_err));
        g_params.ctrl_mode = CTRL_MODE_OFF;
        g_params.ff_enable = 0;
        g_params.heater_duty = 0.0f;
    }
    ESP_ERROR_CHECK(drv_drv8876_init());
    ESP_ERROR_CHECK(drv_n20_encoder_init(g_params.enc_cpr));
    drv_n20_encoder_set_dir(g_params.enc_dir);

    ESP_ERROR_CHECK(svc_ff_init());
    ESP_ERROR_CHECK(svc_pos_pid_init());
    ESP_ERROR_CHECK(svc_torque_ctrl_init());
    ESP_ERROR_CHECK(svc_dob_init());
    ESP_ERROR_CHECK(svc_imu_ff_init());

    app_apply_controllers_from_params();

    app_usb_init();
    app_sampler_start();
    esp_err_t wifi_err = app_wifi_init();
    if (wifi_err != ESP_OK) ESP_LOGE(TAG, "Wi-Fi unavailable: %s; USB remains active", esp_err_to_name(wifi_err));
}
