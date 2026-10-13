#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 控制模式 */
enum {
    CTRL_MODE_OFF = 0,        /* 开环 duty / 停机 */
    CTRL_MODE_IMPEDANCE = 1,  /* 阻抗力反馈 */
    CTRL_MODE_POS_PID = 2,    /* 位置 PID */
    CTRL_MODE_TORQUE = 3,     /* 力矩（电流 PI） */
};

typedef struct {
    float k;           /* 弹簧刚度 */
    float b;           /* 阻尼 */
    float theta_des;   /* 目标角 rad */
    float duty_max;    /* 占空比上限 [0,1] */
    float theta_min;   /* 软限位 rad */
    float theta_max;
    int32_t enc_cpr;   /* 输出轴每转四倍频计数，已包含减速比；默认 7*4*298=8344 */
    uint8_t enc_dir;
    uint8_t ctrl_mode; /* CTRL_MODE_* */
    uint8_t ff_enable; /* 兼容：1 → 等价 ctrl_mode=IMPEDANCE */
    uint32_t telem_hz;
    float heater_duty;
    uint8_t imu_kf_en; /* Six-axis Kalman denoising, independent of motor mode. */

    /* 位置 PID */
    float pid_kp;
    float pid_ki;
    float pid_kd;

    /* 力矩 / 电流环 */
    float tau_des;
    float tq_kp;
    float tq_ki;
    float i_max;
    float kt; /* 力矩系数，力矩环与 DOB 共用默认 */

    /* DOB */
    float dob_vm;
    float dob_r;
    float dob_ke;
    float dob_kt;
    float dob_j;
    float dob_b_visc;
    float dob_tau_c;
    float dob_fc_hz;
    uint8_t dob_enable;
} svc_params_t;

esp_err_t svc_params_init(svc_params_t *out);
esp_err_t svc_params_save(const svc_params_t *in);

#ifdef __cplusplus
}
#endif
