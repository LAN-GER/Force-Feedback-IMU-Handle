#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float tau_des;   /* 目标力矩 N·m（输出轴当量） */
    float kt;        /* N·m/A */
    float i_max;     /* 电流限幅 A */
    float duty_max;
} torque_ctrl_params_t;

typedef struct {
    float tau_des;
    float i_des;
    float i_meas;
    float tau_meas;  /* Kt * i_meas */
    float duty;
    bool enabled;
    bool limited;
} torque_ctrl_state_t;

esp_err_t svc_torque_ctrl_init(void);
void svc_torque_ctrl_set_params(const torque_ctrl_params_t *p);
void svc_torque_ctrl_set_tau(float tau_des);
void svc_torque_ctrl_enable(bool on);
void svc_torque_ctrl_reset(void);

/**
 * 电流 PI → 占空比。
 * @param i_meas 有符号电流 A
 */
float svc_torque_ctrl_step(float i_meas, float dt_s, torque_ctrl_state_t *out);

void svc_torque_ctrl_get_state(torque_ctrl_state_t *out);

#ifdef __cplusplus
}
#endif
