#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float duty_max;
    float theta_des;
    float theta_min;
    float theta_max;
} pos_pid_params_t;

typedef struct {
    float theta;
    float omega;
    float theta_des;
    float err;
    float duty;
    bool enabled;
    bool limited;
} pos_pid_state_t;

esp_err_t svc_pos_pid_init(void);
void svc_pos_pid_set_params(const pos_pid_params_t *p);
void svc_pos_pid_set_target(float theta_des);
void svc_pos_pid_enable(bool on);
void svc_pos_pid_reset(void);

/** 位置 PID → 占空比。dt_s 建议 0.001。 */
float svc_pos_pid_step(float theta, float omega, float dt_s, pos_pid_state_t *out);

void svc_pos_pid_get_state(pos_pid_state_t *out);

#ifdef __cplusplus
}
#endif
