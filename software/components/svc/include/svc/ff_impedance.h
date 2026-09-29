#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float theta;
    float omega;
    float theta_des;
    float tau_cmd;   /* 阻抗输出（占空比前） */
    float duty;
    bool enabled;
    bool limited;
} ff_state_t;

esp_err_t svc_ff_init(void);
void svc_ff_set_gains(float k, float b, float duty_max);
void svc_ff_set_target(float theta_des);
void svc_ff_set_limits(float tmin, float tmax);
void svc_ff_enable(bool on);

/** 一步阻抗：tau = K*(des-theta) - B*omega，再映射占空比并限幅。 */
float svc_ff_step(float theta, float omega, ff_state_t *out);

void svc_ff_get_state(ff_state_t *out);

#ifdef __cplusplus
}
#endif
