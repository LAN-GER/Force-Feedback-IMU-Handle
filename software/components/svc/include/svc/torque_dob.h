#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** DOB / 电枢估计参数（输出轴当量，需标定） */
typedef struct {
    float vm;
    float r;
    float ke;
    float kt;
    float j;
    float b_visc;
    float tau_c;
    float fc_hz;
    uint8_t enable;
} dob_params_t;

typedef struct {
    float i_hat;       /* 电枢估计或实测电流 A */
    float i_meas;      /* IPROPI 实测（有符号） */
    float tau_m_hat;   /* Kt * i */
    float alpha_hat;
    float tau_ext_hat;
    float duty;
    uint8_t i_from_sense; /* 1=用了实测电流 */
} dob_state_t;

esp_err_t svc_dob_init(void);
void svc_dob_set_params(const dob_params_t *p);
void svc_dob_get_params(dob_params_t *out);
void svc_dob_reset(void);

/**
 * @param i_meas   有符号实测电流；have_i=false 时用电枢估计
 * @param have_i   是否使用 IPROPI
 */
void svc_dob_step(float omega, float duty, float i_meas, bool have_i,
                  float dt_s, dob_state_t *out);

void svc_dob_get_state(dob_state_t *out);

#ifdef __cplusplus
}
#endif
