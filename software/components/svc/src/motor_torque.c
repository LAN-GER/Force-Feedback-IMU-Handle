/**
 * @file motor_torque.c
 * @layer SVC
 * @brief 力矩/电流闭环：i_des = τ_des/Kt，duty += PI(i_des - i_meas)
 */

#include "svc/motor_torque.h"
#include <math.h>

static torque_ctrl_params_t s_p;
static torque_ctrl_state_t s_st;
static float s_integ;
static float s_duty;
static bool s_en;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

esp_err_t svc_torque_ctrl_init(void)
{
    s_p = (torque_ctrl_params_t){
        .kp = 1.5f,
        .ki = 8.0f,
        .tau_des = 0.0f,
        .kt = 0.02f,
        .i_max = 1.0f,
        .duty_max = 0.8f,
    };
    svc_torque_ctrl_reset();
    s_en = false;
    return ESP_OK;
}

void svc_torque_ctrl_set_params(const torque_ctrl_params_t *p)
{
    if (!p) {
        return;
    }
    if (isfinite(p->kp) && p->kp >= 0.0f) {
        s_p.kp = p->kp;
    }
    if (isfinite(p->ki) && p->ki >= 0.0f) {
        s_p.ki = p->ki;
    }
    if (isfinite(p->tau_des)) {
        s_p.tau_des = p->tau_des;
    }
    if (isfinite(p->kt) && p->kt > 1e-6f) {
        s_p.kt = p->kt;
    }
    if (isfinite(p->i_max) && p->i_max > 0.05f) {
        s_p.i_max = p->i_max;
    }
    if (isfinite(p->duty_max) && p->duty_max > 0.0f && p->duty_max <= 1.0f) {
        s_p.duty_max = p->duty_max;
    }
}

void svc_torque_ctrl_set_tau(float tau_des)
{
    if (isfinite(tau_des)) {
        s_p.tau_des = tau_des;
    }
}

void svc_torque_ctrl_enable(bool on)
{
    s_en = on;
    if (!on) {
        svc_torque_ctrl_reset();
    }
}

void svc_torque_ctrl_reset(void)
{
    s_integ = 0.0f;
    s_duty = 0.0f;
    s_st = (torque_ctrl_state_t){0};
}

float svc_torque_ctrl_step(float i_meas, float dt_s, torque_ctrl_state_t *out)
{
    if (!isfinite(dt_s) || dt_s < 1e-5f) {
        dt_s = 0.001f;
    }
    if (!isfinite(i_meas)) {
        i_meas = 0.0f;
    }

    torque_ctrl_state_t st = {
        .tau_des = s_p.tau_des,
        .i_meas = i_meas,
        .tau_meas = s_p.kt * i_meas,
        .enabled = s_en,
        .limited = false,
    };

    if (!s_en) {
        st.i_des = 0.0f;
        st.duty = 0.0f;
        s_duty = 0.0f;
        s_st = st;
        if (out) {
            *out = st;
        }
        return 0.0f;
    }

    float i_des = s_p.tau_des / s_p.kt;
    i_des = clampf(i_des, -s_p.i_max, s_p.i_max);
    if (fabsf(i_des) >= s_p.i_max - 1e-6f) {
        st.limited = true;
    }
    st.i_des = i_des;

    float err = i_des - i_meas;
    float duty_cmd = s_p.kp * err + s_p.ki * s_integ;
    float duty = clampf(duty_cmd, -s_p.duty_max, s_p.duty_max);
    if (duty == duty_cmd || (err * duty_cmd) <= 0.0f) {
        s_integ += err * dt_s;
        float lim = 1.0f / fmaxf(s_p.ki, 1e-3f);
        s_integ = clampf(s_integ, -lim, lim);
        duty_cmd = s_p.kp * err + s_p.ki * s_integ;
        duty = clampf(duty_cmd, -s_p.duty_max, s_p.duty_max);
    }
    if (fabsf(duty) >= s_p.duty_max - 1e-6f) {
        st.limited = true;
    }

    s_duty = duty;
    st.duty = duty;
    s_st = st;
    if (out) {
        *out = st;
    }
    return duty;
}

void svc_torque_ctrl_get_state(torque_ctrl_state_t *out)
{
    if (out) {
        *out = s_st;
    }
}
