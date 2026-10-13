/**
 * @file torque_dob.c
 * @layer SVC
 * @brief 力矩估计：优先 IPROPI 实测电流，否则电枢方程
 *
 *   Î = I_meas  或  (d·Vm - Ke·ω) / R
 *   τ̂_m = Kt · Î
 *   τ̂_ext ≈ τ̂_m - J·α - B·ω - τc·sign(ω)   （再低通）
 */

#include "svc/torque_dob.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static dob_params_t s_p;
static dob_state_t s_st;
static float s_omega_prev;
static float s_alpha_f;
static float s_tau_ext_f;
static bool s_primed;

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

static float sign_dz(float x, float dz)
{
    if (x > dz) {
        return 1.0f;
    }
    if (x < -dz) {
        return -1.0f;
    }
    return 0.0f;
}

static float lpf_update(float y, float x, float fc_hz, float dt)
{
    if (fc_hz < 0.5f) {
        fc_hz = 0.5f;
    }
    float tau = 1.0f / (2.0f * (float)M_PI * fc_hz);
    float a = dt / (tau + dt);
    if (a > 1.0f) {
        a = 1.0f;
    }
    return y + a * (x - y);
}

esp_err_t svc_dob_init(void)
{
    s_p = (dob_params_t){
        .vm = 5.0f,
        .r = 5.0f,
        .ke = 0.02f,
        .kt = 0.02f,
        .j = 5.0e-5f,
        .b_visc = 1.0e-4f,
        .tau_c = 0.0f,
        .fc_hz = 30.0f,
        .enable = 1,
    };
    svc_dob_reset();
    return ESP_OK;
}

void svc_dob_set_params(const dob_params_t *p)
{
    if (!p) {
        return;
    }
    if (isfinite(p->vm) && p->vm > 0.5f && p->vm < 20.0f) {
        s_p.vm = p->vm;
    }
    if (isfinite(p->r) && p->r > 0.05f && p->r < 100.0f) {
        s_p.r = p->r;
    }
    if (isfinite(p->ke) && p->ke >= 0.0f) {
        s_p.ke = p->ke;
    }
    if (isfinite(p->kt) && p->kt >= 0.0f) {
        s_p.kt = p->kt;
    }
    if (isfinite(p->j) && p->j > 0.0f) {
        s_p.j = p->j;
    }
    if (isfinite(p->b_visc) && p->b_visc >= 0.0f) {
        s_p.b_visc = p->b_visc;
    }
    if (isfinite(p->tau_c) && p->tau_c >= 0.0f) {
        s_p.tau_c = p->tau_c;
    }
    if (isfinite(p->fc_hz) && p->fc_hz >= 1.0f && p->fc_hz <= 200.0f) {
        s_p.fc_hz = p->fc_hz;
    }
    s_p.enable = p->enable ? 1 : 0;
}

void svc_dob_get_params(dob_params_t *out)
{
    if (out) {
        *out = s_p;
    }
}

void svc_dob_reset(void)
{
    s_st = (dob_state_t){0};
    s_omega_prev = 0.0f;
    s_alpha_f = 0.0f;
    s_tau_ext_f = 0.0f;
    s_primed = false;
}

void svc_dob_step(float omega, float duty, float i_meas, bool have_i,
                  float dt_s, dob_state_t *out)
{
    duty = clampf(duty, -1.0f, 1.0f);
    if (!isfinite(omega)) {
        omega = 0.0f;
    }
    if (!isfinite(dt_s) || dt_s < 1e-5f) {
        dt_s = 0.001f;
    }

    dob_state_t st = {
        .duty = duty,
        .i_meas = have_i ? i_meas : 0.0f,
        .i_from_sense = have_i ? 1 : 0,
    };

    if (!s_p.enable) {
        s_st = st;
        if (out) {
            *out = st;
        }
        return;
    }

    float i;
    if (have_i && isfinite(i_meas)) {
        i = i_meas;
    } else {
        float u = duty * s_p.vm;
        i = (u - s_p.ke * omega) / s_p.r;
    }
    st.i_hat = i;
    st.tau_m_hat = s_p.kt * i;

    float alpha_raw = 0.0f;
    if (s_primed) {
        alpha_raw = (omega - s_omega_prev) / dt_s;
    }
    s_omega_prev = omega;
    s_primed = true;

    s_alpha_f = lpf_update(s_alpha_f, alpha_raw, s_p.fc_hz, dt_s);
    st.alpha_hat = s_alpha_f;

    float tau_f = s_p.tau_c * sign_dz(omega, 0.05f);
    float tau_ext_raw = st.tau_m_hat - s_p.j * s_alpha_f - s_p.b_visc * omega - tau_f;
    s_tau_ext_f = lpf_update(s_tau_ext_f, tau_ext_raw, s_p.fc_hz, dt_s);
    st.tau_ext_hat = s_tau_ext_f;

    s_st = st;
    if (out) {
        *out = st;
    }
}

void svc_dob_get_state(dob_state_t *out)
{
    if (out) {
        *out = s_st;
    }
}
