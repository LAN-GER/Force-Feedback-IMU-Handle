/**
 * @file motor_pid.c
 * @layer SVC
 * @brief 位置闭环：duty = Kp*e + Ki*∫e - Kd*ω
 */

#include "svc/motor_pid.h"
#include <math.h>

static pos_pid_params_t s_p;
static pos_pid_state_t s_st;
static float s_integ;
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

esp_err_t svc_pos_pid_init(void)
{
    s_p = (pos_pid_params_t){
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.05f,
        .duty_max = 0.6f,
        .theta_des = 0.0f,
        .theta_min = -1.5f,
        .theta_max = 1.5f,
    };
    svc_pos_pid_reset();
    s_en = false;
    return ESP_OK;
}

void svc_pos_pid_set_params(const pos_pid_params_t *p)
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
    if (isfinite(p->kd) && p->kd >= 0.0f) {
        s_p.kd = p->kd;
    }
    if (isfinite(p->duty_max) && p->duty_max > 0.0f && p->duty_max <= 1.0f) {
        s_p.duty_max = p->duty_max;
    }
    if (isfinite(p->theta_des)) {
        s_p.theta_des = p->theta_des;
    }
    if (isfinite(p->theta_min) && isfinite(p->theta_max) && p->theta_min < p->theta_max) {
        s_p.theta_min = p->theta_min;
        s_p.theta_max = p->theta_max;
    }
}

void svc_pos_pid_set_target(float theta_des)
{
    if (isfinite(theta_des)) {
        s_p.theta_des = theta_des;
    }
}

void svc_pos_pid_enable(bool on)
{
    s_en = on;
    if (!on) {
        svc_pos_pid_reset();
    }
}

void svc_pos_pid_reset(void)
{
    s_integ = 0.0f;
    s_st = (pos_pid_state_t){0};
}

float svc_pos_pid_step(float theta, float omega, float dt_s, pos_pid_state_t *out)
{
    if (!isfinite(dt_s) || dt_s < 1e-5f) {
        dt_s = 0.001f;
    }
    if (!isfinite(theta)) {
        theta = 0.0f;
    }
    if (!isfinite(omega)) {
        omega = 0.0f;
    }

    pos_pid_state_t st = {
        .theta = theta,
        .omega = omega,
        .theta_des = s_p.theta_des,
        .enabled = s_en,
        .limited = false,
    };

    if (!s_en) {
        st.duty = 0.0f;
        st.err = 0.0f;
        s_st = st;
        if (out) {
            *out = st;
        }
        return 0.0f;
    }

    float des = clampf(s_p.theta_des, s_p.theta_min, s_p.theta_max);
    if (des != s_p.theta_des) {
        st.limited = true;
    }
    st.theta_des = des;

    float err = des - theta;
    st.err = err;

    /* 抗饱和：仅在未饱和方向积分 */
    float u_p = s_p.kp * err;
    float u_d = -s_p.kd * omega;
    float u_unsat = u_p + s_p.ki * s_integ + u_d;
    float u = clampf(u_unsat, -s_p.duty_max, s_p.duty_max);
    if (u == u_unsat || (err * u_unsat) <= 0.0f) {
        s_integ += err * dt_s;
        s_integ = clampf(s_integ, -2.0f, 2.0f);
        u_unsat = u_p + s_p.ki * s_integ + u_d;
        u = clampf(u_unsat, -s_p.duty_max, s_p.duty_max);
    }
    if (fabsf(u) >= s_p.duty_max - 1e-6f) {
        st.limited = true;
    }

    /* 软限位外额外回推 */
    if (theta < s_p.theta_min) {
        u += s_p.kp * (s_p.theta_min - theta);
        st.limited = true;
    } else if (theta > s_p.theta_max) {
        u += s_p.kp * (s_p.theta_max - theta);
        st.limited = true;
    }
    u = clampf(u, -s_p.duty_max, s_p.duty_max);

    st.duty = u;
    s_st = st;
    if (out) {
        *out = st;
    }
    return u;
}

void svc_pos_pid_get_state(pos_pid_state_t *out)
{
    if (out) {
        *out = s_st;
    }
}
