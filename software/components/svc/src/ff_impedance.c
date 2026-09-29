/**
 * @file ff_impedance.c
 * @layer SVC
 * @brief 阻抗力反馈：tau = K*(theta_des - theta) - B*omega
 */

#include "svc/ff_impedance.h"
#include <math.h>

static float s_k = 0.8f;
static float s_b = 0.05f;
static float s_des = 0.0f;
static float s_dmax = 0.6f;
static float s_tmin = -1.5f;
static float s_tmax = 1.5f;
static bool s_en;
static ff_state_t s_last;

esp_err_t svc_ff_init(void)
{
    s_en = false;
    s_last = (ff_state_t){0};
    return ESP_OK;
}

void svc_ff_set_gains(float k, float b, float duty_max)
{
    if (isfinite(k) && k >= 0.0f) {
        s_k = k;
    }
    if (isfinite(b) && b >= 0.0f) {
        s_b = b;
    }
    if (isfinite(duty_max) && duty_max > 0.0f && duty_max <= 1.0f) {
        s_dmax = duty_max;
    }
}

void svc_ff_set_target(float theta_des)
{
    if (isfinite(theta_des)) {
        s_des = theta_des;
    }
}

void svc_ff_set_limits(float tmin, float tmax)
{
    if (isfinite(tmin) && isfinite(tmax) && tmin < tmax) {
        s_tmin = tmin;
        s_tmax = tmax;
    }
}

void svc_ff_enable(bool on)
{
    s_en = on;
    if (!on) {
        s_last.duty = 0.0f;
        s_last.tau_cmd = 0.0f;
        s_last.enabled = false;
    }
}

float svc_ff_step(float theta, float omega, ff_state_t *out)
{
    ff_state_t st = {
        .theta = theta,
        .omega = omega,
        .theta_des = s_des,
        .enabled = s_en,
        .limited = false,
    };

    if (!s_en) {
        st.tau_cmd = 0.0f;
        st.duty = 0.0f;
        s_last = st;
        if (out) {
            *out = st;
        }
        return 0.0f;
    }

    float des = s_des;
    if (des < s_tmin) {
        des = s_tmin;
        st.limited = true;
    }
    if (des > s_tmax) {
        des = s_tmax;
        st.limited = true;
    }

    /* 超出软限位时额外回推 */
    float tau = s_k * (des - theta) - s_b * omega;
    if (theta < s_tmin) {
        tau += s_k * (s_tmin - theta);
        st.limited = true;
    } else if (theta > s_tmax) {
        tau += s_k * (s_tmax - theta);
        st.limited = true;
    }

    float duty = tau;
    if (duty > s_dmax) {
        duty = s_dmax;
        st.limited = true;
    } else if (duty < -s_dmax) {
        duty = -s_dmax;
        st.limited = true;
    }

    st.tau_cmd = tau;
    st.duty = duty;
    s_last = st;
    if (out) {
        *out = st;
    }
    return duty;
}

void svc_ff_get_state(ff_state_t *out)
{
    if (out) {
        *out = s_last;
    }
}
