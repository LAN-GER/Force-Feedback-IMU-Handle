/**
 * @file n20_encoder.c
 * @layer DRV
 * @brief N20 霍尔编码器：计数 → 角度 / 角速度
 */

#include "drv/n20_encoder.h"
#include "bsp/bsp_encoder.h"
#include "esp_timer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int32_t s_cpr = 600;
static int s_dir = 1; /* +1 / -1 */
static int32_t s_prev_counts;
static int64_t s_prev_us;
static float s_omega;

esp_err_t drv_n20_encoder_init(int32_t cpr)
{
    if (cpr <= 0) {
        cpr = 600;
    }
    s_cpr = cpr;
    bsp_encoder_clear();
    s_prev_counts = 0;
    s_prev_us = esp_timer_get_time();
    s_omega = 0.0f;
    return ESP_OK;
}

void drv_n20_encoder_set_cpr(int32_t cpr)
{
    if (cpr > 0) {
        s_cpr = cpr;
    }
}

void drv_n20_encoder_zero(void)
{
    bsp_encoder_clear();
    s_prev_counts = 0;
    s_prev_us = esp_timer_get_time();
    s_omega = 0.0f;
}

void drv_n20_encoder_set_dir(int dir)
{
    s_dir = dir ? -1 : 1;
}

esp_err_t drv_n20_encoder_update(n20_encoder_sample_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t counts = bsp_encoder_get_count() * s_dir;
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - s_prev_us) * 1e-6f;
    if (dt > 1e-4f) {
        float dtheta = (float)(counts - s_prev_counts) * (float)(2.0 * M_PI) / (float)s_cpr;
        float w = dtheta / dt;
        /* 一阶低通，抑制抖动 */
        s_omega = 0.7f * s_omega + 0.3f * w;
        s_prev_counts = counts;
        s_prev_us = now;
    }

    out->counts = counts;
    out->theta_rad = (float)counts * (float)(2.0 * M_PI) / (float)s_cpr;
    out->omega_rad_s = s_omega;
    return ESP_OK;
}
