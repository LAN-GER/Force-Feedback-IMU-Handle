#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t counts;
    float theta_rad;
    float omega_rad_s;
} n20_encoder_sample_t;

/**
 * @param cpr 输出轴每转正交计数 = motor_ppr * 4 * gear_ratio
 *            默认：3 PPR * 4 * 50 = 600
 */
esp_err_t drv_n20_encoder_init(int32_t cpr);
void drv_n20_encoder_set_cpr(int32_t cpr);
void drv_n20_encoder_zero(void);
void drv_n20_encoder_set_dir(int dir); /* 0=正向, 1=反向 */

/** 读取并更新角速度（需周期性调用）。 */
esp_err_t drv_n20_encoder_update(n20_encoder_sample_t *out);

#ifdef __cplusplus
}
#endif
