#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Nominal motor-shaft pulses/rev inferred from ~8292 output counts:
 * 8292 / (298 * 4) ~= 7 PPR. Confirm against the actual motor specification.
 * Output CPR already includes the gearbox and all A/B edges. */
#define BSP_ENC_MOTOR_PPR       7
#define BSP_ENC_QUADRATURE      4
#define BSP_MOTOR_GEAR_RATIO    298
#define BSP_ENC_OUTPUT_CPR      (BSP_ENC_MOTOR_PPR * BSP_ENC_QUADRATURE * BSP_MOTOR_GEAR_RATIO)

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_encoder_init(void);

/** 当前正交累计计数（四倍频）。 */
int32_t bsp_encoder_get_count(void);

void bsp_encoder_set_count(int32_t count);
void bsp_encoder_clear(void);

#ifdef __cplusplus
}
#endif
