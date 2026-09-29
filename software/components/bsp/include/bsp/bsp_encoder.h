#pragma once

#include <stdint.h>
#include "esp_err.h"

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
