#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_heater_init(void);

/** 加热占空比 [0,1]：0=关，经 YJL3400A 驱动 100Ω 电阻。 */
void bsp_heater_set_duty(float duty);

float bsp_heater_get_duty(void);

#ifdef __cplusplus
}
#endif
