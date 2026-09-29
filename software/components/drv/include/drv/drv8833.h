#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t drv_drv8833_init(void);
void drv_drv8833_enable(bool on);

/** 有符号占空比 [-1, 1]。 */
void drv_drv8833_set_duty(float duty);
void drv_drv8833_coast(void);

#ifdef __cplusplus
}
#endif
