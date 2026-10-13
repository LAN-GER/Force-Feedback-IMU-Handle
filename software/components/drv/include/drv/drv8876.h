#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t drv_drv8876_init(void);
void drv_drv8876_enable(bool on);

/** 有符号占空比 [-1, 1]（PH/EN）。 */
void drv_drv8876_set_duty(float duty);
void drv_drv8876_coast(void);

/**
 * 有符号电流（A）：IPROPI 幅值 × sign(duty_hint)。
 * @param duty_hint 当前占空比符号，0 时返回 0
 */
float drv_drv8876_read_current_a(float duty_hint);

#ifdef __cplusplus
}
#endif
