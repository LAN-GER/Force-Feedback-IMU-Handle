#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_motor_pwm_init(void);

/** 使能/休眠 DRV8833（nSLEEP）。 */
void bsp_motor_set_enable(bool enable);

/** 有符号占空比 [-1, 1]：正转 IN1，反转 IN2。 */
void bsp_motor_set_duty(float duty);

/** 立刻停转（两路 PWM=0，coast）。 */
void bsp_motor_coast(void);

#ifdef __cplusplus
}
#endif
