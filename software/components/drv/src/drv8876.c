/**
 * @file drv8876.c
 * @layer DRV
 * @brief DRV8876：PH/EN 驱动 + IPROPI 电流
 */

#include "drv/drv8876.h"
#include "bsp/bsp_motor_pwm.h"
#include "bsp/bsp_adc_current.h"
#include <math.h>

esp_err_t drv_drv8876_init(void)
{
    drv_drv8876_enable(false);
    return ESP_OK;
}

void drv_drv8876_enable(bool on)
{
    bsp_motor_set_enable(on);
}

void drv_drv8876_set_duty(float duty)
{
    bsp_motor_set_duty(duty);
}

void drv_drv8876_coast(void)
{
    bsp_motor_coast();
}

float drv_drv8876_read_current_a(float duty_hint)
{
    float mag = bsp_adc_current_read_amps();
    if (!isfinite(mag) || mag < 0.0f) {
        mag = 0.0f;
    }
    if (fabsf(duty_hint) < 1e-4f) {
        return 0.0f;
    }
    return (duty_hint >= 0.0f) ? mag : -mag;
}
