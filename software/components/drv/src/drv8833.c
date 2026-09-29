/**
 * @file drv8833.c
 * @layer DRV
 * @brief DRV8833 薄封装：使能 + 占空比
 */

#include "drv/drv8833.h"
#include "bsp/bsp_motor_pwm.h"

esp_err_t drv_drv8833_init(void)
{
    /* PWM / nSLEEP 已在 bsp_init 中完成 */
    drv_drv8833_enable(false);
    return ESP_OK;
}

void drv_drv8833_enable(bool on)
{
    bsp_motor_set_enable(on);
}

void drv_drv8833_set_duty(float duty)
{
    bsp_motor_set_duty(duty);
}

void drv_drv8833_coast(void)
{
    bsp_motor_coast();
}
