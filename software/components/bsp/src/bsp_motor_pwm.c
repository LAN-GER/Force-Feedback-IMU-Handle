/**
 * @file bsp_motor_pwm.c
 * @layer BSP
 * @brief DRV8833 双 PWM（LEDC）+ nSLEEP
 */

#include "bsp/bsp_motor_pwm.h"
#include "bsp/bsp_pin.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "bsp_motor";

#define CH_IN1 LEDC_CHANNEL_0
#define CH_IN2 LEDC_CHANNEL_1
#define SPEED  LEDC_LOW_SPEED_MODE
#define TIMER  LEDC_TIMER_0

static bool s_ready;

static uint32_t duty_to_ticks(float abs_duty)
{
    if (abs_duty < 0.0f) {
        abs_duty = 0.0f;
    }
    if (abs_duty > 1.0f) {
        abs_duty = 1.0f;
    }
    return (uint32_t)(abs_duty * (float)BSP_MOTOR_PWM_MAX + 0.5f);
}

esp_err_t bsp_motor_pwm_init(void)
{
    gpio_config_t ns = {
        .pin_bit_mask = 1ULL << BSP_PIN_MOTOR_NSLEEP,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&ns);
    gpio_set_level(BSP_PIN_MOTOR_NSLEEP, 0);

    ledc_timer_config_t timer = {
        .speed_mode = SPEED,
        .duty_resolution = BSP_MOTOR_PWM_RES,
        .timer_num = TIMER,
        .freq_hz = BSP_MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch1 = {
        .gpio_num = BSP_PIN_MOTOR_IN1,
        .speed_mode = SPEED,
        .channel = CH_IN1,
        .timer_sel = TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t ch2 = {
        .gpio_num = BSP_PIN_MOTOR_IN2,
        .speed_mode = SPEED,
        .channel = CH_IN2,
        .timer_sel = TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));
    ESP_ERROR_CHECK(ledc_channel_config(&ch2));

    s_ready = true;
    ESP_LOGI(TAG, "LEDC %d Hz IN1=%d IN2=%d nSLEEP=%d",
             BSP_MOTOR_PWM_HZ, BSP_PIN_MOTOR_IN1, BSP_PIN_MOTOR_IN2,
             BSP_PIN_MOTOR_NSLEEP);
    return ESP_OK;
}

void bsp_motor_set_enable(bool enable)
{
    gpio_set_level(BSP_PIN_MOTOR_NSLEEP, enable ? 1 : 0);
    if (!enable) {
        bsp_motor_coast();
    }
}

void bsp_motor_coast(void)
{
    if (!s_ready) {
        return;
    }
    ledc_set_duty(SPEED, CH_IN1, 0);
    ledc_update_duty(SPEED, CH_IN1);
    ledc_set_duty(SPEED, CH_IN2, 0);
    ledc_update_duty(SPEED, CH_IN2);
}

void bsp_motor_set_duty(float duty)
{
    if (!s_ready) {
        return;
    }
    if (!isfinite(duty)) {
        duty = 0.0f;
    }
    if (duty > 1.0f) {
        duty = 1.0f;
    }
    if (duty < -1.0f) {
        duty = -1.0f;
    }

    uint32_t ticks = duty_to_ticks(fabsf(duty));
    if (duty >= 0.0f) {
        ledc_set_duty(SPEED, CH_IN1, ticks);
        ledc_set_duty(SPEED, CH_IN2, 0);
    } else {
        ledc_set_duty(SPEED, CH_IN1, 0);
        ledc_set_duty(SPEED, CH_IN2, ticks);
    }
    ledc_update_duty(SPEED, CH_IN1);
    ledc_update_duty(SPEED, CH_IN2);
}
