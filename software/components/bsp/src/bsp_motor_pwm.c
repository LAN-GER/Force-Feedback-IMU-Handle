/**
 * @file bsp_motor_pwm.c
 * @layer BSP
 * @brief DRV8876 PH/EN：EN=LEDC PWM，PH=GPIO 方向，nSLEEP
 */

#include "bsp/bsp_motor_pwm.h"
#include "bsp/bsp_pin.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "bsp_motor";

#define CH_EN  LEDC_CHANNEL_0
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
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BSP_PIN_MOTOR_NSLEEP) | (1ULL << BSP_PIN_MOTOR_PH),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(BSP_PIN_MOTOR_NSLEEP, 0);
    gpio_set_level(BSP_PIN_MOTOR_PH, 1);

    ledc_timer_config_t timer = {
        .speed_mode = SPEED,
        .duty_resolution = BSP_MOTOR_PWM_RES,
        .timer_num = TIMER,
        .freq_hz = BSP_MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = BSP_PIN_MOTOR_EN,
        .speed_mode = SPEED,
        .channel = CH_EN,
        .timer_sel = TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    s_ready = true;
    ESP_LOGI(TAG, "DRV8876 PH/EN %d Hz EN=%d PH=%d nSLEEP=%d",
             BSP_MOTOR_PWM_HZ, BSP_PIN_MOTOR_EN, BSP_PIN_MOTOR_PH,
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
    ledc_set_duty(SPEED, CH_EN, 0);
    ledc_update_duty(SPEED, CH_EN);
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
    /* PH=1 正转（与旧双 PWM「IN1 有 PWM」约定一致） */
    gpio_set_level(BSP_PIN_MOTOR_PH, (duty >= 0.0f) ? 1 : 0);
    ledc_set_duty(SPEED, CH_EN, ticks);
    ledc_update_duty(SPEED, CH_EN);
}
