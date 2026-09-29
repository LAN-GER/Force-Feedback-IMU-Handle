/**
 * @file bsp_heater.c
 * @layer BSP
 * @brief BMI088 恒温加热：GPIO21 → YJL3400A 栅极 → 100Ω 电阻
 *
 * 3V3 -- Rheat(100Ω) -- Drain(YJL3400A)
 * Source -- GND
 * Gate -- GPIO21（建议串联 100Ω，栅极对地 10k 下拉）
 *
 * 3.3 V / 100 Ω ≈ 33 mA / 0.11 W，作局部温漂抑制。
 */

#include "bsp/bsp_heater.h"
#include "bsp/bsp_pin.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "bsp_heat";

#define CH_HEAT LEDC_CHANNEL_2
#define SPEED   LEDC_LOW_SPEED_MODE
#define TIMER   LEDC_TIMER_1

static bool s_ready;
static float s_duty;

esp_err_t bsp_heater_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = SPEED,
        .duty_resolution = BSP_HEATER_PWM_RES,
        .timer_num = TIMER,
        .freq_hz = BSP_HEATER_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = BSP_PIN_IMU_HEATER,
        .speed_mode = SPEED,
        .channel = CH_HEAT,
        .timer_sel = TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    s_ready = true;
    s_duty = 0.0f;
    ESP_LOGI(TAG, "heater PWM %d Hz GPIO%d (YJL3400A + 100ohm)",
             BSP_HEATER_PWM_HZ, BSP_PIN_IMU_HEATER);
    return ESP_OK;
}

void bsp_heater_set_duty(float duty)
{
    if (!s_ready) {
        return;
    }
    if (!isfinite(duty)) {
        duty = 0.0f;
    }
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > 1.0f) {
        duty = 1.0f;
    }
    s_duty = duty;
    uint32_t ticks = (uint32_t)(duty * (float)BSP_HEATER_PWM_MAX + 0.5f);
    ledc_set_duty(SPEED, CH_HEAT, ticks);
    ledc_update_duty(SPEED, CH_HEAT);
}

float bsp_heater_get_duty(void)
{
    return s_duty;
}
