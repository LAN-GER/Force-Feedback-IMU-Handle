/**
 * @file bsp_gpio.c
 * @layer BSP
 * @brief 用户键（GPIO9/BOOT）、控制台 TX 互斥、板载灯
 */

#include "bsp/bsp_gpio.h"
#include "bsp/bsp_pin.h"
#include "bsp/bsp_usb.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdarg.h>
#include <stdio.h>

#if BSP_HAS_USER_LED
#define LED_ON  0
#define LED_OFF 1
#define PULSE_US 40000
static int64_t s_led_off_us;
#endif

static vprintf_like_t s_prev_vprintf;
static SemaphoreHandle_t s_tx_mux;

void bsp_console_tx_lock(void)
{
    if (s_tx_mux) {
        xSemaphoreTake(s_tx_mux, pdMS_TO_TICKS(500));
    }
}

void bsp_console_tx_unlock(void)
{
    if (s_tx_mux) {
        xSemaphoreGive(s_tx_mux);
    }
}

static int console_vprintf(const char *fmt, va_list args)
{
    bsp_user_led_pulse();
    bsp_console_tx_lock();
    int n;
    if (s_prev_vprintf) {
        n = s_prev_vprintf(fmt, args);
    } else {
        n = vprintf(fmt, args);
    }
    bsp_console_tx_unlock();
    return n;
}

esp_err_t bsp_gpio_init(void)
{
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BSP_PIN_USER_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

#if BSP_HAS_USER_LED
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << BSP_PIN_USER_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led);
    gpio_set_level(BSP_PIN_USER_LED, LED_OFF);
#endif

    s_tx_mux = xSemaphoreCreateMutex();
    s_prev_vprintf = esp_log_set_vprintf(console_vprintf);
    return ESP_OK;
}

bool bsp_user_btn_pressed(void)
{
    return gpio_get_level(BSP_PIN_USER_BTN) == 0;
}

void bsp_user_led_pulse(void)
{
#if BSP_HAS_USER_LED
    gpio_set_level(BSP_PIN_USER_LED, LED_ON);
    s_led_off_us = esp_timer_get_time() + PULSE_US;
#else
    (void)0;
#endif
}

void bsp_console_activity_poll(void)
{
#if BSP_HAS_USER_LED
    if (s_led_off_us != 0 && esp_timer_get_time() >= s_led_off_us) {
        gpio_set_level(BSP_PIN_USER_LED, LED_OFF);
        s_led_off_us = 0;
    }
#endif
    bsp_usb_poll();
}
