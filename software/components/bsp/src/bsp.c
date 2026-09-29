/**
 * @file bsp.c
 * @layer BSP
 * @brief 板级一次性初始化
 */

#include "bsp/bsp.h"
#include "bsp/bsp_encoder.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_heater.h"
#include "bsp/bsp_motor_pwm.h"
#include "bsp/bsp_spi.h"
#include "bsp/bsp_usb.h"
#include "bsp/bsp_ws2812.h"
#include "esp_log.h"

static const char *TAG = "bsp";

esp_err_t bsp_init(void)
{
    ESP_ERROR_CHECK(bsp_usb_init());
    ESP_ERROR_CHECK(bsp_gpio_init());
    ESP_ERROR_CHECK(bsp_spi_init());
    ESP_ERROR_CHECK(bsp_motor_pwm_init());
    ESP_ERROR_CHECK(bsp_heater_init());
    ESP_ERROR_CHECK(bsp_encoder_init());
    ESP_ERROR_CHECK(bsp_ws2812_init());
    ESP_LOGI(TAG, "板级就绪");
    return ESP_OK;
}
