/**
 * @file bsp_ws2812.c
 * @layer BSP
 * @brief 本板无 WS2812：空实现保留 API
 */

#include "bsp/bsp_ws2812.h"
#include "esp_log.h"

static const char *TAG = "ws2812";

esp_err_t bsp_ws2812_init(void)
{
    ESP_LOGI(TAG, "disabled (no RGB on this board)");
    return ESP_OK;
}

esp_err_t bsp_ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    (void)r;
    (void)g;
    (void)b;
    return ESP_OK;
}

void bsp_ws2812_off(void)
{
}
