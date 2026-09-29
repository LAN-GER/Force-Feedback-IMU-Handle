#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_ws2812_init(void);
esp_err_t bsp_ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void bsp_ws2812_off(void);

#ifdef __cplusplus
}
#endif
