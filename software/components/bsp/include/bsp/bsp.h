#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 USB、GPIO、SPI、电机 PWM、IPROPI ADC、编码器。 */
esp_err_t bsp_init(void);

#ifdef __cplusplus
}
#endif
