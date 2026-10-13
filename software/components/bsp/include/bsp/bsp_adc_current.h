#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_adc_current_init(void);

/**
 * 读 IPROPI 电流幅值（A，非负）。
 * 有符号电流请结合电机方向（占空比符号）在上层合成。
 */
float bsp_adc_current_read_amps(void);

/** 原始电压 V（校准后）。 */
float bsp_adc_current_read_volts(void);

#ifdef __cplusplus
}
#endif
