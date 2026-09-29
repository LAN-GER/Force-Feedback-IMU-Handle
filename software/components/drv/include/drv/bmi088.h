#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    uint8_t acc_id;
    uint8_t gyro_id;
    bool ok;
} bmi088_sample_t;

esp_err_t drv_bmi088_init(void);
esp_err_t drv_bmi088_read(bmi088_sample_t *out);
uint8_t drv_bmi088_read_acc_id(void);
uint8_t drv_bmi088_read_gyro_id(void);

#ifdef __cplusplus
}
#endif
