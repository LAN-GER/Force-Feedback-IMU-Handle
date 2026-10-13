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
    bool acc_new; /* ACC_STATUS bit 7, sampled before reading acceleration. */
} bmi088_sample_t;

typedef struct {
    bool ready;
    uint8_t acc_id;
    uint8_t gyro_id;
    esp_err_t acc_spi_err;
    esp_err_t gyro_spi_err;
} bmi088_diagnostics_t;

#define BMI088_PROBE_CASES 6
typedef struct {
    uint32_t hz;
    uint8_t mode;
    uint8_t acc_raw[3];
    uint8_t gyro_raw[2];
    esp_err_t acc_err;
    esp_err_t gyro_err;
} bmi088_probe_case_t;

typedef struct {
    int miso_up;
    int miso_down;
    int miso_input;
    int miso_oe;
    int miso_periph;
    esp_err_t miso_err;
    bmi088_probe_case_t cases[BMI088_PROBE_CASES];
} bmi088_probe_t;

esp_err_t drv_bmi088_init(void);
esp_err_t drv_bmi088_read(bmi088_sample_t *out);
/** Cached initialization results; does not access SPI or reinitialize the sensor. */
void drv_bmi088_get_diagnostics(bmi088_diagnostics_t *out);
/** Only allowed when initialization failed; reads IDs with weak MISO pull-up. */
esp_err_t drv_bmi088_probe(bmi088_probe_t *out);
uint8_t drv_bmi088_read_acc_id(void);
uint8_t drv_bmi088_read_gyro_id(void);

#ifdef __cplusplus
}
#endif
