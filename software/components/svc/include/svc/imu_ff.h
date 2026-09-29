#pragma once

#include <stdbool.h>
#include "drv/bmi088.h"
#include "drv/n20_encoder.h"
#include "svc/ff_impedance.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bmi088_sample_t imu;
    n20_encoder_sample_t enc;
    ff_state_t ff;
    uint32_t seq;
    bool imu_ok;
} svc_telemetry_t;

esp_err_t svc_imu_ff_init(void);
void svc_imu_ff_on_sample(void); /* 由 1 kHz 定时器任务调用 */
void svc_imu_ff_get(svc_telemetry_t *out);

#ifdef __cplusplus
}
#endif
