#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "drv/bmi088.h"
#include "drv/n20_encoder.h"
#include "svc/ff_impedance.h"
#include "svc/motor_pid.h"
#include "svc/motor_torque.h"
#include "svc/torque_dob.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bmi088_sample_t imu;
    float imu_filtered[6];
    bool imu_kf_en;
    bool imu_kf_ready;
    float quaternion[4]; /* [w,x,y,z], sensor -> world, relative heading. */
    bool quaternion_ok;
    bool quaternion_kf; /* Valid filtered six-axis input was used. */
    n20_encoder_sample_t enc;
    ff_state_t ff;
    pos_pid_state_t pid;
    torque_ctrl_state_t tq;
    dob_state_t dob;
    float applied_duty;
    float i_meas;
    uint8_t ctrl_mode;
    uint32_t seq;
    bool imu_ok;
} svc_telemetry_t;

esp_err_t svc_imu_ff_init(void);
void svc_imu_ff_on_sample(void);

/** 开环占空比（ctrl_mode=OFF 时由采样任务施加）。 */
void svc_imu_ff_set_openloop_duty(float duty);

void svc_imu_ff_get(svc_telemetry_t *out);

#ifdef __cplusplus
}
#endif
