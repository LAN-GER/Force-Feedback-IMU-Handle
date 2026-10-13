#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Independent scalar random-walk filters: acceleration (g), gyro (deg/s).
 * Q is variance per second; R is measurement variance, in physical units. */
#define IMU_KALMAN_ACC_Q 0.04f
#define IMU_KALMAN_ACC_R 0.0025f
#define IMU_KALMAN_GYRO_Q 4.0f
#define IMU_KALMAN_GYRO_R 1.0f
#define IMU_KALMAN_GAP_US 250000

typedef struct {
    float x[6];
    float p[6];
    int64_t updated_us[2];
    bool seeded[2];
} imu_kalman_t;

void imu_kalman_reset(imu_kalman_t *s);
/* Call only from the sampler. Acceleration updates only when acc_new is true.
 * Returns true when both groups have fresh, initialized filter estimates.
 * Invalid/disabled output is zero, and must not be advertised as valid. */
bool imu_kalman_step(imu_kalman_t *s, const float raw[6], bool valid,
                     bool enabled, bool acc_new, int64_t now_us, float out[6]);
