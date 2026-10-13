#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Right-handed [w,x,y,z], sensor/body -> gravity-aligned world (+Z up).
 * Initial yaw is zero; heading is relative, without a magnetometer. */
typedef struct {
    float q[4];
    int64_t last_us;
    bool seeded;
} imu_attitude_t;
#define IMU_ATTITUDE_GRAVITY_GAIN 2.5f
#define IMU_ATTITUDE_GAP_US 250000
void imu_attitude_reset(imu_attitude_t *state);
/* Input: acceleration XYZ g, angular velocity XYZ deg/s, actual sample time.
 * Invalid output is identity and must be ignored. Invalid samples pause;
 * resumed/gapped samples re-seed tilt, preserving yaw, without missing-time integration. */
bool imu_attitude_step(imu_attitude_t *state, const float six_axis[6],
                       bool valid, int64_t now_us, float quaternion[4]);
#ifdef __cplusplus
}
#endif
