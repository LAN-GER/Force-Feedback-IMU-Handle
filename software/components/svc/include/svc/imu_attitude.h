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
    float gyro_bias_rad_s[3];
    float gravity[3]; /* Latest usable acceleration direction. */
    float still_acc_g[3]; /* Raw acceleration at the start of a quiet interval. */
    int64_t last_us;
    int64_t gravity_us;
    int64_t still_since_us;
    bool seeded;
} imu_attitude_t;
#define IMU_ATTITUDE_GRAVITY_GAIN 2.5f
#define IMU_ATTITUDE_GAP_US 250000
#define IMU_GYRO_BIAS_LIMIT_RAD_S 0.05235988f /* 3 degrees per second. */

typedef struct {
    float rad_s[3];
    bool valid;
} imu_gyro_bias_t;

bool imu_gyro_bias_in_range(const float rad_s[3]);
void imu_attitude_reset(imu_attitude_t *state);
bool imu_attitude_restore_gyro_bias(imu_attitude_t *state, const imu_gyro_bias_t *bias);
/* Raw acceleration XYZ g and angular velocity XYZ deg/s; filtered may be NULL.
 * Use raw data to gate bias learning even when integrating filtered input.
 * acc_new is true only for a newly acquired acceleration sample. After an
 * invalid sample/gap, tilt re-seeds from fresh gravity without integrating the gap. */
bool imu_attitude_step(imu_attitude_t *state, const float raw[6], const float filtered[6],
                       bool valid, bool acc_new, int64_t now_us, float quaternion[4]);

typedef enum {
    IMU_GYRO_CAL_IDLE, IMU_GYRO_CAL_RUNNING, IMU_GYRO_CAL_DONE,
    IMU_GYRO_CAL_MOTION, IMU_GYRO_CAL_NO_DATA,
} imu_gyro_cal_status_t;

typedef struct {
    int64_t start_us, last_us, acc_us;
    float initial_acc[3];
    float gyro_integral[3];
} imu_gyro_cal_t;

/* Explicit calibration while the user holds the board still. The sampler
 * commits all three gyro biases only on success, without resetting attitude. */
void imu_gyro_cal_begin(imu_gyro_cal_t *cal, int64_t now_us);
imu_gyro_cal_status_t imu_gyro_cal_step(imu_gyro_cal_t *cal, imu_attitude_t *attitude,
    const float raw[6], bool valid, bool acc_new, int64_t now_us);
#ifdef __cplusplus
}
#endif
