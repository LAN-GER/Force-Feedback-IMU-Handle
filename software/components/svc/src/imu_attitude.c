#include "svc/imu_attitude.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292519943295f
#define GRAVITY_MAX_AGE_US 50000
#define STILL_DWELL_US 750000
#define STILL_ACCEL_DELTA_G 0.02f
#define STILL_TILT_ERROR_SIN 0.022f /* About 1.25 degrees. */
#define STILL_GYRO_RAD_S (3.0f * DEG_TO_RAD)
#define BIAS_GAIN 1.5f
#define GYRO_CAL_DURATION_US 5000000
#define GYRO_CAL_MAX_DPS 3.0f

static float norm_squared(const float v[3])
{
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

static bool gravity_norm(float n2)
{
    return n2 >= 0.7f*0.7f && n2 <= 1.3f*1.3f;
}

bool imu_gyro_bias_in_range(const float rad_s[3])
{
    for (int i = 0; i < 3; ++i) {
        /* The comparisons also reject NaN and infinity. */
        if (!(fabsf(rad_s[i]) <= IMU_GYRO_BIAS_LIMIT_RAD_S)) return false;
    }
    return true;
}

static float clamp_bias(float bias)
{
    if (bias > IMU_GYRO_BIAS_LIMIT_RAD_S) return IMU_GYRO_BIAS_LIMIT_RAD_S;
    if (bias < -IMU_GYRO_BIAS_LIMIT_RAD_S) return -IMU_GYRO_BIAS_LIMIT_RAD_S;
    return bias;
}

void imu_gyro_cal_begin(imu_gyro_cal_t *cal, int64_t now_us)
{
    *cal = (imu_gyro_cal_t){.start_us = now_us, .last_us = now_us, .acc_us = -1};
}

imu_gyro_cal_status_t imu_gyro_cal_step(imu_gyro_cal_t *cal, imu_attitude_t *attitude,
    const float raw[6], bool valid, bool acc_new, int64_t now_us)
{
    if (!valid || now_us < cal->last_us || now_us - cal->last_us > GRAVITY_MAX_AGE_US) {
        return IMU_GYRO_CAL_NO_DATA;
    }
    for (int i = 0; i < 6; ++i) if (!isfinite(raw[i])) return IMU_GYRO_CAL_NO_DATA;
    if (norm_squared(raw + 3) >= GYRO_CAL_MAX_DPS*GYRO_CAL_MAX_DPS) return IMU_GYRO_CAL_MOTION;

    int64_t last_acc = cal->acc_us < 0 ? cal->start_us : cal->acc_us;
    if (now_us - last_acc > GRAVITY_MAX_AGE_US) return IMU_GYRO_CAL_NO_DATA;
    if (acc_new) {
        float n2 = norm_squared(raw);
        if (n2 < 0.9f*0.9f || n2 > 1.1f*1.1f) return IMU_GYRO_CAL_MOTION;
        if (cal->acc_us < 0) {
            memcpy(cal->initial_acc, raw, sizeof(cal->initial_acc));
            cal->start_us = cal->last_us = now_us;
        } else {
            float delta[3];
            for (int i = 0; i < 3; ++i) delta[i] = raw[i] - cal->initial_acc[i];
            if (norm_squared(delta) > STILL_ACCEL_DELTA_G*STILL_ACCEL_DELTA_G) return IMU_GYRO_CAL_MOTION;
        }
        cal->acc_us = now_us;
    }
    int64_t dt = now_us - cal->last_us;
    if (cal->acc_us < 0 || dt < 500) return IMU_GYRO_CAL_RUNNING;

    float seconds = (float)dt * 1.0e-6f;
    for (int i = 0; i < 3; ++i) cal->gyro_integral[i] += raw[3+i] * seconds;
    cal->last_us = now_us;
    int64_t elapsed = now_us - cal->start_us;
    if (elapsed < GYRO_CAL_DURATION_US) return IMU_GYRO_CAL_RUNNING;
    float scale = DEG_TO_RAD / ((float)elapsed * 1.0e-6f);
    for (int i = 0; i < 3; ++i) attitude->gyro_bias_rad_s[i] = clamp_bias(cal->gyro_integral[i] * scale);
    attitude->still_since_us = -1;
    return IMU_GYRO_CAL_DONE;
}

static bool normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (!isfinite(n) || n < 1.0e-6f) return false;
    float inv_norm = 1.0f / n;
    for (int i = 0; i < 4; ++i) q[i] *= inv_norm;
    return true;
}
static void reset_pose(imu_attitude_t *s)
{
    s->q[0] = 1.0f;
    s->q[1] = s->q[2] = s->q[3] = 0.0f;
    s->last_us = -1;
    s->gravity_us = -1;
    s->still_since_us = -1;
    s->seeded = false;
}

void imu_attitude_reset(imu_attitude_t *s)
{
    memset(s, 0, sizeof(*s));
    reset_pose(s);
}

bool imu_attitude_restore_gyro_bias(imu_attitude_t *s, const imu_gyro_bias_t *bias)
{
    if (!bias || !bias->valid || !imu_gyro_bias_in_range(bias->rad_s)) return false;
    memcpy(s->gyro_bias_rad_s, bias->rad_s, sizeof(bias->rad_s));
    s->still_since_us = -1;
    return true;
}
static void seed_tilt(imu_attitude_t *s, const float a[3])
{
    float yaw = s->seeded ? atan2f(2*(s->q[0]*s->q[3] + s->q[1]*s->q[2]),
                                  1 - 2*(s->q[2]*s->q[2] + s->q[3]*s->q[3])) : 0;
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], hypotf(a[1], a[2]));
    float cr = cosf(roll/2), sr = sinf(roll/2), cp = cosf(pitch/2), sp = sinf(pitch/2);
    float cy = cosf(yaw/2), sy = sinf(yaw/2);
    float q[4] = {cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                  cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy};
    float dot = 0; /* Keep sign continuous; q and -q describe the same pose. */
    for (int i = 0; i < 4; ++i) dot += s->q[i]*q[i];
    for (int i = 0; i < 4; ++i) s->q[i] = dot < 0 ? -q[i] : q[i];
    s->seeded = true;
}
static void update_gravity(imu_attitude_t *s, const float raw[6], const float input[6],
                           bool acc_new, int64_t now_us)
{
    /* Use unfiltered motion to leave the quiet window immediately. In
     * particular, filtered gyro can lag at the onset of a rotation. */
    float gyro[3];
    for (int i = 0; i < 3; ++i) gyro[i] = raw[3+i]*DEG_TO_RAD - s->gyro_bias_rad_s[i];
    bool quiet_gyro = norm_squared(gyro) < STILL_GYRO_RAD_S*STILL_GYRO_RAD_S;
    if (!quiet_gyro || s->gravity_us < 0 || now_us - s->gravity_us > GRAVITY_MAX_AGE_US) {
        s->still_since_us = -1;
    }
    if (!acc_new) return;

    float raw_n2 = norm_squared(raw);
    float n2 = input == raw ? raw_n2 : norm_squared(input);
    if (!gravity_norm(raw_n2) || !gravity_norm(n2)) {
        s->gravity_us = s->still_since_us = -1;
        return;
    }
    float inv_norm = 1.0f / sqrtf(n2);
    for (int i = 0; i < 3; ++i) s->gravity[i] = input[i]*inv_norm;
    s->gravity_us = now_us;

    if (!quiet_gyro || raw_n2 < 0.9f*0.9f || raw_n2 > 1.1f*1.1f) {
        s->still_since_us = -1;
        return;
    }
    float delta[3];
    for (int i = 0; i < 3; ++i) delta[i] = raw[i] - s->still_acc_g[i];
    if (s->still_since_us < 0 || norm_squared(delta) > STILL_ACCEL_DELTA_G*STILL_ACCEL_DELTA_G) {
        memcpy(s->still_acc_g, raw, sizeof(s->still_acc_g));
        s->still_since_us = now_us;
    }
}

bool imu_attitude_step(imu_attitude_t *s, const float raw[6], const float filtered[6],
                       bool valid, bool acc_new, int64_t now_us, float out[4])
{
    if (!s || !raw || !out) return false;
    const float *input = filtered ? filtered : raw;
    out[0] = 1; out[1] = out[2] = out[3] = 0;
    for (int i = 0; i < 6; ++i) valid = valid && isfinite(raw[i]);
    if (filtered) {
        for (int i = 0; i < 6; ++i) valid = valid && isfinite(filtered[i]);
    }
    if (!valid || now_us < 0) {
        s->last_us = s->gravity_us = s->still_since_us = -1;
        return false;
    }
    if (s->last_us >= 0 && now_us < s->last_us) reset_pose(s);
    int64_t age = s->last_us < 0 ? 0 : now_us - s->last_us;
    bool reseed = !s->seeded || s->last_us < 0 || age > IMU_ATTITUDE_GAP_US;
    if (reseed) s->gravity_us = s->still_since_us = -1;
    update_gravity(s, raw, input, acc_new, now_us);
    bool gravity = s->gravity_us >= 0 && now_us - s->gravity_us <= GRAVITY_MAX_AGE_US;
    if (reseed) {
        if (!gravity) { s->last_us = -1; return false; }
        seed_tilt(s, s->gravity);
        s->last_us = now_us;
    } else if (age >= 500) {
        unsigned steps = ((unsigned)age + 9999) / 10000;
        float dt = (float)age * 1.0e-6f;
        if (steps > 1) dt /= steps;
        float half_dt = dt * 0.5f;
        bool learn_bias = gravity && s->still_since_us >= 0 &&
                          now_us - s->still_since_us >= STILL_DWELL_US;
        for (unsigned step = 0; step < steps; ++step) {
            float w=s->q[0], x=s->q[1], y=s->q[2], z=s->q[3];
            float omega[3];
            for (int i = 0; i < 3; ++i) omega[i] = input[3+i]*DEG_TO_RAD - s->gyro_bias_rad_s[i];
            if (gravity) {
                float ax=s->gravity[0], ay=s->gravity[1], az=s->gravity[2];
                float gx=2*(x*z-w*y), gy=2*(w*x+y*z), gz=1-2*(x*x+y*y);
                const float error[3] = {ay*gz-az*gy, az*gx-ax*gz, ax*gy-ay*gx};
                if (s->still_since_us >= 0 &&
                    (norm_squared(error) > STILL_TILT_ERROR_SIN*STILL_TILT_ERROR_SIN || ax*gx+ay*gy+az*gz < 0)) {
                    /* Let proportional correction recover from a large tilt
                     * error before attributing the remaining error to bias. */
                    s->still_since_us = -1;
                    learn_bias = false;
                }
                for (int i = 0; i < 3; ++i) {
                    omega[i] += IMU_ATTITUDE_GRAVITY_GAIN*error[i];
                    if (learn_bias) {
                        /* Gravity observes tilt, so do not learn a yaw bias
                         * by assuming every low gyro reading is zero. */
                        float bias = s->gyro_bias_rad_s[i] - BIAS_GAIN*dt*error[i];
                        s->gyro_bias_rad_s[i] = clamp_bias(bias);
                    }
                }
            }
            float wx=omega[0], wy=omega[1], wz=omega[2];
            s->q[0] += (-x*wx-y*wy-z*wz)*half_dt;
            s->q[1] += ( w*wx+y*wz-z*wy)*half_dt;
            s->q[2] += ( w*wy-x*wz+z*wx)*half_dt;
            s->q[3] += ( w*wz+x*wy-y*wx)*half_dt;
            if (!normalize(s->q)) { reset_pose(s); return false; }
        }
        s->last_us = now_us;
    }
    memcpy(out, s->q, sizeof(s->q));
    return true;
}
