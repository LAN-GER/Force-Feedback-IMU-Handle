#include "svc/imu_kalman.h"
#include <math.h>
#include <string.h>

void imu_kalman_reset(imu_kalman_t *s)
{
    memset(s, 0, sizeof(*s));
}

bool imu_kalman_step(imu_kalman_t *s, const float raw[6], bool valid,
                     bool enabled, bool acc_new, int64_t now_us, float out[6])
{
    memset(out, 0, 6 * sizeof(*out));
    for (int i = 0; i < 6; ++i) valid = valid && isfinite(raw[i]);
    if (!valid || !enabled || now_us < 0) {
        imu_kalman_reset(s);
        return false;
    }
    for (int group = 0; group < 2; ++group) {
        int64_t age = now_us - s->updated_us[group];
        if (s->seeded[group] && (age < 0 || age > IMU_KALMAN_GAP_US)) {
            s->seeded[group] = false;
        }
        bool fresh = group == 0 ? acc_new : true;
        /* Coalesced sampler notifications can produce duplicate gyro polls. */
        if (group == 1 && s->seeded[group] && age < 500) fresh = false;
        if (!fresh) continue;
        float r = group == 0 ? IMU_KALMAN_ACC_R : IMU_KALMAN_GYRO_R;
        float q = group == 0 ? IMU_KALMAN_ACC_Q : IMU_KALMAN_GYRO_Q;
        float dt = (float)age * 1.0e-6f;
        for (int i = group * 3; i < group * 3 + 3; ++i) {
            if (!s->seeded[group]) {
                s->x[i] = raw[i];
                s->p[i] = r;
            } else {
                float predicted_p = s->p[i] + q * dt;
                float gain = predicted_p / (predicted_p + r);
                s->x[i] += gain * (raw[i] - s->x[i]);
                s->p[i] = r * predicted_p / (predicted_p + r);
            }
        }
        s->updated_us[group] = now_us;
        s->seeded[group] = true;
    }
    if (!s->seeded[0] || !s->seeded[1]) return false;
    memcpy(out, s->x, sizeof(s->x));
    return true;
}
