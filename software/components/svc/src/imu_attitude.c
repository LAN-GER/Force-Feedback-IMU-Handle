#include "svc/imu_attitude.h"
#include <math.h>
#include <string.h>
static bool normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (!isfinite(n) || n < 1.0e-6f) return false;
    for (int i = 0; i < 4; ++i) q[i] /= n;
    return true;
}
void imu_attitude_reset(imu_attitude_t *s)
{
    memset(s, 0, sizeof(*s));
    s->q[0] = 1.0f;
    s->last_us = -1;
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
bool imu_attitude_step(imu_attitude_t *s, const float raw[6], bool valid,
                       int64_t now_us, float out[4])
{
    if (!s || !raw || !out) return false;
    out[0] = 1; out[1] = out[2] = out[3] = 0;
    for (int i = 0; i < 6; ++i) valid = valid && isfinite(raw[i]);
    if (!valid || now_us < 0) { s->last_us = -1; return false; }
    if (s->last_us >= 0 && now_us < s->last_us) imu_attitude_reset(s);
    float norm = hypotf(hypotf(raw[0], raw[1]), raw[2]);
    bool gravity = norm >= 0.7f && norm <= 1.3f;
    int64_t age = s->last_us < 0 ? 0 : now_us - s->last_us;
    if (!s->seeded || s->last_us < 0 || age > IMU_ATTITUDE_GAP_US) {
        if (!gravity) { s->last_us = -1; return false; }
        seed_tilt(s, raw);
        s->last_us = now_us;
    } else if (age >= 500) {
        float dt = (float)age * 1.0e-6f;
        int steps = (int)ceilf(dt/0.01f);
        float half_dt = dt / (2*steps);
        for (int step = 0; step < steps; ++step) {
            float w=s->q[0], x=s->q[1], y=s->q[2], z=s->q[3];
            float wx=raw[3]*0.017453292519943295f;
            float wy=raw[4]*0.017453292519943295f;
            float wz=raw[5]*0.017453292519943295f;
            if (gravity) {
                float ax=raw[0]/norm, ay=raw[1]/norm, az=raw[2]/norm;
                float gx=2*(x*z-w*y), gy=2*(w*x+y*z), gz=1-2*(x*x+y*y);
                wx += IMU_ATTITUDE_GRAVITY_GAIN*(ay*gz-az*gy);
                wy += IMU_ATTITUDE_GRAVITY_GAIN*(az*gx-ax*gz);
                wz += IMU_ATTITUDE_GRAVITY_GAIN*(ax*gy-ay*gx);
            }
            s->q[0] += (-x*wx-y*wy-z*wz)*half_dt;
            s->q[1] += ( w*wx+y*wz-z*wy)*half_dt;
            s->q[2] += ( w*wy-x*wz+z*wx)*half_dt;
            s->q[3] += ( w*wz+x*wy-y*wx)*half_dt;
            if (!normalize(s->q)) { imu_attitude_reset(s); return false; }
        }
        s->last_us = now_us;
    }
    memcpy(out, s->q, sizeof(s->q));
    return true;
}
