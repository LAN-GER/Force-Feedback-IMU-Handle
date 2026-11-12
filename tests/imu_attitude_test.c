#include "svc/imu_attitude.h"
#include "svc/imu_kalman.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define near(a, b, eps) check_near((a), (b), (eps), __LINE__)
static void check_near(float a, float b, float eps, int line)
{
    if (!(fabsf(a-b) < eps)) fprintf(stderr, "line %d: actual %.9g, expected %.9g (tolerance %.9g)\n", line, a, b, eps);
    assert(fabsf(a-b) < eps);
}
static float yaw(const float q[4])
{ return atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])); }
static float roll(const float q[4])
{ return atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])); }
static float pitch(const float q[4])
{ return asinf(2*(q[0]*q[2]-q[3]*q[1])); }
static bool fresh_step(imu_attitude_t *s, const float raw[6], bool valid, int64_t us, float q[4])
{ return imu_attitude_step(s, raw, NULL, valid, true, us, q); }

#define RAD 0.017453292519943295f

static void stationary_bias(bool kf_enabled)
{
    imu_attitude_t s;
    imu_kalman_t kf;
    imu_attitude_reset(&s);
    imu_kalman_reset(&kf);
    const float angle = 25*RAD;
    float raw[6] = {0, sinf(angle), cosf(angle), 0.8f, -0.4f, 0}, filtered[6], q[4];
    for (int i = 0; i <= 45000; ++i) {
        bool fresh = i % 10 == 0; /* Accel 100 Hz, gyro 1 kHz. */
        if (fresh) raw[1] = sinf(angle) + 0.002f*sinf(i*0.024f);
        raw[3] = 0.8f + 0.015f*(i*0.001f) + 0.07f*sinf(i*0.071f);
        raw[4] = -0.4f*cosf(angle);
        raw[5] = 0.4f*sinf(angle); /* Bias lies across gravity, not along heading. */
        bool ready = imu_kalman_step(&kf, raw, true, kf_enabled, fresh, i*1000LL, filtered);
        assert(imu_attitude_step(&s, raw, ready ? filtered : NULL, true, fresh, i*1000LL, q));
        if (i >= 10000 && i % 1000 == 0) {
            near(roll(q), angle, 0.07f*RAD);
            near(pitch(q), 0, 0.07f*RAD);
        }
    }
}

static void stale_gravity(void)
{
    imu_attitude_t s;
    imu_attitude_reset(&s);
    float raw[6] = {0, 0, 1, 0, 0, 0}, q[4];
    assert(!imu_attitude_step(&s, raw, NULL, true, false, 0, q));
    assert(fresh_step(&s, raw, true, 0, q));
    raw[3] = 1.0f;
    for (int i = 1; i <= 1000; ++i) {
        assert(imu_attitude_step(&s, raw, NULL, true, false, i*1000LL, q));
    }
    near(roll(q), RAD, 0.02f*RAD); /* No correction from the old 1 g reference. */
    assert(!imu_attitude_step(&s, raw, NULL, true, false, 1400000, q));
    assert(fresh_step(&s, raw, true, 1410000, q));
    near(roll(q), 0, 1e-5f);
}

static void slow_motion(void)
{
    imu_attitude_t s;
    imu_kalman_t kf;
    imu_attitude_reset(&s);
    imu_kalman_reset(&kf);
    float raw[6] = {0, 0, 1, 1, 0, 0}, filtered[6], q[4];
    for (int i = 0; i <= 12000; ++i) {
        bool fresh = i % 10 == 0;
        if (fresh) {
            float angle = i*0.001f*RAD;
            raw[1] = sinf(angle);
            raw[2] = cosf(angle);
        }
        assert(imu_kalman_step(&kf, raw, true, true, fresh, i*1000LL, filtered));
        assert(imu_attitude_step(&s, raw, filtered, true, fresh, i*1000LL, q));
    }
    near(roll(q), 12*RAD, 0.3f*RAD); /* A real slow roll still moves. */

    imu_attitude_reset(&s);
    raw[1] = raw[3] = 0; raw[2] = 1; raw[5] = 0.6f;
    for (int i = 0; i <= 10000; ++i) {
        assert(imu_attitude_step(&s, raw, NULL, true, i % 10 == 0, i*1000LL, q));
    }
    near(yaw(q), 6*RAD, 0.01f*RAD); /* Gravity cannot distinguish this from yaw bias. */
}

static void manual_gyro_calibration(void)
{
    imu_attitude_t s;
    imu_gyro_cal_t cal;
    imu_attitude_reset(&s);
    /* Real near-vertical posture: gravity is mostly X, so X gyro bias
     * appears as relative roll after the page zeroes at this orientation. */
    float raw[6] = {.984f, .035f, -.012f, .15f, .08f, -.06f}, q[4];
    imu_gyro_cal_begin(&cal, 0);
    for (int i = 0; i <= 5000; ++i) {
        bool fresh = i % 10 == 0;
        raw[3] = .15f + .25f*sinf(i*0.07f);
        assert(imu_gyro_cal_step(&cal, &s, raw, true, fresh, i*1000LL) ==
               (i == 5000 ? IMU_GYRO_CAL_DONE : IMU_GYRO_CAL_RUNNING));
        assert(imu_attitude_step(&s, raw, NULL, true, fresh, i*1000LL, q));
    }
    near(s.gyro_bias_rad_s[0], .15f*RAD, .002f*RAD);
    near(s.gyro_bias_rad_s[1], .08f*RAD, .002f*RAD);
    near(s.gyro_bias_rad_s[2], -.06f*RAD, .002f*RAD);

    raw[3] = .15f;
    float yaw_after = yaw(q);
    for (int i = 5001; i <= 35000; ++i) {
        assert(imu_attitude_step(&s, raw, NULL, true, i % 10 == 0, i*1000LL, q));
    }
    near(yaw(q), yaw_after, .08f*RAD); /* Previously about 4.5 degrees in 30 s. */

    float bias_x = s.gyro_bias_rad_s[0];
    imu_gyro_cal_begin(&cal, 0);
    assert(imu_gyro_cal_step(&cal, &s, raw, true, true, 0) == IMU_GYRO_CAL_RUNNING);
    raw[3] = 10;
    assert(imu_gyro_cal_step(&cal, &s, raw, true, false, 1000) == IMU_GYRO_CAL_MOTION);
    raw[3] = .15f;
    imu_gyro_cal_begin(&cal, 0);
    assert(imu_gyro_cal_step(&cal, &s, raw, true, true, 0) == IMU_GYRO_CAL_RUNNING);
    for (int i = 1; i <= 50; ++i) {
        assert(imu_gyro_cal_step(&cal, &s, raw, true, false, i*1000LL) == IMU_GYRO_CAL_RUNNING);
    }
    assert(imu_gyro_cal_step(&cal, &s, raw, true, false, 51000) == IMU_GYRO_CAL_NO_DATA);
    assert(s.gyro_bias_rad_s[0] == bias_x); /* Failure never overwrites prior calibration. */
}

int main(void)
{
    imu_attitude_t s;
    float q[4], raw[6]={0,0,1,0,0,0};
    const float pi=3.14159265358979323846f;
    imu_attitude_reset(&s);
    assert(fresh_step(&s, raw, true, 0, q));
    near(q[0],1,1e-6f);
    for (int axis=0; axis<3; ++axis) {
        imu_attitude_reset(&s);
        raw[0]=raw[1]=0; raw[2]=1; raw[3]=raw[4]=raw[5]=0;
        assert(fresh_step(&s,raw,true,0,q));
        raw[2]=2; /* Disable gravity correction to isolate gyro integration. */
        raw[3+axis]=90;
        int64_t now=0;
        for (int i=0; now<1000000; ++i) {
            int64_t dt=(i%2) ? 3000 : 1000;
            if (now+dt>1000000) dt=1000000-now;
            now+=dt;
            assert(fresh_step(&s,raw,true,now,q));
        }
        near(q[0],sqrtf(0.5f),1e-4f); near(q[1+axis],sqrtf(0.5f),1e-4f);
        float norm=0; for(int i=0;i<4;++i) norm+=q[i]*q[i];
        near(norm,1,1e-5f);
    }
    /* Seed a known roll/pitch (30/30 degrees), no startup ramp from zero. */
    imu_attitude_reset(&s);
    raw[0]=-0.5f; raw[1]=sqrtf(3)/4; raw[2]=0.75f;
    raw[3]=raw[4]=raw[5]=0;
    assert(fresh_step(&s,raw,true,0,q));
    near(asinf(2*(q[0]*q[2]-q[3]*q[1])),pi/6,1e-5f);
    near(atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2])),pi/6,1e-5f);
    /* Gravity restores roll/pitch during stationary sampling. */
    raw[0]=raw[1]=0; raw[2]=1;
    for (int i=1;i<=10000;++i) {
        assert(fresh_step(&s,raw,true,i*1000LL,q));
        if (i == 4000) near(roll(q),0,0.1f*RAD);
    }
    near(q[1],0,1e-5f); near(q[2],0,1e-5f);
    /* Yaw integrates independently from gravity, is not forced to north. */
    imu_attitude_reset(&s); assert(fresh_step(&s,raw,true,0,q));
    raw[5]=90;
    for(int i=1;i<=1000;++i) assert(fresh_step(&s,raw,true,i*1000LL,q));
    near(yaw(q),pi/2,1e-4f);
    float before=yaw(q);
    assert(fresh_step(&s,raw,true,1000100,q)); near(yaw(q),before,1e-6f);
    assert(!fresh_step(&s,raw,false,1001000,q)); near(q[0],1,1e-6f);
    assert(fresh_step(&s,raw,true,2000000,q)); near(yaw(q),before,1e-5f);
    assert(fresh_step(&s,raw,true,3000000,q)); near(yaw(q),before,1e-5f);
    raw[3]=NAN; assert(!fresh_step(&s,raw,true,3001000,q));
    raw[3]=0; raw[5]=0;
    assert(fresh_step(&s,raw,true,3002000,q));
    assert(fresh_step(&s,raw,true,0,q)); near(yaw(q),0,1e-5f);
    imu_attitude_reset(&s); raw[2]=0;
    assert(!fresh_step(&s,raw,true,0,q));
    raw[2]=-1; assert(fresh_step(&s,raw,true,1000,q));
    near(fabsf(q[1]),1,1e-5f); /* Upside down remains a valid pose. */
    stationary_bias(false);
    stationary_bias(true);
    stale_gravity();
    slow_motion();
    manual_gyro_calibration();
    puts("PASS: three-axis static calibration near vertical, motion/missing data rejection, stationary tilt bias, stale gravity, slow rotations and quaternion regressions");
    return 0;
}
