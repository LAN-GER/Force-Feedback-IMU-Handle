/* Native tests against the same C implementation linked into the firmware. */
#include "svc/imu_kalman.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    imu_kalman_t s = {0};
    float raw[6] = {0, 0, 1, 0, 0, 0}, out[6];
    assert(!imu_kalman_step(&s, raw, true, true, false, 0, out));
    assert(imu_kalman_step(&s, raw, true, true, true, 1000, out));
    assert(out[2] == 1); /* Gravity is seeded immediately, not ramped from zero. */
    float acc_p = s.p[2];
    raw[2] = 9;
    for (int i = 2; i < 10; ++i) {
        assert(imu_kalman_step(&s, raw, true, true, false, i * 1000, out));
        assert(out[2] == 1 && s.p[2] == acc_p);
    }
    raw[2] = 1;
    double raw_energy = 0, filtered_energy = 0;
    for (int i = 1; i <= 2000; ++i) {
        raw[0] = (i % 2 ? .1f : -.1f);
        raw[3] = (i % 2 ? 2.0f : -2.0f);
        assert(imu_kalman_step(&s, raw, true, true, true, 10000LL + i * 10000LL, out));
        assert(out[1] == 0 && out[2] == 1 && out[4] == 0 && out[5] == 0);
        assert(s.p[0] > 0 && s.p[3] > 0);
        if (i > 100) {
            raw_energy += raw[0] * raw[0] + raw[3] * raw[3];
            filtered_energy += out[0] * out[0] + out[3] * out[3];
        }
    }
    assert(filtered_energy < raw_energy * .05);
    raw[0] = 1; raw[3] = 10;
    for (int i = 1; i <= 100; ++i) {
        assert(imu_kalman_step(&s, raw, true, true, true, 20010000LL + i * 10000LL, out));
    }
    assert(fabsf(out[0] - 1) < .001f && fabsf(out[3] - 10) < .001f);

    assert(!imu_kalman_step(&s, raw, true, false, true, 21020000LL, out));
    assert(out[0] == 0 && !s.seeded[0] && !s.seeded[1]);
    raw[0] = 4;
    assert(imu_kalman_step(&s, raw, true, true, true, 21030000LL, out));
    assert(out[0] == 4);
    assert(!imu_kalman_step(&s, raw, false, true, true, 21040000LL, out));
    assert(!s.seeded[0]);
    assert(imu_kalman_step(&s, raw, true, true, true, 21050000LL, out));
    raw[0] = NAN;
    assert(!imu_kalman_step(&s, raw, true, true, true, 21060000LL, out));
    assert(out[0] == 0 && !s.seeded[0]);
    raw[0] = 2;
    assert(imu_kalman_step(&s, raw, true, true, true, 22000000LL, out));
    assert(!imu_kalman_step(&s, raw, true, true, false, 22300000LL, out));
    assert(!s.seeded[0] && out[0] == 0); /* No fresh acceleration for 300ms. */
    raw[0] = 7;
    assert(imu_kalman_step(&s, raw, true, true, true, 22301000LL, out));
    assert(out[0] == 7);
    raw[0] = -3;
    assert(imu_kalman_step(&s, raw, true, true, true, 1, out));
    assert(out[0] == -3); /* Time reversal resets stale estimates. */

    imu_kalman_t fast = {0}, slow = {0};
    raw[0] = 0;
    assert(imu_kalman_step(&fast, raw, true, true, true, 0, out));
    assert(imu_kalman_step(&slow, raw, true, true, true, 0, out));
    raw[0] = 1;
    assert(imu_kalman_step(&fast, raw, true, true, true, 1000, out));
    float fast_response = out[0];
    assert(imu_kalman_step(&slow, raw, true, true, true, 100000, out));
    assert(out[0] > fast_response); /* Q scales with actual elapsed time. */
    puts("IMU Kalman: seed, duplicate samples, noise, step, independent axes, toggle, invalid data, gaps, time scaling passed");
    return 0;
}
