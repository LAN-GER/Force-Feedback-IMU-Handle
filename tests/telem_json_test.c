/* Include implementation to compare the internal decimal writer with libc. */
#include "../software/main/app_telem_json.c"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

static void number_test(float value, unsigned precision)
{
    char actual[96] = {0}, expected[400];
    writer_t w = {.buf = actual, .capacity = sizeof(actual)};
    fixed_number(&w, value, precision);
    assert(!w.failed);
    actual[w.used] = 0;
    if (!isfinite(value)) assert(strcmp(actual, "null") == 0);
    else if (strchr(actual, 'e')) assert(strtof(actual, NULL) == value);
    else {
        snprintf(expected, sizeof(expected), "%.*f", (int)precision, (double)value);
        if (strcmp(actual, expected)) {
            fprintf(stderr, "Mismatch: %.9g p=%u actual=%s expected=%s\n",
                    (double)value, precision, actual, expected);
            abort();
        }
    }
}

int main(void)
{
    /* Exact half cases, signed zero, denormals, carry, negative and extreme values. */
    const float edges[] = {0, -0.0f, 0.5f, 1.5f, 2.5f, -2.5f, 0.125f,
        9.999999f, -9.999999f, FLT_MIN, FLT_MAX, -FLT_MAX, INFINITY, -INFINITY, NAN};
    for (unsigned p = 0; p <= 6; ++p)
        for (unsigned i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i) number_test(edges[i], p);
    uint32_t bits = 1;
    for (unsigned i = 0; i < 100000; ++i) {
        bits = bits * 1664525u + 1013904223u;
        float value;
        memcpy(&value, &bits, sizeof(value));
        for (unsigned p = 0; p <= 6; ++p) number_test(value, p);
    }

    svc_telemetry_t t = {
        .seq = UINT32_MAX, .imu_ok = true, .ctrl_mode = CTRL_MODE_IMPEDANCE,
        .imu = {.ax_g = -1.25f, .ay_g = 2.5f, .az_g = 3.75f,
                .gx_dps = -4.5f, .gy_dps = 5.25f, .gz_dps = 6.125f, .acc_id = 30, .gyro_id = 15},
        .imu_filtered = {7.5f, 8.25f, -9.125f, 10.5f, -11.25f, 12.125f},
        .imu_kf_en = true, .imu_kf_ready = true,
        .quaternion = {0.5f, -0.25f, 0.125f, -0.0625f}, .quaternion_ok = true, .quaternion_kf = true,
        .enc = {.theta_rad = 13.25f, .omega_rad_s = -14.5f, .counts = INT32_MIN},
        .ff = {.tau_cmd = 15.75f, .enabled = true}, .applied_duty = -0.5f,
        .pid = {.err = -16.25f, .limited = true}, .i_meas = 17.5f,
        .tq = {.i_des = -18.75f, .tau_des = 19.25f, .tau_meas = -20.5f},
        .dob = {.i_hat = 21.75f, .tau_m_hat = -22.25f, .tau_ext_hat = 23.5f, .alpha_hat = -24.75f},
    };
    const char *golden = "{\"t\":\"telem\",\"seq\":4294967295,\"imu_ok\":1,\"mode\":1,"
        "\"imu_acc_id\":30,\"imu_gyro_id\":15,\"acc\":[-1.2500,2.5000,3.7500],"
        "\"gyro\":[-4.500,5.250,6.125],\"imu_kf_en\":1,\"imu_kf\":1,"
        "\"acc_f\":[7.5000,8.2500,-9.1250],\"gyro_f\":[10.500,-11.250,12.125],"
        "\"quat_ok\":1,\"quat_kf\":1,\"quat\":[0.500000,-0.250000,0.125000,-0.062500],"
        "\"theta\":13.25000,\"omega\":-14.50000,\"counts\":-2147483648,"
        "\"tau\":15.7500,\"duty\":-0.5000,\"ff_en\":1,\"lim\":1,\"pid_err\":-16.25000,"
        "\"i_meas\":17.5000,\"i_des\":-18.7500,\"tau_des\":19.250000,\"tau_meas\":-20.500000,"
        "\"i_hat\":21.7500,\"tau_m_hat\":-22.250000,\"tau_ext_hat\":23.500000,\"alpha_hat\":-24.750}";
    char buf[1024];
    size_t length = app_telem_json(buf, sizeof(buf), &t);
    assert(length == strlen(golden));
    assert(strcmp(buf, golden) == 0);
    for (size_t capacity = 1; capacity <= length + 2; ++capacity) {
        memset(buf, '#', sizeof(buf));
        size_t n = app_telem_json(buf, capacity, &t);
        assert(buf[capacity] == '#');
        if (capacity <= length) assert(n == 0 && buf[0] == 0);
        else assert(n == length && strcmp(buf, golden) == 0);
    }
    assert(app_telem_json(NULL, 100, &t) == 0);
    assert(app_telem_json(buf, 0, &t) == 0);
    assert(app_telem_json(buf, sizeof(buf), NULL) == 0);
    t.ctrl_mode = CTRL_MODE_POS_PID;
    t.pid.limited = false;
    t.tq.limited = true;
    t.enc.counts = INT32_MAX;
    t.imu.ax_g = NAN;
    t.imu.gx_dps = FLT_MAX;
    assert(app_telem_json(buf, sizeof(buf), &t));
    assert(strstr(buf, "\"ff_en\":0,\"lim\":1"));
    assert(strstr(buf, "\"counts\":2147483647"));
    assert(strstr(buf, "\"acc\":[null,"));
    puts("PASS: 700000 randomized decimal comparisons, IEEE edge cases, telemetry golden frame, bounds and flags");
    return 0;
}
