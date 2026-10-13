#include "app_telem_json.h"
#include "svc/params.h"
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* C3 has no FPU. Avoid printf's general double-to-decimal conversion for every
 * telemetry value: scale the IEEE binary32 significand with integer arithmetic.
 * Round to nearest, ties to even, retaining the protocol's decimal precision. */
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Telemetry requires IEEE binary32");

typedef struct { char *buf; size_t capacity, used; int failed; } writer_t;

static void append(writer_t *w, const char *s, size_t n)
{
    if (w->failed) return;
    if (n >= w->capacity - w->used) { w->failed = 1; return; }
    memcpy(w->buf + w->used, s, n);
    w->used += n;
}

static void literal(writer_t *w, const char *s) { append(w, s, strlen(s)); }

static void unsigned_number(writer_t *w, uint64_t value)
{
    char digits[20];
    size_t n = 0;
    do { digits[n++] = (char)('0' + value % 10); value /= 10; } while (value);
    while (n) append(w, &digits[--n], 1);
}

static void signed_number(writer_t *w, int32_t value)
{
    if (value < 0) literal(w, "-");
    unsigned_number(w, value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value);
}

static void fixed_number(writer_t *w, float value, unsigned decimals)
{
    static const uint32_t scales[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    unsigned exponent = (bits >> 23) & 255;
    if (exponent == 255) { literal(w, "null"); return; }
    uint32_t mantissa = bits & 0x7fffff;
    if (exponent) mantissa |= 0x800000;
    int shift = exponent ? (int)exponent - 150 : -149;
    uint64_t scaled = (uint64_t)mantissa * scales[decimals];
    if (shift >= 0) {
        if (shift >= 64 || scaled > (UINT64_MAX >> shift)) {
            /* Exceptional large finite values remain valid JSON and fit a
             * bounded field. Nine significant digits round-trip binary32. */
            char scientific[24];
            int n = snprintf(scientific, sizeof(scientific), "%.8e", (double)value);
            if (n > 0 && n < (int)sizeof(scientific)) append(w, scientific, (size_t)n);
            else w->failed = 1;
            return;
        }
        scaled <<= shift;
    } else {
        unsigned right = (unsigned)-shift;
        if (right >= 64) scaled = 0;
        else {
            uint64_t half = UINT64_C(1) << (right - 1);
            uint64_t remainder = scaled & ((half << 1) - 1);
            scaled >>= right;
            if (remainder > half || (remainder == half && (scaled & 1))) ++scaled;
        }
    }
    if (bits >> 31) literal(w, "-");
    unsigned_number(w, scaled / scales[decimals]);
    if (decimals) {
        char fraction[7];
        fraction[0] = '.';
        uint32_t remainder = (uint32_t)(scaled % scales[decimals]);
        for (unsigned i = decimals; i; --i) {
            fraction[i] = (char)('0' + remainder % 10);
            remainder /= 10;
        }
        append(w, fraction, decimals + 1);
    }
}

static void vector(writer_t *w, const float *values, unsigned n, unsigned decimals)
{
    literal(w, "[");
    for (unsigned i = 0; i < n; ++i) {
        if (i) literal(w, ",");
        fixed_number(w, values[i], decimals);
    }
    literal(w, "]");
}

size_t app_telem_json(char *buf, size_t capacity, const svc_telemetry_t *t)
{
    if (!buf || !capacity || !t) return 0;
    writer_t w = {.buf = buf, .capacity = capacity};
#define U(key, value) do { literal(&w, key); unsigned_number(&w, (value)); } while (0)
#define F(key, value, places) do { literal(&w, key); fixed_number(&w, (value), (places)); } while (0)
    literal(&w, "{\"t\":\"telem\"");
    U(",\"seq\":", t->seq);
    U(",\"imu_ok\":", t->imu_ok);
    U(",\"mode\":", t->ctrl_mode);
    U(",\"imu_acc_id\":", t->imu.acc_id);
    U(",\"imu_gyro_id\":", t->imu.gyro_id);
    const float acc[] = {t->imu.ax_g, t->imu.ay_g, t->imu.az_g};
    const float gyro[] = {t->imu.gx_dps, t->imu.gy_dps, t->imu.gz_dps};
    literal(&w, ",\"acc\":"); vector(&w, acc, 3, 4);
    literal(&w, ",\"gyro\":"); vector(&w, gyro, 3, 3);
    U(",\"imu_kf_en\":", t->imu_kf_en);
    U(",\"imu_kf\":", t->imu_kf_ready);
    literal(&w, ",\"acc_f\":"); vector(&w, t->imu_filtered, 3, 4);
    literal(&w, ",\"gyro_f\":"); vector(&w, t->imu_filtered + 3, 3, 3);
    U(",\"quat_ok\":", t->quaternion_ok);
    U(",\"quat_kf\":", t->quaternion_kf);
    literal(&w, ",\"quat\":"); vector(&w, t->quaternion, 4, 6);
    F(",\"theta\":", t->enc.theta_rad, 5);
    F(",\"omega\":", t->enc.omega_rad_s, 5);
    literal(&w, ",\"counts\":"); signed_number(&w, t->enc.counts);
    F(",\"tau\":", t->ff.tau_cmd, 4);
    F(",\"duty\":", t->applied_duty, 4);
    U(",\"ff_en\":", t->ctrl_mode == CTRL_MODE_IMPEDANCE && t->ff.enabled);
    U(",\"lim\":", t->ff.limited || t->pid.limited || t->tq.limited);
    F(",\"pid_err\":", t->pid.err, 5);
    F(",\"i_meas\":", t->i_meas, 4);
    F(",\"i_des\":", t->tq.i_des, 4);
    F(",\"tau_des\":", t->tq.tau_des, 6);
    F(",\"tau_meas\":", t->tq.tau_meas, 6);
    F(",\"i_hat\":", t->dob.i_hat, 4);
    F(",\"tau_m_hat\":", t->dob.tau_m_hat, 6);
    F(",\"tau_ext_hat\":", t->dob.tau_ext_hat, 6);
    F(",\"alpha_hat\":", t->dob.alpha_hat, 3);
    literal(&w, "}");
#undef U
#undef F
    buf[w.failed ? 0 : w.used] = '\0';
    return w.failed ? 0 : w.used;
}
