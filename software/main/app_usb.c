/**
 * @file app_usb.c
 * @layer APP
 * @brief USB JSON 行协议：get/set/zero/save/sub/unsub/duty
 *
 * PC 发一行 JSON（以 \\n 结束），设备回 `>>{...}\\n`。
 */

#include "app.h"
#include "bsp/bsp_heater.h"
#include "bsp/bsp_usb.h"
#include "drv/drv8833.h"
#include "drv/n20_encoder.h"
#include "svc/params.h"
#include "svc/ff_impedance.h"
#include "svc/imu_ff.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static bool s_sub;
static TaskHandle_t s_telem_task;

static const char *find_key(const char *js, const char *key)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(js, pat);
    if (!p) {
        return NULL;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p != ':') {
        return NULL;
    }
    p++;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    return p;
}

static bool json_str(const char *js, const char *key, char *out, size_t n)
{
    const char *p = find_key(js, key);
    if (!p || *p != '"' || n == 0) {
        return false;
    }
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < n) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return true;
}

static bool json_i32(const char *js, const char *key, int32_t *out)
{
    const char *p = find_key(js, key);
    if (!p) {
        return false;
    }
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) {
        return false;
    }
    *out = (int32_t)v;
    return true;
}

static bool json_f(const char *js, const char *key, float *out)
{
    const char *p = find_key(js, key);
    if (!p) {
        return false;
    }
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

static void reply_err(const char *cmd, const char *msg)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"t\":\"err\",\"cmd\":\"%s\",\"msg\":\"%s\"}",
             cmd ? cmd : "", msg ? msg : "fail");
    bsp_usb_write_json(buf);
}

static void reply_ok(const char *cmd)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"t\":\"ok\",\"cmd\":\"%s\"}", cmd);
    bsp_usb_write_json(buf);
}

static void apply_live_to_controllers(void)
{
    svc_ff_set_gains(g_params.k, g_params.b, g_params.duty_max);
    svc_ff_set_target(g_params.theta_des);
    svc_ff_set_limits(g_params.theta_min, g_params.theta_max);
    svc_ff_enable(g_params.ff_enable != 0);
    drv_n20_encoder_set_cpr(g_params.enc_cpr);
    drv_n20_encoder_set_dir(g_params.enc_dir);
    bsp_heater_set_duty(g_params.heater_duty);
}

static void reply_get(const char *cmd)
{
    static char buf[480];
    const svc_params_t *p = &g_params;
    int n = snprintf(
        buf, sizeof(buf),
        "{\"t\":\"ok\",\"cmd\":\"%.12s\","
        "\"live\":{\"k\":%.4f,\"b\":%.4f,\"theta_des\":%.4f,\"duty_max\":%.3f,"
        "\"theta_min\":%.3f,\"theta_max\":%.3f,\"enc_cpr\":%ld,\"enc_dir\":%u,"
        "\"ff_enable\":%u,\"telem_hz\":%lu,\"heater_duty\":%.3f}}",
        cmd ? cmd : "get",
        p->k, p->b, p->theta_des, p->duty_max,
        p->theta_min, p->theta_max, (long)p->enc_cpr, p->enc_dir,
        p->ff_enable, (unsigned long)p->telem_hz, p->heater_duty);
    if (n > 0 && n < (int)sizeof(buf)) {
        bsp_usb_write_json(buf);
    }
}

static void push_telem(void)
{
    svc_telemetry_t t;
    svc_imu_ff_get(&t);
    static char buf[512];
    int n = snprintf(
        buf, sizeof(buf),
        "{\"t\":\"telem\",\"seq\":%lu,\"imu_ok\":%u,"
        "\"acc\":[%.4f,%.4f,%.4f],\"gyro\":[%.3f,%.3f,%.3f],"
        "\"theta\":%.5f,\"omega\":%.5f,\"counts\":%ld,"
        "\"tau\":%.4f,\"duty\":%.4f,\"ff_en\":%u,\"lim\":%u}",
        (unsigned long)t.seq, t.imu_ok ? 1u : 0u,
        t.imu.ax_g, t.imu.ay_g, t.imu.az_g,
        t.imu.gx_dps, t.imu.gy_dps, t.imu.gz_dps,
        t.enc.theta_rad, t.enc.omega_rad_s, (long)t.enc.counts,
        t.ff.tau_cmd, t.ff.duty, t.ff.enabled ? 1u : 0u, t.ff.limited ? 1u : 0u);
    if (n > 0 && n < (int)sizeof(buf)) {
        bsp_usb_write_json(buf);
    }
}

static void telem_loop(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_sub) {
            push_telem();
            uint32_t hz = g_params.telem_hz ? g_params.telem_hz : 50;
            if (hz > 200) {
                hz = 200;
            }
            vTaskDelay(pdMS_TO_TICKS(1000 / hz));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void on_line(const char *line, void *ctx)
{
    (void)ctx;
    char cmd[24] = {0};
    if (!json_str(line, "cmd", cmd, sizeof(cmd))) {
        reply_err("?", "missing cmd");
        return;
    }

    if (strcmp(cmd, "get") == 0) {
        reply_get("get");
        return;
    }
    if (strcmp(cmd, "sub") == 0) {
        s_sub = true;
        reply_ok("sub");
        return;
    }
    if (strcmp(cmd, "unsub") == 0) {
        s_sub = false;
        reply_ok("unsub");
        return;
    }
    if (strcmp(cmd, "zero") == 0) {
        drv_n20_encoder_zero();
        reply_ok("zero");
        return;
    }
    if (strcmp(cmd, "save") == 0) {
        g_params_nvs = g_params;
        if (svc_params_save(&g_params_nvs) != ESP_OK) {
            reply_err("save", "nvs");
            return;
        }
        reply_ok("save");
        return;
    }
    if (strcmp(cmd, "duty") == 0) {
        /* 开环占空比测试：{"cmd":"duty","v":0.3}，同时强制关闭闭环 */
        float v = 0.0f;
        if (!json_f(line, "v", &v)) {
            reply_err("duty", "need v");
            return;
        }
        g_params.ff_enable = 0;
        svc_ff_enable(false);
        drv_drv8833_enable(true);
        drv_drv8833_set_duty(v);
        reply_ok("duty");
        return;
    }
    if (strcmp(cmd, "heater") == 0) {
        float v = 0.0f;
        if (!json_f(line, "v", &v)) {
            reply_err("heater", "need v");
            return;
        }
        g_params.heater_duty = v;
        bsp_heater_set_duty(v);
        reply_get("heater");
        return;
    }
    if (strcmp(cmd, "set") == 0) {
        float f;
        int32_t i;
        if (json_f(line, "k", &f)) {
            g_params.k = f;
        }
        if (json_f(line, "b", &f)) {
            g_params.b = f;
        }
        if (json_f(line, "theta_des", &f)) {
            g_params.theta_des = f;
        }
        if (json_f(line, "duty_max", &f)) {
            g_params.duty_max = f;
        }
        if (json_f(line, "theta_min", &f)) {
            g_params.theta_min = f;
        }
        if (json_f(line, "theta_max", &f)) {
            g_params.theta_max = f;
        }
        if (json_i32(line, "enc_cpr", &i) && i > 0) {
            g_params.enc_cpr = i;
        }
        if (json_i32(line, "enc_dir", &i)) {
            g_params.enc_dir = i ? 1 : 0;
        }
        if (json_i32(line, "ff_enable", &i)) {
            g_params.ff_enable = i ? 1 : 0;
        }
        if (json_i32(line, "telem_hz", &i) && i >= 1 && i <= 200) {
            g_params.telem_hz = (uint32_t)i;
        }
        if (json_f(line, "heater_duty", &f)) {
            g_params.heater_duty = f;
        }
        apply_live_to_controllers();
        reply_get("set");
        return;
    }

    reply_err(cmd, "unknown");
}

void app_usb_init(void)
{
    bsp_usb_set_line_cb(on_line, NULL);
    xTaskCreatePinnedToCore(telem_loop, "app_telem", 4096, NULL, 5, &s_telem_task, 0);
}
