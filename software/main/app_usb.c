/**
 * @file app_usb.c
 * @layer APP
 * @brief USB / Wi-Fi 共享 JSON：多模式控制 + 电流遥测 + USB 配网
 */

#include "app.h"
#include "app_wifi.h"
#include "app_telem_json.h"
#include "bsp/bsp_heater.h"
#include "bsp/bsp_usb.h"
#include "drv/n20_encoder.h"
#include "svc/params.h"
#include "svc/ff_impedance.h"
#include "svc/motor_pid.h"
#include "svc/motor_torque.h"
#include "svc/torque_dob.h"
#include "svc/imu_ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "cJSON.h"

static atomic_bool s_sub;
static SemaphoreHandle_t s_command_mutex;
/* Protected by the command mutex: replies go only to the requesting transport. */
static bool s_reply_wifi;
static TaskHandle_t s_telem_task;
static esp_timer_handle_t s_telem_timer;
static atomic_uint s_telem_work_us, s_telem_work_max_us, s_telem_skipped;
static atomic_uint s_telem_snapshot_us, s_telem_format_us, s_telem_usb_us, s_telem_wifi_us;
static atomic_uint s_telem_format_errors;
static const char *TAG = "app_usb";

static uint64_t telem_period_us(uint32_t hz)
{
    if (!hz) hz = 50;
    if (hz > 200) hz = 200;
    return 1000000ULL / hz;
}

static void on_telem_timer(void *arg)
{
    (void)arg;
    /* Keep the shared ESP_TIMER_TASK callback short; serialize on app_telem. */
    if (s_telem_task && (atomic_load(&s_sub) || app_wifi_subscribed())) {
        xTaskNotifyGive(s_telem_task);
    }
}

static void send_reply(const char *json)
{
    if (s_reply_wifi) app_wifi_reply(json);
    else bsp_usb_write_json(json);
}

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
    cJSON *root = cJSON_Parse(js);
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    bool valid = cJSON_IsString(item) && n && strlen(item->valuestring) < n;
    if (valid) memcpy(out, item->valuestring, strlen(item->valuestring) + 1);
    cJSON_Delete(root);
    return valid;
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
    cJSON *reply = cJSON_CreateObject();
    cJSON_AddStringToObject(reply, "t", "err");
    cJSON_AddStringToObject(reply, "cmd", cmd ? cmd : "");
    cJSON_AddStringToObject(reply, "msg", msg ? msg : "fail");
    if (cJSON_PrintPreallocated(reply, buf, sizeof(buf), false)) send_reply(buf);
    cJSON_Delete(reply);
}

static void reply_ok(const char *cmd)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"t\":\"ok\",\"cmd\":\"%s\"}", cmd);
    send_reply(buf);
}

static void reply_get(const char *cmd)
{
    static char buf[BSP_USB_JSON_CAPACITY];
    const svc_params_t *p = &g_params;
    int n = snprintf(
        buf, sizeof(buf),
        "{\"t\":\"ok\",\"cmd\":\"%.12s\","
        "\"live\":{\"ctrl_mode\":%u,\"ff_enable\":%u,"
        "\"k\":%.4f,\"b\":%.4f,\"theta_des\":%.4f,\"duty_max\":%.3f,"
        "\"theta_min\":%.3f,\"theta_max\":%.3f,\"enc_cpr\":%ld,\"enc_dir\":%u,"
        "\"telem_hz\":%lu,\"heater_duty\":%.3f,\"imu_kf_en\":%u,"
        "\"pid_kp\":%.4f,\"pid_ki\":%.4f,\"pid_kd\":%.4f,"
        "\"tau_des\":%.6f,\"tq_kp\":%.4f,\"tq_ki\":%.4f,\"i_max\":%.3f,\"kt\":%.6f,"
        "\"dob_enable\":%u,\"dob_vm\":%.2f,\"dob_r\":%.3f,\"dob_ke\":%.6f,\"dob_kt\":%.6f,"
        "\"dob_j\":%.9f,\"dob_b_visc\":%.9f,\"dob_tau_c\":%.6f,\"dob_fc_hz\":%.1f}}",
        cmd ? cmd : "get",
        p->ctrl_mode, p->ff_enable,
        p->k, p->b, p->theta_des, p->duty_max,
        p->theta_min, p->theta_max, (long)p->enc_cpr, p->enc_dir,
        (unsigned long)p->telem_hz, p->heater_duty, (unsigned)p->imu_kf_en,
        p->pid_kp, p->pid_ki, p->pid_kd,
        p->tau_des, p->tq_kp, p->tq_ki, p->i_max, p->kt,
        p->dob_enable, p->dob_vm, p->dob_r, p->dob_ke, p->dob_kt,
        p->dob_j, p->dob_b_visc, p->dob_tau_c, p->dob_fc_hz);
    if (n > 0 && n < (int)sizeof(buf)) {
        send_reply(buf);
    }
}

static void reply_diag(void)
{
    bmi088_diagnostics_t imu = {0};
    drv_bmi088_get_diagnostics(&imu);
    svc_telemetry_t tel = {0};
    svc_imu_ff_get(&tel);
    const esp_app_desc_t *app = esp_app_get_description();
    char hash[17];
    for (size_t i = 0; i < 8; ++i) {
        snprintf(hash + i * 2, 3, "%02x", (unsigned)app->app_elf_sha256[i]);
    }
    static char buf[BSP_USB_JSON_CAPACITY];
    int n = snprintf(buf, sizeof(buf),
        "{\"t\":\"ok\",\"cmd\":\"diag\",\"diag\":{"
        "\"firmware_sha\":\"%s\",\"uptime_ms\":%llu,\"reset_reason\":%u,\"sample_seq\":%lu,"
        "\"telem_hz\":%lu,\"telem_work_us\":%u,\"telem_work_max_us\":%u,\"telem_skipped\":%u,"
        "\"telem_snapshot_us\":%u,\"telem_format_us\":%u,\"telem_usb_us\":%u,\"telem_wifi_us\":%u,\"telem_format_errors\":%u,"
        "\"imu_ready\":%u,\"imu_init\":\"%s\",\"acc_id\":%u,\"gyro_id\":%u,"
        "\"acc_spi\":\"%s\",\"gyro_spi\":\"%s\"}}",
        hash, (unsigned long long)(esp_timer_get_time() / 1000),
        (unsigned)esp_reset_reason(), (unsigned long)tel.seq,
        (unsigned long)g_params.telem_hz, atomic_load(&s_telem_work_us),
        atomic_load(&s_telem_work_max_us), atomic_load(&s_telem_skipped),
        atomic_load(&s_telem_snapshot_us), atomic_load(&s_telem_format_us),
        atomic_load(&s_telem_usb_us), atomic_load(&s_telem_wifi_us), atomic_load(&s_telem_format_errors),
        imu.ready ? 1u : 0u, esp_err_to_name(g_imu_init_err),
        (unsigned)imu.acc_id, (unsigned)imu.gyro_id,
        esp_err_to_name(imu.acc_spi_err), esp_err_to_name(imu.gyro_spi_err));
    if (n > 0 && n < (int)sizeof(buf)) {
        send_reply(buf);
    }
}

static void reply_imu_probe(void)
{
    bmi088_probe_t probe;
    esp_err_t err = drv_bmi088_probe(&probe);
    if (err != ESP_OK) {
        reply_err("imu_probe", esp_err_to_name(err));
        return;
    }
    static char buf[BSP_USB_JSON_CAPACITY];
    int used = snprintf(buf, sizeof(buf),
        "{\"t\":\"ok\",\"cmd\":\"imu_probe\",\"probe\":{\"miso_up\":%d,\"miso_down\":%d,\"miso_err\":%d,"
        "\"miso_input\":%d,\"miso_oe\":%d,\"miso_periph\":%d,\"cases\":[",
        probe.miso_up, probe.miso_down, (int)probe.miso_err,
        probe.miso_input, probe.miso_oe, probe.miso_periph);
    for (size_t i = 0; i < BMI088_PROBE_CASES; ++i) {
        const bmi088_probe_case_t *c = &probe.cases[i];
        if (used < 0 || used >= (int)sizeof(buf)) break;
        int n = snprintf(buf + used, sizeof(buf) - used,
            "%s{\"mode\":%u,\"hz\":%lu,\"acc\":[%u,%u,%u],\"gyro\":[%u,%u],\"ae\":%d,\"ge\":%d}",
            i ? "," : "", (unsigned)c->mode, (unsigned long)c->hz,
            (unsigned)c->acc_raw[0], (unsigned)c->acc_raw[1], (unsigned)c->acc_raw[2],
            (unsigned)c->gyro_raw[0], (unsigned)c->gyro_raw[1], (int)c->acc_err, (int)c->gyro_err);
        if (n < 0) { used = -1; break; }
        used += n;
    }
    if (used > 0 && used + 4 <= (int)sizeof(buf)) {
        memcpy(buf + used, "]}}", 4);
        send_reply(buf);
    } else {
        reply_err("imu_probe", "response too long");
    }
}

static void push_telem(void)
{
    int64_t start = esp_timer_get_time();
    svc_telemetry_t t = {0};
    svc_imu_ff_get(&t);
    int64_t snapshot = esp_timer_get_time();
    static char buf[BSP_USB_JSON_CAPACITY];
    size_t n = app_telem_json(buf, sizeof(buf), &t);
    int64_t formatted = esp_timer_get_time();
    if (n && atomic_load(&s_sub)) bsp_usb_write_json(buf);
    int64_t usb = esp_timer_get_time();
    if (n) app_wifi_push_json(buf);
    else atomic_fetch_add(&s_telem_format_errors, 1);
    int64_t wifi = esp_timer_get_time();
    /* Wall times include preemption by the sampler and other higher tasks.
     * Wi-Fi measures queue submission; network delivery happens asynchronously. */
    atomic_store(&s_telem_snapshot_us, (unsigned)(snapshot - start));
    atomic_store(&s_telem_format_us, (unsigned)(formatted - snapshot));
    atomic_store(&s_telem_usb_us, (unsigned)(usb - formatted));
    atomic_store(&s_telem_wifi_us, (unsigned)(wifi - usb));
}

static void telem_loop(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notified > 1) atomic_fetch_add(&s_telem_skipped, notified - 1);
        if (atomic_load(&s_sub) || app_wifi_subscribed()) {
            int64_t started = esp_timer_get_time();
            push_telem();
            unsigned work_us = (unsigned)(esp_timer_get_time() - started);
            atomic_store(&s_telem_work_us, work_us);
            if (work_us > atomic_load(&s_telem_work_max_us)) atomic_store(&s_telem_work_max_us, work_us);
        }
        /* Drop timer notifications accumulated during a slow format/write.
         * Wait for a future slot instead of sending a burst of stale snapshots. */
        uint32_t skipped = ulTaskNotifyTake(pdTRUE, 0);
        if (skipped) atomic_fetch_add(&s_telem_skipped, skipped);
    }
}

static void handle_line(const char *line)
{
    char cmd[24] = {0};
    if (!json_str(line, "cmd", cmd, sizeof(cmd))) {
        reply_err("?", "missing cmd");
        return;
    }

    if (strcmp(cmd, "get") == 0) {
        reply_get("get");
        return;
    }
    if (strcmp(cmd, "wifi_get") == 0 || strcmp(cmd, "wifi_set") == 0 ||
        strcmp(cmd, "wifi_enable") == 0 || strcmp(cmd, "wifi_forget") == 0) {
        if (strcmp(cmd, "wifi_get") != 0) {
            if (s_reply_wifi) { reply_err(cmd, "USB provisioning only"); return; }
            esp_err_t err = strcmp(cmd, "wifi_set") == 0 ? app_wifi_configure(line) :
                strcmp(cmd, "wifi_enable") == 0 ? app_wifi_enable(line) : app_wifi_forget();
            if (err != ESP_OK) { reply_err(cmd, esp_err_to_name(err)); return; }
        }
        char buf[BSP_USB_JSON_CAPACITY];
        app_wifi_status_json(buf, sizeof(buf), cmd, !s_reply_wifi);
        send_reply(buf);
        return;
    }
    if (strcmp(cmd, "diag") == 0) {
        reply_diag();
        return;
    }
    if (strcmp(cmd, "imu_filter") == 0) {
        float v;
        if (!json_f(line, "v", &v) || (v != 0.0f && v != 1.0f)) {
            reply_err(cmd, "v must be 0 or 1");
            return;
        }
        /* Filter state is reset by its owning sampler; no motor reconfiguration. */
        g_params.imu_kf_en = (uint8_t)v;
        reply_get(cmd);
        return;
    }
    if (strcmp(cmd, "imu_probe") == 0) {
        reply_imu_probe();
        return;
    }
    if (strcmp(cmd, "sub") == 0) {
        reply_ok("sub");
        if (s_reply_wifi) app_wifi_subscribe(true);
        else atomic_store(&s_sub, true);
        return;
    }
    if (strcmp(cmd, "unsub") == 0) {
        if (s_reply_wifi) app_wifi_subscribe(false);
        else atomic_store(&s_sub, false);
        reply_ok("unsub");
        return;
    }
    if (strcmp(cmd, "zero") == 0) {
        drv_n20_encoder_zero();
        svc_dob_reset();
        svc_pos_pid_reset();
        svc_torque_ctrl_reset();
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
        float v = 0.0f;
        if (!json_f(line, "v", &v)) {
            reply_err("duty", "need v");
            return;
        }
        g_params.ctrl_mode = CTRL_MODE_OFF;
        g_params.ff_enable = 0;
        app_apply_controllers_from_params();
        svc_imu_ff_set_openloop_duty(v);
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
        bool mode_set = false;
        if (json_i32(line, "ctrl_mode", &i) && i >= 0 && i <= CTRL_MODE_TORQUE) {
            g_params.ctrl_mode = (uint8_t)i;
            mode_set = true;
        }
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
            /* 兼容旧字段：1→阻抗，0→若未显式设 mode 则 OFF */
            if (i) {
                g_params.ctrl_mode = CTRL_MODE_IMPEDANCE;
            } else if (!mode_set && g_params.ctrl_mode == CTRL_MODE_IMPEDANCE) {
                g_params.ctrl_mode = CTRL_MODE_OFF;
            }
        }
        if (json_i32(line, "telem_hz", &i) && i >= 1 && i <= 200) {
            if ((uint32_t)i != g_params.telem_hz) {
                esp_err_t err = esp_timer_restart(s_telem_timer, telem_period_us((uint32_t)i));
                if (err != ESP_OK) { reply_err("set", esp_err_to_name(err)); return; }
            }
            g_params.telem_hz = (uint32_t)i;
        }
        if (json_f(line, "heater_duty", &f)) {
            g_params.heater_duty = f;
        }
        if (json_f(line, "pid_kp", &f)) {
            g_params.pid_kp = f;
        }
        if (json_f(line, "pid_ki", &f)) {
            g_params.pid_ki = f;
        }
        if (json_f(line, "pid_kd", &f)) {
            g_params.pid_kd = f;
        }
        if (json_f(line, "tau_des", &f)) {
            g_params.tau_des = f;
        }
        if (json_f(line, "tq_kp", &f)) {
            g_params.tq_kp = f;
        }
        if (json_f(line, "tq_ki", &f)) {
            g_params.tq_ki = f;
        }
        if (json_f(line, "i_max", &f)) {
            g_params.i_max = f;
        }
        if (json_f(line, "kt", &f)) {
            g_params.kt = f;
            g_params.dob_kt = f;
        }
        if (json_i32(line, "dob_enable", &i)) {
            g_params.dob_enable = i ? 1 : 0;
        }
        if (json_f(line, "dob_vm", &f)) {
            g_params.dob_vm = f;
        }
        if (json_f(line, "dob_r", &f)) {
            g_params.dob_r = f;
        }
        if (json_f(line, "dob_ke", &f)) {
            g_params.dob_ke = f;
        }
        if (json_f(line, "dob_kt", &f)) {
            g_params.dob_kt = f;
            g_params.kt = f;
        }
        if (json_f(line, "dob_j", &f)) {
            g_params.dob_j = f;
        }
        if (json_f(line, "dob_b_visc", &f)) {
            g_params.dob_b_visc = f;
        }
        if (json_f(line, "dob_tau_c", &f)) {
            g_params.dob_tau_c = f;
        }
        if (json_f(line, "dob_fc_hz", &f)) {
            g_params.dob_fc_hz = f;
        }

        if (g_params.ctrl_mode != CTRL_MODE_OFF) {
            svc_imu_ff_set_openloop_duty(0.0f);
        }
        if (g_params.ctrl_mode == CTRL_MODE_POS_PID) {
            svc_pos_pid_reset();
        }
        if (g_params.ctrl_mode == CTRL_MODE_TORQUE) {
            svc_torque_ctrl_reset();
        }
        app_apply_controllers_from_params();
        reply_get("set");
        return;
    }

    reply_err(cmd, "unknown");
}

static void process_line(const char *line, bool wifi)
{
    if (!s_command_mutex) return;
    xSemaphoreTake(s_command_mutex, portMAX_DELAY);
    s_reply_wifi = wifi;
    handle_line(line);
    xSemaphoreGive(s_command_mutex);
}

static void on_line(const char *line, void *ctx)
{
    (void)ctx;
    process_line(line, false);
}

void app_protocol_wifi_line(const char *line)
{
    process_line(line, true);
}

void app_usb_init(void)
{
    s_command_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_command_mutex ? ESP_OK : ESP_ERR_NO_MEM);
    bsp_usb_set_line_cb(on_line, NULL);
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(telem_loop, "app_telem", 4096, NULL, 5,
                                          &s_telem_task, 0) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    const esp_timer_create_args_t args = {
        .callback = on_telem_timer,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "telem_rate",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_telem_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_telem_timer, telem_period_us(g_params.telem_hz)));
    ESP_LOGI(TAG, "USB JSON ready (get/sub), payload_capacity=%u", (unsigned)BSP_USB_JSON_CAPACITY);
}
