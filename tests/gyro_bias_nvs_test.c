#include "svc/params.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Minimal flash stand-in. Persist bytes across fresh svc_params_init calls. */
static unsigned char saved[16], staged[16];
static size_t saved_len, staged_len;
static uint32_t telem_hz = 100, staged_hz;
static bool commit_fails;
static unsigned closed, committed;

esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { assert(!"unexpected erase"); return ESP_FAIL; }
const char *esp_err_to_name(esp_err_t err) { (void)err; return "mock error"; }
esp_err_t nvs_open(const char *ns, nvs_open_mode_t mode, nvs_handle_t *h)
{
    assert(strcmp(ns, "ffh") == 0);
    *h = mode;
    staged_len = 0;
    staged_hz = telem_hz;
    return ESP_OK;
}
void nvs_close(nvs_handle_t h) { (void)h; ++closed; }
esp_err_t nvs_commit(nvs_handle_t h)
{
    assert(h == NVS_READWRITE);
    if (commit_fails) return ESP_FAIL;
    if (staged_len) {
        memcpy(saved, staged, staged_len);
        saved_len = staged_len;
    }
    telem_hz = staged_hz;
    ++committed;
    return ESP_OK;
}

esp_err_t nvs_get_i32(nvs_handle_t h, const char *key, int32_t *out)
{ (void)h; (void)key; (void)out; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_get_u8(nvs_handle_t h, const char *key, uint8_t *out)
{ (void)h; (void)key; (void)out; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_get_u32(nvs_handle_t h, const char *key, uint32_t *out)
{
    assert(h == NVS_READONLY);
    if (strcmp(key, "thz") != 0) return ESP_ERR_NVS_NOT_FOUND;
    *out = telem_hz;
    return ESP_OK;
}
esp_err_t nvs_set_i32(nvs_handle_t h, const char *key, int32_t value)
{ assert(h == NVS_READWRITE); (void)key; (void)value; return ESP_OK; }
esp_err_t nvs_set_u8(nvs_handle_t h, const char *key, uint8_t value)
{ assert(h == NVS_READWRITE); (void)key; (void)value; return ESP_OK; }
esp_err_t nvs_set_u32(nvs_handle_t h, const char *key, uint32_t value)
{ assert(h == NVS_READWRITE); if (strcmp(key, "thz") == 0) staged_hz = value; return ESP_OK; }
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *len)
{
    assert(h == NVS_READONLY);
    if (!saved_len || strcmp(key, "gyro_bias_v1") != 0) return ESP_ERR_NVS_NOT_FOUND;
    if (*len < saved_len) return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(out, saved, saved_len);
    *len = saved_len;
    return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *value, size_t len)
{
    assert(h == NVS_READWRITE && strcmp(key, "gyro_bias_v1") == 0 && len == 12);
    memcpy(staged, value, len);
    staged_len = len;
    return ESP_OK;
}

int main(void)
{
    const float rad = 0.017453292519943295f;
    const float raw[6] = {1, 0, 0, .15f, -.08f, .06f};
    svc_params_t params, boot;
    assert(svc_params_init(&params) == ESP_OK);
    assert(!params.imu_gyro_bias.valid && params.telem_hz == 100);
    assert(svc_params_save(&params) == ESP_OK && saved_len == 0); /* Old NVS remains valid. */
    params.imu_gyro_bias = (imu_gyro_bias_t){.valid = true,
        .rad_s = {raw[3]*rad, raw[4]*rad, raw[5]*rad}};
    assert(svc_params_save(&params) == ESP_OK && saved_len == 12);

    assert(svc_params_init(&boot) == ESP_OK && boot.imu_gyro_bias.valid);
    assert(memcmp(boot.imu_gyro_bias.rad_s, params.imu_gyro_bias.rad_s, 12) == 0);
    imu_attitude_t attitude;
    imu_attitude_reset(&attitude);
    assert(imu_attitude_restore_gyro_bias(&attitude, &boot.imu_gyro_bias));
    float first[4], q[4];
    assert(imu_attitude_step(&attitude, raw, NULL, true, true, 0, first));
    for (int i = 1; i <= 1000; ++i) {
        assert(imu_attitude_step(&attitude, raw, NULL, true, i % 10 == 0, i*1000LL, q));
    }
    for (int i = 0; i < 4; ++i) assert(fabsf(first[i] - q[i]) < 1e-6f);
    assert(imu_attitude_step(&attitude, raw, NULL, true, true, 0, q));
    assert(attitude.gyro_bias_rad_s[0] == params.imu_gyro_bias.rad_s[0]);

    commit_fails = true;
    params.imu_gyro_bias.rad_s[0] = -.01f;
    unsigned prev_closed = closed;
    assert(svc_params_save(&params) == ESP_FAIL && closed == prev_closed + 1 && committed == 2);
    commit_fails = false;
    assert(svc_params_init(&boot) == ESP_OK && boot.imu_gyro_bias.rad_s[0] == raw[3]*rad);

    params.imu_gyro_bias.rad_s[0] = NAN;
    assert(svc_params_save(&params) == ESP_ERR_INVALID_ARG);
    memcpy(saved, params.imu_gyro_bias.rad_s, 12);
    assert(svc_params_init(&boot) == ESP_OK && !boot.imu_gyro_bias.valid && boot.telem_hz == 100);
    params.imu_gyro_bias.rad_s[0] = 0.1f;
    memcpy(saved, params.imu_gyro_bias.rad_s, 12);
    assert(svc_params_init(&boot) == ESP_OK && !boot.imu_gyro_bias.valid);
    saved_len = 8;
    assert(svc_params_init(&boot) == ESP_OK && !boot.imu_gyro_bias.valid);
    puts("PASS: legacy NVS, gyro bias round trip and startup fusion, invalid blob, save failure");
    return 0;
}
