#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef int nvs_handle_t;
typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;
#define ESP_ERR_NVS_NOT_FOUND 0x1102
#define ESP_ERR_NVS_INVALID_LENGTH 0x110c

esp_err_t nvs_open(const char *ns, nvs_open_mode_t mode, nvs_handle_t *h);
void nvs_close(nvs_handle_t h);
esp_err_t nvs_commit(nvs_handle_t h);
esp_err_t nvs_get_i32(nvs_handle_t h, const char *key, int32_t *out);
esp_err_t nvs_get_u32(nvs_handle_t h, const char *key, uint32_t *out);
esp_err_t nvs_get_u8(nvs_handle_t h, const char *key, uint8_t *out);
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *len);
esp_err_t nvs_set_i32(nvs_handle_t h, const char *key, int32_t value);
esp_err_t nvs_set_u32(nvs_handle_t h, const char *key, uint32_t value);
esp_err_t nvs_set_u8(nvs_handle_t h, const char *key, uint8_t value);
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *value, size_t len);
