#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t app_wifi_init(void);
esp_err_t app_wifi_configure(const char *json);
/* USB-only persistent switch; preserves credentials and access key. */
esp_err_t app_wifi_enable(const char *json);
esp_err_t app_wifi_forget(void);
/* Status never includes the router password; only USB may retrieve the access key. */
void app_wifi_status_json(char *out, size_t size, const char *cmd, bool usb);
bool app_wifi_subscribed(void);
void app_wifi_subscribe(bool enabled);
void app_wifi_reply(const char *json);
void app_wifi_push_json(const char *json);
