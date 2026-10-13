#pragma once

#include "svc/params.h"

#ifdef __cplusplus
extern "C" {
#endif

extern svc_params_t g_params;
extern svc_params_t g_params_nvs;
extern esp_err_t g_imu_init_err;

void app_sampler_start(void);
void app_usb_init(void);
void app_protocol_wifi_line(const char *line);
void app_indicate_poll(void);
void app_user_btn_poll(void);
void app_apply_controllers_from_params(void);

#ifdef __cplusplus
}
#endif
