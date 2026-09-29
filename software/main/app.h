#pragma once

#include "svc/params.h"

#ifdef __cplusplus
extern "C" {
#endif

extern svc_params_t g_params;
extern svc_params_t g_params_nvs;

void app_sampler_start(void);
void app_usb_init(void);
void app_indicate_poll(void);
void app_user_btn_poll(void);

#ifdef __cplusplus
}
#endif
