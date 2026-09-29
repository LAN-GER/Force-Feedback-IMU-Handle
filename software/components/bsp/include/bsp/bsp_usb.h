#pragma once

#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bsp_usb_line_cb_t)(const char *line, void *ctx);

void bsp_usb_set_line_cb(bsp_usb_line_cb_t cb, void *ctx);
esp_err_t bsp_usb_init(void);
void bsp_usb_poll(void);
int bsp_usb_write(const void *data, size_t len);
void bsp_usb_write_json(const char *json);

#ifdef __cplusplus
}
#endif
