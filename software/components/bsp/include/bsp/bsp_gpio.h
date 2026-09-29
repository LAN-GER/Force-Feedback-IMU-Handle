#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_gpio_init(void);

/** 用户键按下为 true（低电平有效，内部上拉）。与 BOOT 共用 GPIO9。 */
bool bsp_user_btn_pressed(void);

void bsp_user_led_pulse(void);
void bsp_console_activity_poll(void);
void bsp_console_tx_lock(void);
void bsp_console_tx_unlock(void);

#ifdef __cplusplus
}
#endif
