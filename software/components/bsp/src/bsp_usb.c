/**
 * @file bsp_usb.c
 * @layer BSP
 * @brief USB-Serial/JTAG 行缓冲。PC 工具走 JSON 行协议。
 */

#include "bsp/bsp_usb.h"
#include "bsp/bsp_gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define USB_LINE_MAX     256
#define USB_TX_BUF_SIZE  2048
#define USB_RX_BUF_SIZE  512

static const char *TAG = "usb";
static char s_line[USB_LINE_MAX];
static size_t s_len;
static bool s_discard_line;
static bsp_usb_line_cb_t s_cb;
static void *s_ctx;

esp_err_t bsp_usb_init(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_vfs_use_nonblocking();
        esp_err_t err = usb_serial_jtag_driver_uninstall();
        if (err != ESP_OK) {
            return err;
        }
    }
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = USB_TX_BUF_SIZE;
    cfg.rx_buffer_size = USB_RX_BUF_SIZE;
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB 驱动安装失败 %s", esp_err_to_name(err));
        return err;
    }
    /* Route ESP_LOG/stdout through the same TX ring buffer as JSON. */
    usb_serial_jtag_vfs_use_driver();
    ESP_LOGI(TAG, "USB-Serial/JTAG tx_buf=%d rx_buf=%d", USB_TX_BUF_SIZE, USB_RX_BUF_SIZE);
    return ESP_OK;
}

void bsp_usb_set_line_cb(bsp_usb_line_cb_t cb, void *ctx)
{
    s_cb = cb;
    s_ctx = ctx;
}

static void feed(uint8_t b)
{
    if (b == '\r') {
        return;
    }
    if (b == '\n') {
        if (!s_discard_line && s_len > 0 && s_cb) {
            s_line[s_len] = '\0';
            s_cb(s_line, s_ctx);
        }
        s_len = 0;
        s_discard_line = false;
        return;
    }
    if (s_discard_line) {
        return;
    }
    if (s_len + 1 < USB_LINE_MAX) {
        s_line[s_len++] = (char)b;
    } else {
        s_len = 0;
        s_discard_line = true;
    }
}

void bsp_usb_poll(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        return;
    }
    uint8_t buf[64];
    int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (n <= 0) {
        return;
    }
    bsp_user_led_pulse();
    for (int i = 0; i < n; i++) {
        feed(buf[i]);
    }
}

int bsp_usb_write(const void *data, size_t len)
{
    if (!usb_serial_jtag_is_driver_installed() || data == NULL || len == 0) {
        return -1;
    }
    if (!bsp_console_tx_lock()) {
        return -1;
    }
    bsp_user_led_pulse();
    /* Enqueue one whole frame; a full TX buffer must not stall RX for seconds. */
    int written = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(100));
    bsp_console_tx_unlock();
    return written;
}

void bsp_usb_write_json(const char *json)
{
    char line[BSP_USB_JSON_CAPACITY + 3]; /* >> + JSON + LF + NUL */
    int n = snprintf(line, sizeof(line), ">>%s\n", json ? json : "{}");
    if (n <= 0 || n >= (int)sizeof(line)) {
        ESP_LOGW(TAG, "JSON 行过长 need=%d capacity=%u", n, (unsigned)sizeof(line));
        return;
    }
    int w = bsp_usb_write(line, (size_t)n);
    if (w != n) {
        static int64_t s_last_warn_us;
        int64_t now = esp_timer_get_time();
        if (now - s_last_warn_us > 2000000) {
            s_last_warn_us = now;
            ESP_LOGW(TAG, "JSON 行发送失败 wrote=%d need=%d", w, n);
        }
    }
}
