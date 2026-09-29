/**
 * @file bsp_usb.c
 * @layer BSP
 * @brief USB-Serial/JTAG 行缓冲。PC 工具走 JSON 行协议。
 */

#include "bsp/bsp_usb.h"
#include "bsp/bsp_gpio.h"
#include "driver/usb_serial_jtag.h"
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
#define USB_TX_CHUNK     64

static const char *TAG = "usb";
static char s_line[USB_LINE_MAX];
static size_t s_len;
static bsp_usb_line_cb_t s_cb;
static void *s_ctx;

esp_err_t bsp_usb_init(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        (void)usb_serial_jtag_driver_uninstall();
    }
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = USB_TX_BUF_SIZE;
    cfg.rx_buffer_size = USB_RX_BUF_SIZE;
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB 驱动安装失败 %s，JSON 配置口可能不可用", esp_err_to_name(err));
        return ESP_OK;
    }
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
        if (s_len > 0 && s_cb) {
            s_line[s_len] = '\0';
            s_cb(s_line, s_ctx);
        }
        s_len = 0;
        return;
    }
    if (s_len + 1 < USB_LINE_MAX) {
        s_line[s_len++] = (char)b;
    } else {
        s_len = 0;
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
    bsp_console_tx_lock();
    bsp_user_led_pulse();
    const uint8_t *p = (const uint8_t *)data;
    size_t left = len;

    while (left > 0) {
        size_t want = left > USB_TX_CHUNK ? USB_TX_CHUNK : left;
        int w = 0;
        for (int attempt = 0; attempt < 80; attempt++) {
            w = usb_serial_jtag_write_bytes(p, want, pdMS_TO_TICKS(30));
            if (w > 0) {
                break;
            }
            (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(30));
            vTaskDelay(1);
        }
        if (w <= 0) {
            bsp_console_tx_unlock();
            return (int)(len - left);
        }
        p += (size_t)w;
        left -= (size_t)w;
    }
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    bsp_console_tx_unlock();
    return (int)len;
}

void bsp_usb_write_json(const char *json)
{
    char line[640];
    int n = snprintf(line, sizeof(line), ">>%s\n", json ? json : "{}");
    if (n <= 0 || n >= (int)sizeof(line)) {
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
