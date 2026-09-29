/**
 * @file bsp_ws2812.c
 * @layer BSP
 * @brief GPIO4 WS2812
 */

#include "bsp/bsp_ws2812.h"
#include "bsp/bsp_pin.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

static const char *TAG = "ws2812";

#define WS2812_RMT_HZ 10000000u

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static bool s_ok = true;
static uint8_t s_last_r = 0xFF, s_last_g = 0xFF, s_last_b = 0xFF;

esp_err_t bsp_ws2812_init(void)
{
    rmt_tx_channel_config_t tx = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = BSP_PIN_WS2812,
        .mem_block_symbols = 64,
        .resolution_hz = WS2812_RMT_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx, &s_chan));

    rmt_bytes_encoder_config_t bytes = {
        .bit0.level0 = 1,
        .bit0.duration0 = 4,
        .bit0.level1 = 0,
        .bit0.duration1 = 8,
        .bit1.level0 = 1,
        .bit1.duration0 = 8,
        .bit1.level1 = 0,
        .bit1.duration1 = 4,
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes, &s_enc));
    ESP_ERROR_CHECK(rmt_enable(s_chan));
    ESP_LOGI(TAG, "DIN=GPIO%d", BSP_PIN_WS2812);
    return ESP_OK;
}

esp_err_t bsp_ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ok || !s_chan || !s_enc) {
        return ESP_ERR_INVALID_STATE;
    }
    if (r == s_last_r && g == s_last_g && b == s_last_b) {
        return ESP_OK;
    }

    if (rmt_tx_wait_all_done(s_chan, 0) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t cfg = {.loop_count = 0};
    esp_err_t err = rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_last_r = r;
    s_last_g = g;
    s_last_b = b;
    return ESP_OK;
}

void bsp_ws2812_off(void)
{
    s_last_r = s_last_g = s_last_b = 0xFF;
    (void)bsp_ws2812_set_rgb(0, 0, 0);
}
