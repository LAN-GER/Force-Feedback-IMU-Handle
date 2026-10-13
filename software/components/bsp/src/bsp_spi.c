/**
 * @file bsp_spi.c
 * @layer BSP
 * @brief BMI088 用 SPI2，Mode 3，双片选手动控制
 */

#include "bsp/bsp_spi.h"
#include "bsp/bsp_pin.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "bsp_spi";
static spi_device_handle_t s_spi;
static spi_device_handle_t s_spi_mode0;

static gpio_num_t cs_pin(bsp_spi_cs_t which)
{
    return (which == BSP_SPI_CS_ACC) ? BSP_PIN_IMU_CS_ACC : BSP_PIN_IMU_CS_GYRO;
}

esp_err_t bsp_spi_init(void)
{
    gpio_config_t cs = {
        .pin_bit_mask = (1ULL << BSP_PIN_IMU_CS_ACC) | (1ULL << BSP_PIN_IMU_CS_GYRO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cs);
    gpio_set_level(BSP_PIN_IMU_CS_ACC, 1);
    gpio_set_level(BSP_PIN_IMU_CS_GYRO, 1);

    spi_bus_config_t bus = {
        .mosi_io_num = BSP_PIN_IMU_MOSI,
        .miso_io_num = BSP_PIN_IMU_MISO,
        .sclk_io_num = BSP_PIN_IMU_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = BSP_SPI_HZ,
        .mode = 3, /* CPOL=1, CPHA=1 */
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(BSP_SPI_HOST, &dev, &s_spi));
    dev.mode = 0;
    ESP_ERROR_CHECK(spi_bus_add_device(BSP_SPI_HOST, &dev, &s_spi_mode0));
    ESP_LOGI(TAG, "SPI Mode3 %d Hz SCK=%d MISO=%d MOSI=%d CS_ACC=%d CS_GYRO=%d",
             BSP_SPI_HZ, BSP_PIN_IMU_SCK, BSP_PIN_IMU_MISO, BSP_PIN_IMU_MOSI,
             BSP_PIN_IMU_CS_ACC, BSP_PIN_IMU_CS_GYRO);
    return ESP_OK;
}

void bsp_spi_cs_low(bsp_spi_cs_t which)
{
    gpio_set_level(cs_pin(which), 0);
}

void bsp_spi_cs_high(bsp_spi_cs_t which)
{
    gpio_set_level(cs_pin(which), 1);
}

void bsp_spi_cs_high_all(void)
{
    gpio_set_level(BSP_PIN_IMU_CS_ACC, 1);
    gpio_set_level(BSP_PIN_IMU_CS_GYRO, 1);
}

static gpio_pull_mode_t saved_pull(const gpio_io_config_t *cfg)
{
    return cfg->pu ? (cfg->pd ? GPIO_PULLUP_PULLDOWN : GPIO_PULLUP_ONLY)
                   : (cfg->pd ? GPIO_PULLDOWN_ONLY : GPIO_FLOATING);
}

static esp_err_t transfer(bsp_spi_cs_t cs, const uint8_t *tx, uint8_t *rx, size_t len,
                          uint32_t hz, uint8_t mode, bool weak_pullup)
{
    if (!tx || !rx || len == 0 || len > 32 || (mode != 0 && mode != 3) ||
        (cs != BSP_SPI_CS_ACC && cs != BSP_SPI_CS_GYRO) || hz == 0 || hz > BSP_SPI_HZ) {
        return ESP_ERR_INVALID_ARG;
    }
    spi_device_handle_t spi = mode == 3 ? s_spi : s_spi_mode0;
    if (!spi) return ESP_ERR_INVALID_STATE;
    gpio_io_config_t miso_cfg = {0};
    if (weak_pullup) {
        esp_err_t err = gpio_get_io_config(BSP_PIN_IMU_MISO, &miso_cfg);
        if (err != ESP_OK) return err;
        err = gpio_set_pull_mode(BSP_PIN_IMU_MISO, GPIO_PULLUP_ONLY);
        if (err != ESP_OK) return err;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .rxlength = len * 8,
        .override_freq_hz = hz,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    /* Inline buffers avoid DMA bounce buffers for short ID/register transfers. */
    if (len <= 4) {
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        memcpy(t.tx_data, tx, len);
    }
    bsp_spi_cs_high_all();
    /* acquire_bus configures CPOL before CS falls (BMI088 auto-selects mode). */
    esp_err_t err = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (err == ESP_OK) {
        esp_rom_delay_us(2);
        bsp_spi_cs_low(cs);
        esp_rom_delay_us(2);
        err = spi_device_polling_transmit(spi, &t);
        esp_rom_delay_us(2);
        bsp_spi_cs_high(cs);
        esp_rom_delay_us(2);
        spi_device_release_bus(spi);
        if (err == ESP_OK && len <= 4) memcpy(rx, t.rx_data, len);
    }
    if (weak_pullup) {
        esp_err_t restore_err = gpio_set_pull_mode(BSP_PIN_IMU_MISO, saved_pull(&miso_cfg));
        if (err == ESP_OK) err = restore_err;
    }
    return err;
}

esp_err_t bsp_spi_transfer(bsp_spi_cs_t cs, const uint8_t *tx, uint8_t *rx, size_t len)
{
    return transfer(cs, tx, rx, len, BSP_SPI_HZ, 3, false);
}

esp_err_t bsp_spi_probe_transfer(bsp_spi_cs_t cs, const uint8_t *tx, uint8_t *rx,
                                 size_t len, uint32_t hz, uint8_t mode)
{
    return transfer(cs, tx, rx, len, hz, mode, true);
}

esp_err_t bsp_spi_probe_miso(int *up, int *down, gpio_io_config_t *out_cfg)
{
    if (!up || !down) return ESP_ERR_INVALID_ARG;
    *up = *down = -1;
    bsp_spi_cs_high_all();
    gpio_io_config_t cfg;
    esp_err_t err = gpio_get_io_config(BSP_PIN_IMU_MISO, &cfg);
    if (err != ESP_OK) return err;
    if (out_cfg) *out_cfg = cfg;
    /* SPI's GPIO-matrix setup enables OE under peripheral control, including
     * on MISO for dual-I/O support. The raw OE register alone does not mean
     * that this four-wire master is driving MISO. Do not reject that setup. */
    if (!cfg.ie || (cfg.oe && !cfg.oe_ctrl_by_periph)) return ESP_ERR_INVALID_STATE;
    err = gpio_set_pull_mode(BSP_PIN_IMU_MISO, GPIO_PULLUP_ONLY);
    if (err == ESP_OK) {
        esp_rom_delay_us(100);
        *up = gpio_get_level(BSP_PIN_IMU_MISO);
        err = gpio_set_pull_mode(BSP_PIN_IMU_MISO, GPIO_PULLDOWN_ONLY);
        if (err == ESP_OK) {
            esp_rom_delay_us(100);
            *down = gpio_get_level(BSP_PIN_IMU_MISO);
        }
    }
    esp_err_t restore_err = gpio_set_pull_mode(BSP_PIN_IMU_MISO, saved_pull(&cfg));
    return err != ESP_OK ? err : restore_err;
}
