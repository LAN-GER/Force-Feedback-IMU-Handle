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

static const char *TAG = "bsp_spi";
static spi_device_handle_t s_spi;

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
    ESP_LOGI(TAG, "SPI Mode3 %d Hz CS_ACC=%d CS_GYRO=%d",
             BSP_SPI_HZ, BSP_PIN_IMU_CS_ACC, BSP_PIN_IMU_CS_GYRO);
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

esp_err_t bsp_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!s_spi || !tx || !rx || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}
