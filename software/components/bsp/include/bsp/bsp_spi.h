#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_SPI_CS_ACC = 0,
    BSP_SPI_CS_GYRO = 1,
} bsp_spi_cs_t;

esp_err_t bsp_spi_init(void);

void bsp_spi_cs_low(bsp_spi_cs_t which);
void bsp_spi_cs_high(bsp_spi_cs_t which);
void bsp_spi_cs_high_all(void);

/** Configure SPI while CS is high, then transfer with setup/hold time. */
esp_err_t bsp_spi_transfer(bsp_spi_cs_t cs, const uint8_t *tx, uint8_t *rx, size_t len);
/** Diagnostic transfer with weak MISO pull-up; restores original bias afterwards. */
esp_err_t bsp_spi_probe_transfer(bsp_spi_cs_t cs, const uint8_t *tx, uint8_t *rx,
                                 size_t len, uint32_t hz, uint8_t mode);
/** Both CS high: sample MISO with weak pull-up/down, then restore original bias. */
esp_err_t bsp_spi_probe_miso(int *up, int *down, gpio_io_config_t *out_cfg);

#ifdef __cplusplus
}
#endif
