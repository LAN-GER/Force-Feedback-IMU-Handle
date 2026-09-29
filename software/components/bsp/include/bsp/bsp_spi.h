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

/** 全双工移位 len 字节。片选由调用方包在整次事务外。 */
esp_err_t bsp_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len);

#ifdef __cplusplus
}
#endif
