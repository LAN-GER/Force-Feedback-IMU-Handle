/**
 * @file bmi088.c
 * @layer DRV
 * @brief BMI088 SPI：Acc/Gyro 独立片选；Acc 读丢弃 dummy 字节
 *
 * Chip ID：Acc 0x1E，Gyro 0x0F
 */

#include "drv/bmi088.h"
#include "bsp/bsp_spi.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>

static const char *TAG = "bmi088";

#define ACC_CHIP_ID_REG   0x00
#define ACC_DATA_START    0x12
#define ACC_CONF          0x40
#define ACC_RANGE         0x41
#define ACC_PWR_CONF      0x7C
#define ACC_PWR_CTRL      0x7D
#define ACC_SOFTRESET     0x7E

#define GYRO_CHIP_ID_REG  0x00
#define GYRO_RATE_X_LSB   0x02
#define GYRO_RANGE        0x0F
#define GYRO_BANDWIDTH    0x10
#define GYRO_LPM1         0x11
#define GYRO_SOFTRESET    0x14

#define ACC_CHIP_ID_VAL   0x1E
#define GYRO_CHIP_ID_VAL  0x0F

/* ±24 g → 1365 LSB/g；±2000 dps → 16.384 LSB/(°/s) */
#define ACC_LSB_PER_G     1365.0f
#define GYRO_LSB_PER_DPS  16.384f

static esp_err_t write_reg(bsp_spi_cs_t cs, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2] = {0};
    bsp_spi_cs_low(cs);
    esp_err_t err = bsp_spi_xfer(tx, rx, sizeof(tx));
    bsp_spi_cs_high(cs);
    return err;
}

static esp_err_t read_regs_acc(uint8_t reg, uint8_t *buf, size_t len)
{
    /* Acc SPI 读：地址 + dummy + N data */
    size_t total = 1 + 1 + len;
    uint8_t tx[16];
    uint8_t rx[16];
    if (total > sizeof(tx) || !buf) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = (uint8_t)(reg | 0x80);
    for (size_t i = 1; i < total; i++) {
        tx[i] = 0x00;
    }
    bsp_spi_cs_low(BSP_SPI_CS_ACC);
    esp_err_t err = bsp_spi_xfer(tx, rx, total);
    bsp_spi_cs_high(BSP_SPI_CS_ACC);
    if (err != ESP_OK) {
        return err;
    }
    /* rx[0]=junk during addr, rx[1]=dummy, rx[2..]=data */
    for (size_t i = 0; i < len; i++) {
        buf[i] = rx[2 + i];
    }
    return ESP_OK;
}

static esp_err_t read_regs_gyro(uint8_t reg, uint8_t *buf, size_t len)
{
    size_t total = 1 + len;
    uint8_t tx[16];
    uint8_t rx[16];
    if (total > sizeof(tx) || !buf) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = (uint8_t)(reg | 0x80);
    for (size_t i = 1; i < total; i++) {
        tx[i] = 0x00;
    }
    bsp_spi_cs_low(BSP_SPI_CS_GYRO);
    esp_err_t err = bsp_spi_xfer(tx, rx, total);
    bsp_spi_cs_high(BSP_SPI_CS_GYRO);
    if (err != ESP_OK) {
        return err;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = rx[1 + i];
    }
    return ESP_OK;
}

uint8_t drv_bmi088_read_acc_id(void)
{
    uint8_t id = 0;
    if (read_regs_acc(ACC_CHIP_ID_REG, &id, 1) != ESP_OK) {
        return 0;
    }
    return id;
}

uint8_t drv_bmi088_read_gyro_id(void)
{
    uint8_t id = 0;
    if (read_regs_gyro(GYRO_CHIP_ID_REG, &id, 1) != ESP_OK) {
        return 0;
    }
    return id;
}

esp_err_t drv_bmi088_init(void)
{
    /* Acc 默认 I2C：CSB1 上升沿切 SPI；上电后先保证双 CS 高 */
    bsp_spi_cs_high_all();
    esp_rom_delay_us(2000);

    /* 一次 dummy 读强制进入 SPI */
    (void)drv_bmi088_read_acc_id();
    esp_rom_delay_us(200);

    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_ACC, ACC_SOFTRESET, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_spi_cs_high_all();
    (void)drv_bmi088_read_acc_id();

    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_ACC, ACC_PWR_CONF, 0x00)); /* active */
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_ACC, ACC_PWR_CTRL, 0x04)); /* enable accel */
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_ACC, ACC_CONF, 0xA8));     /* ODR 100 Hz, normal */
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_ACC, ACC_RANGE, 0x03));    /* ±24 g */

    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_GYRO, GYRO_SOFTRESET, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_GYRO, GYRO_LPM1, 0x00));   /* normal */
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_GYRO, GYRO_RANGE, 0x00));  /* ±2000 dps */
    ESP_ERROR_CHECK(write_reg(BSP_SPI_CS_GYRO, GYRO_BANDWIDTH, 0x02)); /* 1000 Hz / 116 Hz */

    uint8_t aid = drv_bmi088_read_acc_id();
    uint8_t gid = drv_bmi088_read_gyro_id();
    ESP_LOGI(TAG, "acc_id=0x%02X gyro_id=0x%02X", aid, gid);
    if (aid != ACC_CHIP_ID_VAL || gid != GYRO_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "Chip ID 不匹配（期望 Acc 0x1E / Gyro 0x0F）");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

esp_err_t drv_bmi088_read(bmi088_sample_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t abuf[6] = {0};
    uint8_t gbuf[6] = {0};
    esp_err_t ea = read_regs_acc(ACC_DATA_START, abuf, 6);
    esp_err_t eg = read_regs_gyro(GYRO_RATE_X_LSB, gbuf, 6);
    if (ea != ESP_OK || eg != ESP_OK) {
        out->ok = false;
        return (ea != ESP_OK) ? ea : eg;
    }

    out->ax_g = (float)le16(&abuf[0]) / ACC_LSB_PER_G;
    out->ay_g = (float)le16(&abuf[2]) / ACC_LSB_PER_G;
    out->az_g = (float)le16(&abuf[4]) / ACC_LSB_PER_G;
    out->gx_dps = (float)le16(&gbuf[0]) / GYRO_LSB_PER_DPS;
    out->gy_dps = (float)le16(&gbuf[2]) / GYRO_LSB_PER_DPS;
    out->gz_dps = (float)le16(&gbuf[4]) / GYRO_LSB_PER_DPS;
    out->ok = true;
    return ESP_OK;
}
