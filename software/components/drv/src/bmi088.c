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
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>

static const char *TAG = "bmi088";
static bool s_ready;
static uint8_t s_acc_id;
static uint8_t s_gyro_id;
static esp_err_t s_acc_spi_err = ESP_ERR_INVALID_STATE;
static esp_err_t s_gyro_spi_err = ESP_ERR_INVALID_STATE;

#define ACC_CHIP_ID_REG   0x00
#define ACC_STATUS_REG    0x03
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

static void init_delay_ms(uint32_t ms)
{
    /* One extra tick covers truncation and the next tick arriving immediately. */
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

static esp_err_t write_reg(bsp_spi_cs_t cs, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2] = {0};
    esp_err_t err = bsp_spi_transfer(cs, tx, rx, sizeof(tx));
    /* Normal-mode register synchronization and CS high time. */
    esp_rom_delay_us(2);
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
    esp_err_t err = bsp_spi_transfer(BSP_SPI_CS_ACC, tx, rx, total);
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
    esp_err_t err = bsp_spi_transfer(BSP_SPI_CS_GYRO, tx, rx, total);
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
    esp_err_t err = read_regs_acc(ACC_CHIP_ID_REG, &id, 1);
    s_acc_spi_err = err;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Acc ID SPI transfer failed: %s", esp_err_to_name(err));
        return 0;
    }
    return id;
}

uint8_t drv_bmi088_read_gyro_id(void)
{
    uint8_t id = 0;
    esp_err_t err = read_regs_gyro(GYRO_CHIP_ID_REG, &id, 1);
    s_gyro_spi_err = err;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyro ID SPI transfer failed: %s", esp_err_to_name(err));
        return 0;
    }
    return id;
}

esp_err_t drv_bmi088_init(void)
{
    s_ready = false;
    s_acc_id = 0;
    s_gyro_id = 0;
    s_acc_spi_err = ESP_ERR_INVALID_STATE;
    s_gyro_spi_err = ESP_ERR_INVALID_STATE;
    /* Acc 默认 I2C：CSB1 上升沿切 SPI；上电后先保证双 CS 高 */
    bsp_spi_cs_high_all();
    init_delay_ms(30); /* Gyroscope power-on startup. */

    /* 一次 dummy 读强制进入 SPI */
    (void)drv_bmi088_read_acc_id();
    esp_rom_delay_us(1000);

    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_ACC, ACC_SOFTRESET, 0xB6), TAG, "acc reset");
    init_delay_ms(50);
    bsp_spi_cs_high_all();
    (void)drv_bmi088_read_acc_id();
    esp_rom_delay_us(1000);

    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_ACC, ACC_PWR_CONF, 0x00), TAG, "acc active");
    init_delay_ms(2); /* Suspend-mode write synchronization must not round to 0. */
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_ACC, ACC_PWR_CTRL, 0x04), TAG, "acc enable");
    init_delay_ms(50);
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_ACC, ACC_CONF, 0xA8), TAG, "acc ODR"); /* 100 Hz */
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_ACC, ACC_RANGE, 0x03), TAG, "acc range"); /* ±24 g */

    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_GYRO, GYRO_SOFTRESET, 0xB6), TAG, "gyro reset");
    init_delay_ms(50);
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_GYRO, GYRO_LPM1, 0x00), TAG, "gyro normal");
    init_delay_ms(30);
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_GYRO, GYRO_RANGE, 0x00), TAG, "gyro range"); /* ±2000 dps */
    ESP_RETURN_ON_ERROR(write_reg(BSP_SPI_CS_GYRO, GYRO_BANDWIDTH, 0x02), TAG, "gyro bandwidth"); /* 1000 Hz / 116 Hz */

    uint8_t aid = 0;
    uint8_t gid = 0;
    esp_err_t acc_err = read_regs_acc(ACC_CHIP_ID_REG, &aid, 1);
    esp_err_t gyro_err = read_regs_gyro(GYRO_CHIP_ID_REG, &gid, 1);
    s_acc_spi_err = acc_err;
    s_gyro_spi_err = gyro_err;
    s_acc_id = aid;
    s_gyro_id = gid;
    ESP_LOGI(TAG, "acc_id=0x%02X gyro_id=0x%02X", aid, gid);
    ESP_LOGI(TAG, "ID SPI status: acc=%s gyro=%s", esp_err_to_name(acc_err), esp_err_to_name(gyro_err));
    if (acc_err != ESP_OK || gyro_err != ESP_OK) {
        return acc_err != ESP_OK ? acc_err : gyro_err;
    }
    if (aid != ACC_CHIP_ID_VAL || gid != GYRO_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "Chip ID 不匹配（期望 Acc 0x1E / Gyro 0x0F）");
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_ready = true;
    return ESP_OK;
}

void drv_bmi088_get_diagnostics(bmi088_diagnostics_t *out)
{
    if (out) {
        *out = (bmi088_diagnostics_t){
            .ready = s_ready,
            .acc_id = s_acc_id,
            .gyro_id = s_gyro_id,
            .acc_spi_err = s_acc_spi_err,
            .gyro_spi_err = s_gyro_spi_err,
        };
    }
}

esp_err_t drv_bmi088_probe(bmi088_probe_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    /* The sampler skips SPI when !s_ready; avoid changing an active sensor bus. */
    if (s_ready) return ESP_ERR_INVALID_STATE;
    *out = (bmi088_probe_t){0};
    gpio_io_config_t miso_cfg = {0};
    out->miso_err = bsp_spi_probe_miso(&out->miso_up, &out->miso_down, &miso_cfg);
    out->miso_input = miso_cfg.ie;
    out->miso_oe = miso_cfg.oe;
    out->miso_periph = miso_cfg.oe_ctrl_by_periph;
    const uint32_t rates[] = {8000000, 1000000, 250000};
    const uint8_t modes[] = {0, 3};
    const uint8_t tx[] = {0x80, 0, 0}; /* Read-only CHIP_ID register. */
    size_t index = 0;
    for (size_t m = 0; m < 2; ++m) {
        for (size_t r = 0; r < 3; ++r) {
            bmi088_probe_case_t *c = &out->cases[index++];
            c->mode = modes[m];
            c->hz = rates[r];
            uint8_t dummy[3] = {0};
            c->acc_err = bsp_spi_probe_transfer(BSP_SPI_CS_ACC, tx, dummy, 3, c->hz, c->mode);
            esp_rom_delay_us(1000); /* Complete the SPI-activation dummy read. */
            if (c->acc_err == ESP_OK) {
                c->acc_err = bsp_spi_probe_transfer(BSP_SPI_CS_ACC, tx, c->acc_raw, 3, c->hz, c->mode);
            }
            c->gyro_err = bsp_spi_probe_transfer(BSP_SPI_CS_GYRO, tx, c->gyro_raw, 2, c->hz, c->mode);
        }
    }
    /* Last case uses Mode3; subsequent regular transfers restore the normal rate. */
    bsp_spi_cs_high_all();
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
    *out = (bmi088_sample_t){
        .acc_id = s_acc_id,
        .gyro_id = s_gyro_id,
    };
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t abuf[6] = {0};
    uint8_t gbuf[6] = {0};
    uint8_t status = 0;
    esp_err_t es = read_regs_acc(ACC_STATUS_REG, &status, 1);
    esp_err_t ea = read_regs_acc(ACC_DATA_START, abuf, 6);
    esp_err_t eg = read_regs_gyro(GYRO_RATE_X_LSB, gbuf, 6);
    if (es != ESP_OK || ea != ESP_OK || eg != ESP_OK) {
        out->ok = false;
        return es != ESP_OK ? es : ((ea != ESP_OK) ? ea : eg);
    }

    out->ax_g = (float)le16(&abuf[0]) / ACC_LSB_PER_G;
    out->ay_g = (float)le16(&abuf[2]) / ACC_LSB_PER_G;
    out->az_g = (float)le16(&abuf[4]) / ACC_LSB_PER_G;
    out->gx_dps = (float)le16(&gbuf[0]) / GYRO_LSB_PER_DPS;
    out->gy_dps = (float)le16(&gbuf[2]) / GYRO_LSB_PER_DPS;
    out->gz_dps = (float)le16(&gbuf[4]) / GYRO_LSB_PER_DPS;
    out->ok = true;
    out->acc_new = (status & 0x80) != 0;
    return ESP_OK;
}
