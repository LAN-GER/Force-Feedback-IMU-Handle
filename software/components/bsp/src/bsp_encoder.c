/**
 * @file bsp_encoder.c
 * @layer BSP
 * @brief GA12-N20 霍尔正交编码器（软件四倍频；C3 无硬件 PCNT）
 */

#include "bsp/bsp_encoder.h"
#include "bsp/bsp_pin.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "bsp_enc";

static volatile int32_t s_count;
static volatile uint8_t s_prev;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* 标准正交状态机：索引 = (prev<<2)|curr，值 = 步进 -1/0/+1 */
static const int8_t s_quad_table[16] = {
    0, +1, -1, 0,
    -1, 0, 0, +1,
    +1, 0, 0, -1,
    0, -1, +1, 0,
};

static inline uint8_t read_ab(void)
{
    uint8_t a = (uint8_t)gpio_get_level(BSP_PIN_ENC_A);
    uint8_t b = (uint8_t)gpio_get_level(BSP_PIN_ENC_B);
    return (uint8_t)((a << 1) | b);
}

static void IRAM_ATTR enc_isr(void *arg)
{
    (void)arg;
    uint8_t curr = read_ab();
    int8_t step = s_quad_table[(s_prev << 2) | curr];
    s_prev = curr;
    if (step != 0) {
        portENTER_CRITICAL_ISR(&s_mux);
        s_count += step;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
}

esp_err_t bsp_encoder_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BSP_PIN_ENC_A) | (1ULL << BSP_PIN_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    s_count = 0;
    s_prev = read_ab();

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(BSP_PIN_ENC_A, enc_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BSP_PIN_ENC_B, enc_isr, NULL));

    ESP_LOGI(TAG, "SW quad ENC_A=%d ENC_B=%d (C3 no PCNT)",
             BSP_PIN_ENC_A, BSP_PIN_ENC_B);
    return ESP_OK;
}

int32_t bsp_encoder_get_count(void)
{
    int32_t v;
    portENTER_CRITICAL(&s_mux);
    v = s_count;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void bsp_encoder_set_count(int32_t count)
{
    portENTER_CRITICAL(&s_mux);
    s_count = count;
    portEXIT_CRITICAL(&s_mux);
}

void bsp_encoder_clear(void)
{
    bsp_encoder_set_count(0);
}
