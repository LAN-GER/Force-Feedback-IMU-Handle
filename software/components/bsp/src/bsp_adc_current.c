/**
 * @file bsp_adc_current.c
 * @layer BSP
 * @brief DRV8876 IPROPI → GPIO4 ADC1_CH4
 *
 * I = V_IPROPI / (A_IPROPI * R_IPROPI)，A=1000µA/A，R=2.2kΩ → I=V/2.2
 */

#include "bsp/bsp_adc_current.h"
#include "bsp/bsp_pin.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "bsp_i";

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;
static bool s_ready;
static float s_zero_v; /* 空载偏置校准 */

esp_err_t bsp_adc_current_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));

    adc_oneshot_chan_cfg_t ccfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_4, &ccfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        s_cali_ok = true;
    }
#endif

    /* 上电零点：电机未使能时采样均值 */
    float sum = 0.0f;
    for (int i = 0; i < 16; i++) {
        sum += bsp_adc_current_read_volts();
    }
    s_zero_v = sum / 16.0f;
    if (s_zero_v < 0.0f) {
        s_zero_v = 0.0f;
    }
    if (s_zero_v > 0.15f) {
        /* 异常偏大时不扣，避免负电流饱和 */
        ESP_LOGW(TAG, "IPROPI zero=%.3f V 偏大，跳过零点", s_zero_v);
        s_zero_v = 0.0f;
    }

    s_ready = true;
    ESP_LOGI(TAG, "IPROPI ADC1_CH4 R=%.0f ohm zero=%.3f V cali=%d",
             BSP_IPROPI_R_OHM, s_zero_v, (int)s_cali_ok);
    return ESP_OK;
}

float bsp_adc_current_read_volts(void)
{
    if (!s_adc) {
        return 0.0f;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_4, &raw) != ESP_OK) {
        return 0.0f;
    }
    if (s_cali_ok) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            return (float)mv * 0.001f;
        }
    }
    /* 粗略：12-bit、衰减约 0–2.5 V */
    return ((float)raw / 4095.0f) * 2.5f;
}

float bsp_adc_current_read_amps(void)
{
    if (!s_ready) {
        return 0.0f;
    }
    float v = bsp_adc_current_read_volts() - s_zero_v;
    if (v < 0.0f) {
        v = 0.0f;
    }
    float scale = BSP_IPROPI_A * BSP_IPROPI_R_OHM; /* 2.2 */
    if (scale < 1e-6f) {
        return 0.0f;
    }
    return v / scale;
}
