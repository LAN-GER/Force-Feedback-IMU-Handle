/**
 * @file params.c
 * @layer SVC
 * @brief NVS：阻抗参数、编码器 CPR、使能标志
 */

#include "svc/params.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "params";
static const char *NS = "ffh";

static void apply_defaults(svc_params_t *p)
{
    p->k = 0.8f;
    p->b = 0.05f;
    p->theta_des = 0.0f;
    p->duty_max = 0.6f;
    p->theta_min = -1.5f;
    p->theta_max = 1.5f;
    p->enc_cpr = 600; /* 3PPR * 4 * 50:1 */
    p->enc_dir = 0;
    p->ff_enable = 0;
    p->telem_hz = 50;
    p->heater_duty = 0.0f;
}

esp_err_t svc_params_init(svc_params_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    apply_defaults(out);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t h;
    err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "无存档，用默认阻抗参数");
        return ESP_OK;
    }

    int32_t i32;
    uint8_t u8;
    uint32_t u32;
    if (nvs_get_i32(h, "k_x1000", &i32) == ESP_OK) {
        out->k = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "b_x1000", &i32) == ESP_OK) {
        out->b = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "td_x1000", &i32) == ESP_OK) {
        out->theta_des = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "dmax_x1000", &i32) == ESP_OK) {
        out->duty_max = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "tmin_x1000", &i32) == ESP_OK) {
        out->theta_min = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "tmax_x1000", &i32) == ESP_OK) {
        out->theta_max = (float)i32 / 1000.0f;
    }
    if (nvs_get_i32(h, "cpr", &i32) == ESP_OK && i32 > 0) {
        out->enc_cpr = i32;
    }
    if (nvs_get_u8(h, "edir", &u8) == ESP_OK) {
        out->enc_dir = u8 ? 1 : 0;
    }
    if (nvs_get_u8(h, "ffen", &u8) == ESP_OK) {
        out->ff_enable = u8 ? 1 : 0;
    }
    if (nvs_get_u32(h, "thz", &u32) == ESP_OK && u32 >= 1 && u32 <= 200) {
        out->telem_hz = u32;
    }
    if (nvs_get_i32(h, "heat_x1000", &i32) == ESP_OK) {
        float d = (float)i32 / 1000.0f;
        if (d < 0.0f) {
            d = 0.0f;
        }
        if (d > 1.0f) {
            d = 1.0f;
        }
        out->heater_duty = d;
    }
    nvs_close(h);

    ESP_LOGI(TAG, "k=%.3f b=%.3f cpr=%ld ff=%u",
             out->k, out->b, (long)out->enc_cpr, out->ff_enable);
    return ESP_OK;
}

esp_err_t svc_params_save(const svc_params_t *in)
{
    if (!in) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NS, NVS_READWRITE, &h));
    ESP_ERROR_CHECK(nvs_set_i32(h, "k_x1000", (int32_t)(in->k * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "b_x1000", (int32_t)(in->b * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "td_x1000", (int32_t)(in->theta_des * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dmax_x1000", (int32_t)(in->duty_max * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "tmin_x1000", (int32_t)(in->theta_min * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "tmax_x1000", (int32_t)(in->theta_max * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "cpr", in->enc_cpr));
    ESP_ERROR_CHECK(nvs_set_u8(h, "edir", in->enc_dir));
    ESP_ERROR_CHECK(nvs_set_u8(h, "ffen", in->ff_enable));
    ESP_ERROR_CHECK(nvs_set_u32(h, "thz", in->telem_hz));
    ESP_ERROR_CHECK(nvs_set_i32(h, "heat_x1000", (int32_t)(in->heater_duty * 1000.0f)));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI(TAG, "已写入 NVS");
    return ESP_OK;
}
