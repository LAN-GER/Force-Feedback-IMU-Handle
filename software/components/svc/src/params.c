/**
 * @file params.c
 * @layer SVC
 * @brief NVS：控制模式 / PID / 力矩 / DOB
 */

#include "svc/params.h"
#include "bsp/bsp_encoder.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "params";
static const char *NS = "ffh";

#define ENC_CAL_VERSION 3
#define LEGACY_ENC_CPR 600 /* 3 PPR * 4 * the old 50:1 gearbox default. */
#define UNCALIBRATED_ENC_CPR 3576 /* Previous 3 PPR * 4 * 298 assumption. */
#define ROUGH_MEASURED_ENC_CPR 8292 /* Version 2 default from a rough manual turn. */

static void apply_defaults(svc_params_t *p)
{
    p->k = 0.8f;
    p->b = 0.05f;
    p->theta_des = 0.0f;
    p->duty_max = 0.6f;
    p->theta_min = -1.5f;
    p->theta_max = 1.5f;
    p->enc_cpr = BSP_ENC_OUTPUT_CPR;
    p->enc_dir = 0;
    p->ctrl_mode = CTRL_MODE_OFF;
    p->ff_enable = 0;
    p->telem_hz = 50;
    p->heater_duty = 0.0f;
    p->imu_kf_en = 1;

    p->pid_kp = 2.0f;
    p->pid_ki = 0.5f;
    p->pid_kd = 0.05f;

    p->tau_des = 0.0f;
    p->tq_kp = 1.5f;
    p->tq_ki = 8.0f;
    p->i_max = 1.0f;
    p->kt = 0.02f;

    p->dob_vm = 5.0f;
    p->dob_r = 5.0f;
    p->dob_ke = 0.02f;
    p->dob_kt = 0.02f;
    p->dob_j = 5.0e-5f;
    p->dob_b_visc = 1.0e-4f;
    p->dob_tau_c = 0.0f;
    p->dob_fc_hz = 30.0f;
    p->dob_enable = 1;
}

static float get_f_scaled(nvs_handle_t h, const char *key, float scale, float def)
{
    int32_t i32;
    if (nvs_get_i32(h, key, &i32) == ESP_OK) {
        return (float)i32 / scale;
    }
    return def;
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
        ESP_LOGI(TAG, "无存档，用默认参数");
        return ESP_OK;
    }

    int32_t i32;
    uint8_t u8;
    uint32_t u32;
    if (nvs_get_u8(h, "imu_kf_en", &u8) == ESP_OK && u8 <= 1) {
        out->imu_kf_en = u8;
    }
    uint8_t enc_cal_version = 0;
    bool migrate_enc_cpr = false;
    (void)nvs_get_u8(h, "enc_ver", &enc_cal_version);
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
        if ((enc_cal_version < 1 && i32 == LEGACY_ENC_CPR) ||
            (enc_cal_version < 2 && i32 == UNCALIBRATED_ENC_CPR) ||
            (enc_cal_version == 2 && i32 == ROUGH_MEASURED_ENC_CPR)) {
            migrate_enc_cpr = true;
            ESP_LOGI(TAG, "encoder CPR migration: %ld -> %d (nominal motor_ppr=%d, gear=%d, quadrature=%d)",
                     (long)i32, BSP_ENC_OUTPUT_CPR, BSP_ENC_MOTOR_PPR, BSP_MOTOR_GEAR_RATIO,
                     BSP_ENC_QUADRATURE);
        } else {
            out->enc_cpr = i32; /* Preserve custom calibration and versioned saves. */
        }
    }
    if (nvs_get_u8(h, "edir", &u8) == ESP_OK) {
        out->enc_dir = u8 ? 1 : 0;
    }
    if (nvs_get_u8(h, "cmode", &u8) == ESP_OK && u8 <= CTRL_MODE_TORQUE) {
        out->ctrl_mode = u8;
    } else if (nvs_get_u8(h, "ffen", &u8) == ESP_OK && u8) {
        out->ctrl_mode = CTRL_MODE_IMPEDANCE;
    }
    out->ff_enable = (out->ctrl_mode == CTRL_MODE_IMPEDANCE) ? 1 : 0;
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

    out->pid_kp = get_f_scaled(h, "pkp_x1000", 1000.0f, out->pid_kp);
    out->pid_ki = get_f_scaled(h, "pki_x1000", 1000.0f, out->pid_ki);
    out->pid_kd = get_f_scaled(h, "pkd_x1000", 1000.0f, out->pid_kd);

    out->tau_des = get_f_scaled(h, "tdes_x1e6", 1.0e6f, out->tau_des);
    out->tq_kp = get_f_scaled(h, "tkp_x1000", 1000.0f, out->tq_kp);
    out->tq_ki = get_f_scaled(h, "tki_x1000", 1000.0f, out->tq_ki);
    out->i_max = get_f_scaled(h, "imax_x1000", 1000.0f, out->i_max);
    out->kt = get_f_scaled(h, "kt_x1e6", 1.0e6f, out->kt);

    out->dob_vm = get_f_scaled(h, "dvm_x100", 100.0f, out->dob_vm);
    out->dob_r = get_f_scaled(h, "dr_x100", 100.0f, out->dob_r);
    out->dob_ke = get_f_scaled(h, "dke_x1e6", 1.0e6f, out->dob_ke);
    out->dob_kt = get_f_scaled(h, "dkt_x1e6", 1.0e6f, out->dob_kt);
    out->dob_j = get_f_scaled(h, "dj_x1e9", 1.0e9f, out->dob_j);
    out->dob_b_visc = get_f_scaled(h, "dbv_x1e9", 1.0e9f, out->dob_b_visc);
    out->dob_tau_c = get_f_scaled(h, "dtc_x1e6", 1.0e6f, out->dob_tau_c);
    out->dob_fc_hz = get_f_scaled(h, "dfc_x10", 10.0f, out->dob_fc_hz);
    if (nvs_get_u8(h, "doben", &u8) == ESP_OK) {
        out->dob_enable = u8 ? 1 : 0;
    }
    nvs_close(h);

    if (migrate_enc_cpr) {
        /* Update only encoder calibration; preserve all other saved settings.
         * Runtime still uses the corrected default if this persistence fails. */
        err = nvs_open(NS, NVS_READWRITE, &h);
        if (err == ESP_OK) {
            err = nvs_set_i32(h, "cpr", out->enc_cpr);
            if (err == ESP_OK) err = nvs_set_u8(h, "enc_ver", ENC_CAL_VERSION);
            if (err == ESP_OK) err = nvs_commit(h);
            nvs_close(h);
        }
        if (err != ESP_OK) ESP_LOGW(TAG, "encoder calibration persistence failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "mode=%u cpr=%ld pid=%.2f/%.2f/%.3f kt=%.4f",
             out->ctrl_mode, (long)out->enc_cpr, out->pid_kp, out->pid_ki, out->pid_kd, out->kt);
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
    ESP_ERROR_CHECK(nvs_set_u8(h, "enc_ver", ENC_CAL_VERSION));
    ESP_ERROR_CHECK(nvs_set_u8(h, "edir", in->enc_dir));
    ESP_ERROR_CHECK(nvs_set_u8(h, "cmode", in->ctrl_mode));
    ESP_ERROR_CHECK(nvs_set_u8(h, "ffen", (in->ctrl_mode == CTRL_MODE_IMPEDANCE) ? 1 : 0));
    ESP_ERROR_CHECK(nvs_set_u32(h, "thz", in->telem_hz));
    ESP_ERROR_CHECK(nvs_set_u8(h, "imu_kf_en", in->imu_kf_en));
    ESP_ERROR_CHECK(nvs_set_i32(h, "heat_x1000", (int32_t)(in->heater_duty * 1000.0f)));

    ESP_ERROR_CHECK(nvs_set_i32(h, "pkp_x1000", (int32_t)(in->pid_kp * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "pki_x1000", (int32_t)(in->pid_ki * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "pkd_x1000", (int32_t)(in->pid_kd * 1000.0f)));

    ESP_ERROR_CHECK(nvs_set_i32(h, "tdes_x1e6", (int32_t)(in->tau_des * 1.0e6f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "tkp_x1000", (int32_t)(in->tq_kp * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "tki_x1000", (int32_t)(in->tq_ki * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "imax_x1000", (int32_t)(in->i_max * 1000.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "kt_x1e6", (int32_t)(in->kt * 1.0e6f)));

    ESP_ERROR_CHECK(nvs_set_i32(h, "dvm_x100", (int32_t)(in->dob_vm * 100.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dr_x100", (int32_t)(in->dob_r * 100.0f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dke_x1e6", (int32_t)(in->dob_ke * 1.0e6f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dkt_x1e6", (int32_t)(in->dob_kt * 1.0e6f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dj_x1e9", (int32_t)(in->dob_j * 1.0e9f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dbv_x1e9", (int32_t)(in->dob_b_visc * 1.0e9f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dtc_x1e6", (int32_t)(in->dob_tau_c * 1.0e6f)));
    ESP_ERROR_CHECK(nvs_set_i32(h, "dfc_x10", (int32_t)(in->dob_fc_hz * 10.0f)));
    ESP_ERROR_CHECK(nvs_set_u8(h, "doben", in->dob_enable));

    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI(TAG, "已写入 NVS");
    return ESP_OK;
}
