#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float k;           /* 弹簧刚度 (N·m/rad 当量，映射到占空比) */
    float b;           /* 阻尼 (N·m·s/rad 当量) */
    float theta_des;   /* 目标角 rad */
    float duty_max;    /* 占空比上限 [0,1] */
    float theta_min;   /* 软限位 rad */
    float theta_max;
    int32_t enc_cpr;   /* 输出轴每转计数 */
    uint8_t enc_dir;   /* 0 正 / 1 反 */
    uint8_t ff_enable; /* 1=闭环使能 */
    uint32_t telem_hz; /* USB 订阅上报频率 */
    float heater_duty; /* IMU 加热占空比 0~1（YJL3400A + 100Ω） */
} svc_params_t;

esp_err_t svc_params_init(svc_params_t *out);
esp_err_t svc_params_save(const svc_params_t *in);

#ifdef __cplusplus
}
#endif
