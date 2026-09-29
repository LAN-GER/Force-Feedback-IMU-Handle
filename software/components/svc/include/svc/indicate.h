#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} svc_rgb_t;

enum {
    FF_IND_OK = 0,
    FF_IND_IMU_FAIL = 1,
    FF_IND_DISABLED = 2,
    FF_IND_LIMIT = 3,
    FF_IND_FAULT = 4,
};

svc_rgb_t svc_indicate_from_mode(int mode);

#ifdef __cplusplus
}
#endif
