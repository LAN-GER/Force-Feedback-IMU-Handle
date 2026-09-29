/**
 * @file indicate.c
 * @layer SVC
 * @brief WS2812 灯色策略
 */

#include "svc/indicate.h"

#define DIM 24

svc_rgb_t svc_indicate_from_mode(int mode)
{
    switch (mode) {
    case FF_IND_IMU_FAIL:
        return (svc_rgb_t){DIM, 0, 0};       /* 红 */
    case FF_IND_DISABLED:
        return (svc_rgb_t){0, 0, DIM};       /* 蓝：就绪但未使能 */
    case FF_IND_LIMIT:
        return (svc_rgb_t){DIM, DIM / 3, 0}; /* 橙：限位 */
    case FF_IND_FAULT:
        return (svc_rgb_t){DIM, 0, DIM};     /* 品红 */
    default:
        return (svc_rgb_t){0, DIM, 0};       /* 绿：闭环运行 */
    }
}
