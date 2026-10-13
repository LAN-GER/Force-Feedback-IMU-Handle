#pragma once

#include <stddef.h>
#include "svc/imu_ff.h"

/* Returns payload length, or 0 if capacity is insufficient. No partial frame. */
size_t app_telem_json(char *buf, size_t capacity, const svc_telemetry_t *t);
