#pragma once
/* Only for native tests; firmware uses ESP-IDF's real header. */
#include <stdlib.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERROR_CHECK(call) do { if ((call) != ESP_OK) abort(); } while (0)
const char *esp_err_to_name(esp_err_t err);
