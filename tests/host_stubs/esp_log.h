#pragma once
static inline void host_log(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; }
#define ESP_LOGI host_log
#define ESP_LOGW host_log
