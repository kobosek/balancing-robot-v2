#pragma once

#ifdef __cplusplus
using esp_err_t = int;
#define ESP_HOST_CONSTEXPR constexpr
#else
typedef int esp_err_t;
#define ESP_HOST_CONSTEXPR static const
#endif

ESP_HOST_CONSTEXPR int ESP_OK = 0;
ESP_HOST_CONSTEXPR int ESP_FAIL = -1;
ESP_HOST_CONSTEXPR int ESP_ERR_NO_MEM = 0x101;
ESP_HOST_CONSTEXPR int ESP_ERR_INVALID_ARG = 0x102;
ESP_HOST_CONSTEXPR int ESP_ERR_INVALID_STATE = 0x103;
ESP_HOST_CONSTEXPR int ESP_ERR_NOT_FOUND = 0x105;
ESP_HOST_CONSTEXPR int ESP_ERR_NOT_SUPPORTED = 0x106;

#ifdef __cplusplus
inline const char* esp_err_to_name(int error)
#else
static inline const char* esp_err_to_name(int error)
#endif
{
    (void)error;
    return "host-audit-error";
}
