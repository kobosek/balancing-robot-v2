#pragma once

#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_event_base.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "config/WiFiConfig.hpp"

class WiFiManager {
public:
    // Init now takes the specific WiFiConfig struct
    esp_err_t init(const WiFiConfig& initialConfig);

private:
    static constexpr const char* TAG = "WiFiManager";

    static void eventHandler(void*, esp_event_base_t, int32_t, void*);
    // Connect now takes OUR WiFiConfig struct by const reference
    esp_err_t connect(const WiFiConfig& config); // <-- Use our struct type
    esp_err_t initNVS();

    // Static members for event handling
    static EventGroupHandle_t s_wifiEventGroup;
    static const int WIFI_CONNECTED_BIT = BIT0;
    static const int WIFI_FAIL_BIT = BIT1;
    static const int MAXIMUM_RETRY = 5;
    static int s_retryNum;
};
