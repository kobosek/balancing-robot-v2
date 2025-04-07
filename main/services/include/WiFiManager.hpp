// main/include/WiFiManager.hpp
#pragma once

#include "nvs_flash.h"
#include "esp_log.h"                    // Keep for TAG
#include "freertos/FreeRTOS.h"
#include "esp_event_base.h"             // For esp_event_base_t
#include "freertos/event_groups.h"      // For EventGroupHandle_t
#include "esp_wifi.h"                   // Moved from header
#include "esp_event.h"                  // Moved from header

// Forward declare services/data it needs
class ConfigurationService; // Defined in config/include
struct WiFiConfig;          // Defined in config/include

class WiFiManager {
public:
    // Init now takes ConfigurationService to get necessary params
    esp_err_t init(ConfigurationService& configService);

    // Add method to handle config updates if needed later (called by event handler)
    // void handleConfigUpdate(const ConfigData& newConfig);

private:
    static constexpr const char* TAG = "WiFiManager";

    static void eventHandler(void*, esp_event_base_t, int32_t, void*);
    // Connect now takes **OUR** WiFiConfig struct by const reference
    esp_err_t connect(const WiFiConfig& config); // <-- Use our struct type
    esp_err_t initNVS();

    // Static members for event handling
    static EventGroupHandle_t s_wifiEventGroup;
    static const int WIFI_CONNECTED_BIT = BIT0;
    static const int WIFI_FAIL_BIT = BIT1;
    static const int MAXIMUM_RETRY = 5;
    static int s_retryNum;
};