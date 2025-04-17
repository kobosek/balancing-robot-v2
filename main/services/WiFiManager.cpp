
#include "WiFiManager.hpp"
#include "ConfigData.hpp"

#include "nvs_flash.h"
#include "esp_netif.h"

#include <cstring>
#include "esp_log.h"
#include <cstring> // For strncpy

// Static member initialization
EventGroupHandle_t WiFiManager::s_wifiEventGroup = nullptr;
int WiFiManager::s_retryNum = 0;

// NVS Init remains the same
esp_err_t WiFiManager::initNVS() {
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }
    return ret;
}

// Init now takes WiFiConfig struct
esp_err_t WiFiManager::init(const WiFiConfig& initialConfig) {
    ESP_LOGI(TAG, "Initializing WiFiManager");

    esp_err_t ret = initNVS();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        return ret;
    }

    s_wifiEventGroup = xEventGroupCreate();
    if (!s_wifiEventGroup) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NETIF: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_wifiEventGroup); // Cleanup event group on failure
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_wifiEventGroup);
        // Consider esp_netif_deinit() here too?
        return ret;
    }

    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        ESP_LOGE(TAG, "Failed to create STA netif");
        vEventGroupDelete(s_wifiEventGroup);
        // Cleanup event loop?
        return ESP_FAIL;
    }


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_wifiEventGroup);
        // Cleanup netif, event loop?
        return ret;
    }

    // Register event handlers
    esp_event_handler_instance_t instanceAnyId;
    esp_event_handler_instance_t instanceGotIp;
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, this, &instanceAnyId);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed register WIFI_EVENT handler: %s", esp_err_to_name(ret)); return ret; }
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, this, &instanceGotIp);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed register IP_EVENT handler: %s", esp_err_to_name(ret)); return ret; }

    ESP_LOGI(TAG, "WiFiManager base initialized successfully. Connecting...");

    // Use the initial config passed to init()
    return connect(initialConfig); // Call connect with OUR struct
}

// Connect now takes OUR WiFiConfig struct
esp_err_t WiFiManager::connect(const WiFiConfig& config) {
    if (config.ssid.empty()) {
        ESP_LOGE(TAG, "WiFi SSID is empty. Cannot connect.");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to WiFi network: %s", config.ssid.c_str());

    // Use IDF's wifi_config_t structure for the API call
    wifi_config_t idf_wifi_config = {}; // Initialize to zero

    // Copy SSID from our config struct to IDF's config struct
    strncpy((char*)idf_wifi_config.sta.ssid, config.ssid.c_str(), sizeof(idf_wifi_config.sta.ssid) - 1);
    idf_wifi_config.sta.ssid[sizeof(idf_wifi_config.sta.ssid) - 1] = '\0'; // Ensure null termination

    // Copy Password
    strncpy((char*)idf_wifi_config.sta.password, config.password.c_str(), sizeof(idf_wifi_config.sta.password) - 1);
    idf_wifi_config.sta.password[sizeof(idf_wifi_config.sta.password) - 1] = '\0'; // Ensure null termination

    // Assuming WPA2-PSK, make configurable later if needed
    idf_wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode STA: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &idf_wifi_config); // Pass IDF's struct
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi stack started. Waiting for connection result...");

    // Wait for connection event (same logic)
    EventBits_t bits = xEventGroupWaitBits(s_wifiEventGroup,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to SSID: %s", config.ssid.c_str());
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", config.ssid.c_str());
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT while waiting for connection.");
        return ESP_ERR_INVALID_STATE;
    }
}

// Event handler remains the same
void WiFiManager::eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, invoking esp_wifi_connect()");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retryNum < MAXIMUM_RETRY) {
            s_retryNum++;
            ESP_LOGI(TAG, "WiFi disconnected, retrying connection (%d/%d)...", s_retryNum, MAXIMUM_RETRY);
            // Use a delay before retrying?
            // vTaskDelay(pdMS_TO_TICKS(5000));
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi disconnected, failed to connect after %d retries.", MAXIMUM_RETRY);
            if (s_wifiEventGroup) xEventGroupSetBits(s_wifiEventGroup, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retryNum = 0; // Reset retry counter on success
        if (s_wifiEventGroup) xEventGroupSetBits(s_wifiEventGroup, WIFI_CONNECTED_BIT);
    }
}