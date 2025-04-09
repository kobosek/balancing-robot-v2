#include "ConfigurationService.hpp"     // Relative path within module's include dir
#include "ConfigUpdatedEvent.hpp"       // Found via INCLUDE_DIRS
#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "JsonConfigParser.hpp"         // Relative path within module's include dir
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for constructor/defaults)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for publish)
#include <string>
#include "esp_log.h"                    // Moved from header

ConfigurationService::ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey) :
    m_storageService(storage),
    m_configParser(parser),
    m_eventBus(bus),
    m_configKey(configKey)
{}

esp_err_t ConfigurationService::init() {
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGI(TAG, "Initializing Configuration Service. Loading from key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_storageService.loadData(m_configKey, rawData);

    if (ret == ESP_OK) {
        if (rawData.empty()) {
            ESP_LOGW(TAG, "Configuration file '%s' is empty. Using default values.", m_configKey.c_str());
            // Keep default m_configData
        } else {
            ret = m_configParser.deserialize(rawData, m_configData);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Configuration loaded and parsed successfully.");
            } else {
                ESP_LOGE(TAG, "Failed to parse loaded configuration data from '%s'. Using default values.", m_configKey.c_str());
                // Reset to defaults on parse error?
                m_configData = ConfigData(); // Re-assign defaults
            }
        }
    } else if (ret == ESP_ERR_NOT_FOUND) {
         ESP_LOGW(TAG, "Configuration file '%s' not found. Using default values and attempting to save them.", m_configKey.c_str());
         // Optionally save defaults here if desired
         m_configData = ConfigData(); // Ensure defaults are set
         esp_err_t save_ret = saveInternal(); // Attempt to save defaults
         if (save_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save default configuration to '%s'.", m_configKey.c_str());
            // Decide if this is a fatal error for init. Let's continue for now.
         }
         ret = ESP_OK; // Treat not found as OK for init, using defaults
    }
    else {
            ESP_LOGW(TAG, "Failed to load configuration from storage key '%s' (%s). Using default values.", m_configKey.c_str(), esp_err_to_name(ret));
             // Reset to defaults on other load errors?
             m_configData = ConfigData();
             // Return the underlying storage error? Or mask it? Let's mask for now.
             ret = ESP_OK;
    }
    return ret;
}

esp_err_t ConfigurationService::save() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return saveInternal();
}

esp_err_t ConfigurationService::updateConfigFromJson(const std::string& json) {
    ConfigData tempConfig = m_configData; // Start with current config as base for deserialize
    esp_err_t ret = ESP_FAIL;

    ret = m_configParser.deserialize(json, tempConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deserialize JSON for update.");
        return ret;
    }

    // Lock only when updating the internal state and saving
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_configData = tempConfig; // Update internal state
        ret = saveInternal();      // Attempt to save
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save configuration after update.");
        } else {
             ESP_LOGI(TAG, "Configuration updated and saved.");
        }
    } // Mutex released

    // Publish event regardless of save outcome (config in memory changed)
    ESP_LOGI(TAG, "Publishing configuration update event.");
    ConfigUpdatedEvent event;
    m_eventBus.publish(event);

    return ret; // Return the result of the save operation
}

esp_err_t ConfigurationService::getJsonString(std::string& jsonOutput) const {
    std::lock_guard<std::mutex> lock(m_mutex); // Lock for reading m_configData
    // Use the member parser instance to serialize the current data
    esp_err_t ret = m_configParser.serialize(m_configData, jsonOutput);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize current configuration.");
    }
    return ret;
}


esp_err_t ConfigurationService::saveInternal() {
    // Assumes mutex is held
    ESP_LOGD(TAG, "Saving configuration to storage key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_configParser.serialize(m_configData, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize configuration for saving.");
        return ret;
    }
    ret = m_storageService.saveData(m_configKey, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration data to storage (%s).", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Configuration saved successfully.");
    }
    return ret;
}