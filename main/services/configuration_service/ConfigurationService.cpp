#include "ConfigurationService.hpp"     // Relative path within module's include dir
#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "JsonConfigParser.hpp"         // Relative path within module's include dir
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for constructor/defaults)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for publish)
#include "IMU_GyroOffsetsUpdated.hpp" // For handling gyro offset updates

#include <string>
#include <inttypes.h>
#include "esp_log.h"                    // Moved from header

ConfigurationService::ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey) :
    m_storageService(storage),
    m_configParser(parser),
    m_eventBus(bus),
    m_configKey(configKey),
    m_configChangePublisher(bus)
{}

esp_err_t ConfigurationService::init() {
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGI(TAG, "Initializing Configuration Service. Loading from key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_storageService.loadData(m_configKey, rawData);
    bool useDefaults = false;
    std::string reasonForDefaults;

    if (ret == ESP_OK) {
        if (rawData.empty()) {
            reasonForDefaults = "Config file empty";
            useDefaults = true;
        } else {
            ret = m_configParser.deserialize(rawData, m_configData);
            if (ret == ESP_OK) {
                std::string validationError;
                if (!m_configValidator.validate(m_configData, validationError)) {
                    reasonForDefaults = "Validation failed: " + validationError;
                    useDefaults = true;
                } else {
                    ESP_LOGI(TAG, "Configuration loaded, parsed, and validated successfully (Version: %d).", m_configData.config_version);
                }
            } else {
                reasonForDefaults = "JSON parse failed";
                useDefaults = true;
            }
        }
    } else if (ret == ESP_ERR_NOT_FOUND) {
        reasonForDefaults = "Config file not found";
        useDefaults = true;
    } else {
        reasonForDefaults = std::string("Storage load failed: ") + esp_err_to_name(ret);
        useDefaults = true;
    }

    if (useDefaults) {
        ESP_LOGW(TAG, "%s. Using default values.", reasonForDefaults.c_str());
        m_configData = ConfigData(); // Re-assign defaults
        ESP_LOGW(TAG, "Stored configuration preserved; defaults are in memory only.");
    }

    // Publish initial config state regardless of load source
    ESP_LOGI(TAG, "Publishing initial configuration state.");
    m_configChangePublisher.publishFullConfig(m_configData);

    return ESP_OK; // Return OK even if defaults were used, as the service is operational
}

esp_err_t ConfigurationService::save() {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Optionally add validation before saving? Depends if internal state could become invalid.
    // std::string validationError;
    // if (!validateConfig(m_configData, validationError)) {
    //     ESP_LOGE(TAG, "Pre-save validation failed: %s. Aborting save.", validationError.c_str());
    //     return ESP_FAIL;
    // }
    return saveInternal();
}

esp_err_t ConfigurationService::updateConfigFromJson(const std::string& json, std::string* error) {
    ConfigData tempConfig; // Create a temporary config to parse into
    esp_err_t ret = m_configParser.deserialize(json, tempConfig);
    if (ret != ESP_OK) {
        if (error) *error = ret == ESP_ERR_NOT_SUPPORTED ? "Unsupported config_version; supported versions are 1, 2 and 3" : "Invalid configuration JSON or field type";
        ESP_LOGE(TAG, "Failed to deserialize JSON for update: %s", esp_err_to_name(ret));
        return ret;
    }

    std::string validationError;
    if (!m_configValidator.validate(tempConfig, validationError)) {
        if (error) *error = validationError;
        ESP_LOGE(TAG, "Validation failed for config update: %s", validationError.c_str());
        return ESP_FAIL; // Return specific validation failure (maybe a different error code?)
    }

    // Store old config to detect changes
    ConfigData oldConfig;
    ConfigData newConfig;
    
    // Lock only when updating the internal state and saving. A v3 client must
    // send the revision it read; this prevents an older config card from
    // overwriting newer strategy parameters.
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;  // Store old config for comparison
        if (m_controlActive &&
            tempConfig.control.strategies.active != oldConfig.control.strategies.active) {
            if (error) *error = "Strategy changes require an inactive control mode";
            ESP_LOGW(TAG, "Rejected strategy change while control mode is active");
            return ESP_ERR_INVALID_STATE;
        }
        if (tempConfig.control.strategies.revision != oldConfig.control.strategies.revision) {
            if (error) *error = "Configuration revision conflict; reload before saving";
            ESP_LOGW(TAG, "Rejected config update with stale strategy revision (%" PRIu32 ", current %" PRIu32 ")",
                     tempConfig.control.strategies.revision,
                     oldConfig.control.strategies.revision);
            return ESP_ERR_INVALID_STATE;
        }
        if (tempConfig.control.strategies.active != oldConfig.control.strategies.active) {
            tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
        }
        if (tempConfig.control.strategies.nested_pid != oldConfig.control.strategies.nested_pid) {
            tempConfig.control.strategies.nested_pid.revision = oldConfig.control.strategies.nested_pid.revision + 1;
            tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
        }
        if (tempConfig.control.strategies.longitudinal_cascade != oldConfig.control.strategies.longitudinal_cascade) {
            tempConfig.control.strategies.longitudinal_cascade.revision = oldConfig.control.strategies.longitudinal_cascade.revision + 1;
            tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
        }
        m_configData = tempConfig; // Update internal state
        newConfig = m_configData;
        ret = saveInternal();      // Attempt to save
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save configuration after update.");
            m_configData = oldConfig;
            newConfig = oldConfig;
        } else {
             ESP_LOGI(TAG, "Configuration updated and saved successfully (Version: %d).", m_configData.config_version);
        }
    } // Mutex released
    
    if (ret != ESP_OK) {
        return ret;
    }

    // Publish granular events for changed components
    m_configChangePublisher.publishChanges(oldConfig, newConfig);
    
    // Still publish the full update as a fallback for components not using specific events
    ESP_LOGI(TAG, "Publishing general configuration update event.");
    m_configChangePublisher.publishFullConfig(newConfig);

    return ret; // Return the result of the save operation
}

esp_err_t ConfigurationService::applyPidConfig(const std::string& pidName, const PIDConfig& config, bool persist) {
    ConfigData oldConfig;
    ConfigData newConfig;
    esp_err_t ret = ESP_OK;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
        newConfig = m_configData;

        auto& nested = newConfig.control.strategies.nested_pid;
        if (pidName == "angle") {
            nested.angle = config;
        } else if (pidName == "speed_left") {
            nested.speed_left = config;
        } else if (pidName == "speed_right") {
            nested.speed_right = config;
        } else if (pidName == "yaw_angle") {
            nested.yaw_angle = config;
        } else if (pidName == "yaw_rate") {
            nested.yaw_rate = config;
        } else {
            ESP_LOGE(TAG, "Unknown PID name for applyPidConfig: %s", pidName.c_str());
            return ESP_ERR_INVALID_ARG;
        }

        if (nested != m_configData.control.strategies.nested_pid) {
            newConfig.control.strategies.nested_pid.revision =
                m_configData.control.strategies.nested_pid.revision + 1;
            newConfig.control.strategies.revision =
                m_configData.control.strategies.revision + 1;
        }

        std::string validationError;
        if (!m_configValidator.validate(newConfig, validationError)) {
            ESP_LOGE(TAG, "Rejected PID update for '%s': %s", pidName.c_str(), validationError.c_str());
            return ESP_FAIL;
        }

        m_configData = newConfig;
        if (persist) {
            ret = saveInternal();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to persist PID config '%s'", pidName.c_str());
                m_configData = oldConfig;
                newConfig = oldConfig;
            }
        }
    }

    if (oldConfig.control.strategies.nested_pid != newConfig.control.strategies.nested_pid) {
        m_configChangePublisher.publishChanges(oldConfig, newConfig);
        m_configChangePublisher.publishFullConfig(newConfig);
    }

    return ret;
}

void ConfigurationService::updateImuGyroOffsets(float x, float y, float z) {
    ESP_LOGI(TAG, "Updating IMU gyro offsets: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    
    // Store old config for comparison
    ConfigData oldConfig;
    ConfigData newConfig;
    
    // Update and save the new offsets
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData; // Store old config for comparison
        m_configData.imu.gyro_offset_x = x;
        m_configData.imu.gyro_offset_y = y;
        m_configData.imu.gyro_offset_z = z;
        newConfig = m_configData;
        
        // Save the updated offsets to persistent storage
        esp_err_t ret = saveInternal();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save gyro offsets to storage");
        } else {
            ESP_LOGI(TAG, "Gyro offsets saved to storage");
        }
    }
    
    // Publish granular IMU config update event
    m_configChangePublisher.publishImuConfig(newConfig.imu, false);
    
    // Also publish a general config update event for backward compatibility
    m_configChangePublisher.publishFullConfig(newConfig);
}

// EventHandler implementation
void ConfigurationService::handleEvent(const BaseEvent& event) {
    if (event.is<IMU_GyroOffsetsUpdated>()) {
        const IMU_GyroOffsetsUpdated& offsetEvent = event.as<IMU_GyroOffsetsUpdated>();
        updateImuGyroOffsets(offsetEvent.x_dps, offsetEvent.y_dps, offsetEvent.z_dps);
    } else if (event.is<CONTROL_RunModeChanged>()) {
        handleRunModeChanged(event.as<CONTROL_RunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void ConfigurationService::handleRunModeChanged(const CONTROL_RunModeChanged& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (event.armId < m_controlArmId) {
        return;
    }
    m_controlArmId = event.armId;
    m_controlActive = event.mode != ControlRunMode::DISABLED;
}

// Keep for backward compatibility
void ConfigurationService::subscribeToEvents(EventBus& bus) {
    ESP_LOGW(TAG, "ConfigurationService::subscribeToEvents is deprecated. Use EventBus::subscribe with EventHandler instead.");
}

esp_err_t ConfigurationService::getJsonString(std::string& jsonOutput) const {
    std::lock_guard<std::mutex> lock(m_mutex); // Lock for reading m_configData
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
