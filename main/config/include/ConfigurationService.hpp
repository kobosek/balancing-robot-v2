// main/config/include/ConfigurationService.hpp
#pragma once

// Include definitions needed for members/return types in this header
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include <string>
#include <mutex>
#include "esp_err.h" // Include ESP types

// Forward declarations for constructor arguments (interfaces)
class IStorageService; // Assuming this interface exists somewhere
class IConfigParser;   // Assuming this interface exists somewhere
// Forward declarations for events/other types
class ConfigUpdatedEvent;
class BaseEvent; // Needed by handleConfigUpdate if added later

class ConfigurationService {
public:
    // Constructor takes INTERFACE references
    ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey = "config.json");

    esp_err_t init();
    esp_err_t save();
    esp_err_t updateConfigFromJson(const std::string& json);
    esp_err_t getJsonString(std::string& jsonOutput) const;

    // Getters (inline is fine)
    ConfigData getConfigData() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData;
    }
    const WiFiConfig& getWiFiConfig() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.wifi;
    }
    const PIDConfig& getAnglePidConfig() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.anglePid;
    }
    const PIDConfig& getSpeedPidLeftConfig() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.speedPidLeft;
    }
    const PIDConfig& getSpeedPidRightConfig() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.speedPidRight;
    }
     const MPU6050Config& getMpu6050Config() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.imu;
    }
     const MainLoopConfig& getMainLoopConfig() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_configData.mainLoop;
    }
     const EncoderConfig& getEncoderConfig() const {
         std::lock_guard<std::mutex> lock(m_mutex);
         return m_configData.encoder;
     }
     const MotorConfig& getMotorConfig() const {
         std::lock_guard<std::mutex> lock(m_mutex);
         return m_configData.motor;
     }
      const BatteryConfig& getBatteryConfig() const {
         std::lock_guard<std::mutex> lock(m_mutex);
         return m_configData.battery;
     }
    // --- End re-add ---

private:
    static constexpr const char* TAG = "ConfigService";
    IStorageService& m_storageService; // Store interface reference
    IConfigParser& m_configParser;     // Store interface reference
    EventBus& m_eventBus;
    const std::string m_configKey;

    ConfigData m_configData;
    mutable std::mutex m_mutex;

    esp_err_t saveInternal();
    // Optional: Add event handler if needed later
    // void handleConfigUpdate(const BaseEvent& event);
};