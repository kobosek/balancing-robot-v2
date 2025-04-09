#pragma once


#include "ConfigData.hpp"
#include "EventBus.hpp"
#include <string>
#include <mutex>
#include "esp_err.h" // Include ESP types

class IStorageService;
class IConfigParser;
class ConfigUpdatedEvent;
class BaseEvent;

class ConfigurationService {
public:
    ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey = "config.json");

    esp_err_t init();
    esp_err_t save();
    esp_err_t updateConfigFromJson(const std::string& json);
    esp_err_t getJsonString(std::string& jsonOutput) const;

    // Getters
    ConfigData getConfigData() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData; }
    const WiFiConfig& getWiFiConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.wifi; }
    const PIDConfig& getAnglePidConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.anglePid; }
    const PIDConfig& getSpeedPidLeftConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.speedPidLeft; }
    const PIDConfig& getSpeedPidRightConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.speedPidRight; }
    const PIDConfig& getYawRatePidConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.yawRatePid; } // <<< ADDED Yaw Rate PID Getter
    const MPU6050Config& getMpu6050Config() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.imu; }
    const MainLoopConfig& getMainLoopConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.mainLoop; }
    const EncoderConfig& getEncoderConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.encoder; }
    const MotorConfig& getMotorConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.motor; }
    const BatteryConfig& getBatteryConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.battery; }
    const ControlConfig& getControlConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control; }

private:
    static constexpr const char* TAG = "ConfigService";
    IStorageService& m_storageService;
    IConfigParser& m_configParser;
    EventBus& m_eventBus;
    const std::string m_configKey;
    ConfigData m_configData;
    mutable std::mutex m_mutex;

    esp_err_t saveInternal();
};