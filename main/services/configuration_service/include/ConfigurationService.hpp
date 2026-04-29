#pragma once

#include "ConfigData.hpp"
#include "ConfigChangePublisher.hpp"
#include "ConfigValidator.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include <string>
#include <mutex>
#include "esp_err.h" // Include ESP types

class IStorageService;
class IConfigParser;
class ConfigUpdatedEvent;
class BaseEvent;
class IMU_GyroOffsetsUpdated;

class ConfigurationService : public EventHandler {
public:
    ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey = "config.json");

    esp_err_t init();
    esp_err_t save();
    esp_err_t updateConfigFromJson(const std::string& json);
    esp_err_t applyPidConfig(const std::string& pidName, const PIDConfig& config, bool persist);
    esp_err_t getJsonString(std::string& jsonOutput) const;

    // Getters
    ConfigData getConfigData() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData; }
    const WiFiConfig& getWiFiConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.wifi; }
    const PIDConfig& getPidAngleConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_angle; }
    const PIDConfig& getPidSpeedLeftConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_speed_left; }
    const PIDConfig& getPidSpeedRightConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_speed_right; }
    const PIDConfig& getPidYawAngleConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_yaw_angle; }
    const PIDConfig& getPidYawRateConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_yaw_rate; }
    const PidTuningConfig& getPidTuningConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_tuning; }
    const MPU6050Config& getMpu6050Config() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.imu; }
    const MainLoopConfig& getMainLoopConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.mainLoop; }
    const EncoderConfig& getEncoderConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.encoder; }
    const MotorConfig& getMotorConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.motor; }
    const BatteryConfig& getBatteryConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.battery; }
    const ControlConfig& getControlConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control; }
    // --- Getters for NEW sections ---
    const SystemBehaviorConfig& getSystemBehaviorConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.behavior; }
    const RobotDimensionsConfig& getRobotDimensionsConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.dimensions; }
    const WebServerConfig& getWebServerConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.web; }

    // Update persistent gyro offsets
    void updateImuGyroOffsets(float x, float y, float z);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Keep for backward compatibility
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "ConfigService";
    IStorageService& m_storageService;
    IConfigParser& m_configParser;
    EventBus& m_eventBus;
    const std::string m_configKey;
    ConfigValidator m_configValidator;
    ConfigChangePublisher m_configChangePublisher;
    ConfigData m_configData; // Holds the current configuration
    mutable std::mutex m_mutex; // Protects m_configData

    esp_err_t saveInternal(); // Internal save helper (assumes mutex is held)
};
