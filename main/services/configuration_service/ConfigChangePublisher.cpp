#include "ConfigChangePublisher.hpp"

#include "CONFIG_BatteryConfigUpdate.hpp"
#include "CONFIG_BehaviorConfigUpdate.hpp"
#include "CONFIG_ControlConfigUpdate.hpp"
#include "CONFIG_EncoderConfigUpdate.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_ImuConfigUpdate.hpp"
#include "CONFIG_MotorConfigUpdate.hpp"
#include "CONFIG_PidConfigUpdate.hpp"
#include "CONFIG_WiFiConfigUpdate.hpp"
#include "EventBus.hpp"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "ConfigPublisher";
}

ConfigChangePublisher::ConfigChangePublisher(EventBus& eventBus) : m_eventBus(eventBus) {}

void ConfigChangePublisher::publishFullConfig(const ConfigData& config) const {
    CONFIG_FullConfigUpdate event(config);
    m_eventBus.publish(event);
}

void ConfigChangePublisher::publishImuConfig(const MPU6050Config& config, bool requiresHardwareInit) const {
    CONFIG_ImuConfigUpdate event(config, requiresHardwareInit);
    m_eventBus.publish(event);
}

void ConfigChangePublisher::publishChanges(const ConfigData& oldConfig, const ConfigData& newConfig) const {
    ESP_LOGI(TAG, "Publishing granular configuration events for changed components");

    if (oldConfig.pid_angle != newConfig.pid_angle) {
        CONFIG_PidConfigUpdate event("angle", newConfig.pid_angle);
        m_eventBus.publish(event);
    }
    if (oldConfig.pid_speed_left != newConfig.pid_speed_left) {
        CONFIG_PidConfigUpdate event("speed_left", newConfig.pid_speed_left);
        m_eventBus.publish(event);
    }
    if (oldConfig.pid_speed_right != newConfig.pid_speed_right) {
        CONFIG_PidConfigUpdate event("speed_right", newConfig.pid_speed_right);
        m_eventBus.publish(event);
    }
    if (oldConfig.pid_yaw_angle != newConfig.pid_yaw_angle) {
        CONFIG_PidConfigUpdate event("yaw_angle", newConfig.pid_yaw_angle);
        m_eventBus.publish(event);
    }
    if (oldConfig.pid_yaw_rate != newConfig.pid_yaw_rate) {
        CONFIG_PidConfigUpdate event("yaw_rate", newConfig.pid_yaw_rate);
        m_eventBus.publish(event);
    }
    if (oldConfig.imu != newConfig.imu) {
        const bool requiresHardwareInit = oldConfig.imu.requiresHardwareInit(newConfig.imu);
        CONFIG_ImuConfigUpdate event(newConfig.imu, requiresHardwareInit);
        m_eventBus.publish(event);
    }
    if (oldConfig.motor != newConfig.motor) {
        const bool requiresHardwareInit = oldConfig.motor.requiresHardwareInit(newConfig.motor);
        CONFIG_MotorConfigUpdate event(newConfig.motor, requiresHardwareInit);
        m_eventBus.publish(event);
    }
    if (oldConfig.encoder != newConfig.encoder) {
        const bool requiresHardwareInit = oldConfig.encoder.requiresHardwareInit(newConfig.encoder);
        CONFIG_EncoderConfigUpdate event(newConfig.encoder, requiresHardwareInit);
        m_eventBus.publish(event);
    }
    if (oldConfig.battery != newConfig.battery) {
        const bool requiresHardwareInit = oldConfig.battery.requiresHardwareInit(newConfig.battery);
        CONFIG_BatteryConfigUpdate event(newConfig.battery, requiresHardwareInit);
        m_eventBus.publish(event);
    }
    if (oldConfig.behavior != newConfig.behavior) {
        CONFIG_BehaviorConfigUpdate event(newConfig.behavior);
        m_eventBus.publish(event);
    }
    if (oldConfig.wifi != newConfig.wifi) {
        CONFIG_WiFiConfigUpdate event(newConfig.wifi);
        m_eventBus.publish(event);
    }
    if (oldConfig.control != newConfig.control) {
        CONFIG_ControlConfigUpdate event(newConfig.control);
        m_eventBus.publish(event);
    }
}
