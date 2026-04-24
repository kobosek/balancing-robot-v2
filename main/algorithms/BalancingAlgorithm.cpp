// ================================================
// File: main/algorithms/BalancingAlgorithm.cpp
// ================================================
#include "BalancingAlgorithm.hpp"
#include "NestedPidBalanceStrategy.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_PidConfigUpdate.hpp"
#include "BaseEvent.hpp"
#include "esp_check.h"

// Constructor takes initial config structs
BalancingAlgorithm::BalancingAlgorithm(EventBus& eventBus,
                                       const PIDConfig& initialAnglePid,
                                       const PIDConfig& initialSpeedLeftPid,
                                       const PIDConfig& initialSpeedRightPid,
                                       const PIDConfig& initialYawRatePid,
                                       const ControlConfig& initialControl,
                                       const EncoderConfig& initialEncoder,
                                       const RobotDimensionsConfig& initialDimensions) :
    m_eventBus(eventBus),
    m_strategy(std::make_unique<NestedPidBalanceStrategy>())
{
     ESP_LOGI(TAG, "Balancing Algorithm created.");

     ConfigData initial_config_data;
     initial_config_data.pid_angle = initialAnglePid;
     initial_config_data.pid_speed_left = initialSpeedLeftPid;
     initial_config_data.pid_speed_right = initialSpeedRightPid;
     initial_config_data.pid_yaw_rate = initialYawRatePid;
     initial_config_data.control = initialControl;
     initial_config_data.encoder = initialEncoder;
     initial_config_data.dimensions = initialDimensions;

     applyConfig(initial_config_data);
}

esp_err_t BalancingAlgorithm::init() {
    ESP_LOGI(TAG, "Initializing Balancing Algorithm...");
    // Active strategy is already initialized in constructor with initial config.
    resetState();
    ESP_LOGI(TAG, "Balancing Algorithm Initialized.");
    return ESP_OK;
}

void BalancingAlgorithm::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else if (event.is<CONFIG_PidConfigUpdate>()) {
        handlePIDConfigUpdate(event.as<CONFIG_PidConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void BalancingAlgorithm::resetState() {
    if (m_strategy) {
        m_strategy->reset();
    }
}

MotorEffort BalancingAlgorithm::update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                                      float currentYawRate_dps,
                                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                                      float targetPitchOffset_deg, float targetAngVel_dps)
{
    if (!m_strategy) {
        return {};
    }

    const BalanceControlInput input = {
        dt,
        currentPitch_deg,
        currentPitchRate_dps,
        currentYawRate_dps,
        currentSpeedLeft_dps,
        currentSpeedRight_dps,
        targetPitchOffset_deg,
        targetAngVel_dps
    };
    return m_strategy->update(input);
}

// Helper to apply config values
void BalancingAlgorithm::applyConfig(const ConfigData& config) {
     if (!m_strategy) {
         return;
     }
     ESP_LOGD(TAG, "Applying new config to balance strategy '%s'.", m_strategy->name());
     m_strategy->applyConfig(config);
}

// Handle config update event
void BalancingAlgorithm::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
     ESP_LOGD(TAG, "Handling general config update event."); // Use DEBUG level
     applyConfig(event.configData); // Apply the full config payload
}

// Handle granular PID config update event
void BalancingAlgorithm::handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event) {
    if (m_strategy) {
        m_strategy->updatePidConfig(event.pidName, event.config);
    }
}

float BalancingAlgorithm::getLastSpeedSetpointLeftDPS() const {
    return m_strategy ? m_strategy->getLastSpeedSetpointLeftDPS() : 0.0f;
}

float BalancingAlgorithm::getLastSpeedSetpointRightDPS() const {
    return m_strategy ? m_strategy->getLastSpeedSetpointRightDPS() : 0.0f;
}

bool BalancingAlgorithm::isYawControlEnabled() const {
    return m_strategy ? m_strategy->isYawControlEnabled() : false;
}
