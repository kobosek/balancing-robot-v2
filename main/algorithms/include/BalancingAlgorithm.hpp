
#pragma once

#include "ConfigData.hpp" // Include full definition
#include "EventBus.hpp"
#include "EventHandler.hpp" // Include for EventHandler base class
#include "PIDController.hpp"
#include "esp_log.h"
#include <mutex>

// Forward declarations for event classes
class CONFIG_PidConfigUpdate;
class CONFIG_FullConfigUpdate;

struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

class BalancingAlgorithm : public EventHandler {
public:
    // Constructor takes initial config structs
    BalancingAlgorithm(EventBus& eventBus,
                       const PIDConfig& initialAnglePid,
                       const PIDConfig& initialSpeedLeftPid,
                       const PIDConfig& initialSpeedRightPid,
                       const PIDConfig& initialYawRatePid,
                       const ControlConfig& initialControl,
                       const EncoderConfig& initialEncoder,
                       const RobotDimensionsConfig& initialDimensions);

    esp_err_t init();
    MotorEffort update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                      float currentYawRate_dps,
                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                      float targetPitchOffset_deg, float targetAngVel_dps);
    void resetState();

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return "BalancingAlgorithm"; }

    // --- Getters for Telemetry ---
    float getLastSpeedSetpointLeftDPS() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_last_speed_setpoint_left_dps;
    }
    float getLastSpeedSetpointRightDPS() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_last_speed_setpoint_right_dps;
    }
    bool isYawControlEnabled() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_yaw_control_enabled;
    }
    // --- End Getters ---

private:
    static constexpr const char* TAG = "BalancingAlgo";
    EventBus& m_eventBus;
    mutable std::mutex m_mutex;
    // ConfigurationService& m_configService; // REMOVE

    PIDController m_anglePid;
    PIDController m_speedPidLeft;
    PIDController m_speedPidRight;
    PIDController m_yawRatePid;

    // Configurable parameters (store locally)
    float m_max_control_effort = 1.0f; // Might load this from somewhere eventually
    float m_wheel_radius_m;
    float m_robot_wheelbase_m;
    float m_angle_pid_output_min;
    float m_angle_pid_output_max;
    bool m_yaw_control_enabled = false;

    // --- Members for Telemetry ---
    float m_last_speed_setpoint_left_dps = 0.0f;
    float m_last_speed_setpoint_right_dps = 0.0f;
    // --- End Members ---

    // Internal helpers to apply config from events
    void applyConfig(const ConfigData& config);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event);
    
    // Helper to update dimensions from config
    void updateDimensions(const EncoderConfig& encoderConfig, const RobotDimensionsConfig& dimensionsConfig);
};
