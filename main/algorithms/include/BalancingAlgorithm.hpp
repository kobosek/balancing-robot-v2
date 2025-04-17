// ================================================
// File: main/algorithms/include/BalancingAlgorithm.hpp
// ================================================
#pragma once

#include "ConfigData.hpp" // Include full definition
#include "EventBus.hpp"
#include "PIDController.hpp"
#include "esp_log.h"

class BaseEvent; // Forward declare

struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

class BalancingAlgorithm {
public:
    // Constructor takes initial config structs
    BalancingAlgorithm(EventBus& eventBus,
                       const PIDConfig& initialAnglePid,
                       const PIDConfig& initialSpeedLeftPid,
                       const PIDConfig& initialSpeedRightPid,
                       const PIDConfig& initialYawRatePid,
                       const EncoderConfig& initialEncoder,
                       const RobotDimensionsConfig& initialDimensions);

    esp_err_t init();
    MotorEffort update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                      float currentYawRate_dps,
                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                      float targetPitchOffset_deg, float targetAngVel_dps);
    void resetState();

    void subscribeToEvents(EventBus& bus);

    // --- Getters for Telemetry ---
    float getLastSpeedSetpointLeftDPS() const { return m_last_speed_setpoint_left_dps; }
    float getLastSpeedSetpointRightDPS() const { return m_last_speed_setpoint_right_dps; }
    // --- End Getters ---

private:
    static constexpr const char* TAG = "BalancingAlgo";
    EventBus& m_eventBus;
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

    // --- Members for Telemetry ---
    float m_last_speed_setpoint_left_dps = 0.0f;
    float m_last_speed_setpoint_right_dps = 0.0f;
    // --- End Members ---

    // Internal helper to apply config from event
    void applyConfig(const ConfigData& config);
    void handleConfigUpdate(const BaseEvent& event);
};