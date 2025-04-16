#pragma once

#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "PIDController.hpp"
#include "esp_log.h"

class ConfigurationService;
class BaseEvent;

struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

class BalancingAlgorithm {
public:
    BalancingAlgorithm(ConfigurationService& configService, EventBus& eventBus);

    esp_err_t init();
    MotorEffort update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                      float currentYawRate_dps, //
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
    ConfigurationService& m_configService;
    EventBus& m_eventBus;

    PIDController m_anglePid;
    PIDController m_speedPidLeft;
    PIDController m_speedPidRight;
    PIDController m_yawRatePid; 

    float m_max_control_effort = 1.0f;
    float m_wheel_radius_m = 0.0325f;
    float m_robot_wheelbase_m = 0.15f;

    // Store PID limits locally for clamping
    float m_angle_pid_output_min = -720.0f;
    float m_angle_pid_output_max = 720.0f;


    // --- Members for Telemetry ---
    float m_last_speed_setpoint_left_dps = 0.0f;
    float m_last_speed_setpoint_right_dps = 0.0f;
    // --- End Members ---

    void handleConfigUpdate(const BaseEvent& event);
};