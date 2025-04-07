#pragma once

#include "ConfigData.hpp"               // Found via INCLUDE_DIRS
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "PIDController.hpp"            // Relative path within module's include dir
#include "esp_log.h"
#include <memory>                       // std::shared_ptr if needed later

// Forward declare needed classes
class ConfigurationService;
class BaseEvent;

// Define output struct
struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

class BalancingAlgorithm {
public:
    BalancingAlgorithm(ConfigurationService& configService, EventBus& eventBus);

    // Declarations only
    esp_err_t init();
    // Input units are DEGREES and DPS
    MotorEffort update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                      float targetLinVel_mps, float targetAngVel_dps); // Target angular in DPS
    void resetState();

    // --- Added Getter ---
    float getLastBalancingSpeedDPS() const { return m_last_balancing_speed_dps; } // In DPS
    // --- End Added Getter ---

private:
    static constexpr const char* TAG = "BalancingAlgo";
    ConfigurationService& m_configService;
    EventBus& m_eventBus;

    PIDController m_anglePid;
    PIDController m_speedPidLeft;
    PIDController m_speedPidRight;

    float m_angleSetpoint_rad = 0.0f; // Still keep internal goal as rad, convert in update
    float m_max_target_speed = 10.0f; // This limit is now in RAD/S - needs config or conversion
    float m_max_control_effort = 1.0f;
    float m_wheel_radius_m = 0.0325f;
    float m_robot_wheelbase_m = 0.15f;

    // --- Added Member ---
    float m_last_balancing_speed_dps = 0.0f; // Store last output of angle PID
    // --- End Added Member ---

    void handleConfigUpdate(const BaseEvent& event);
};