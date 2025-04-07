// main/algorithms/BalancingAlgorithm.cpp
#include "BalancingAlgorithm.hpp"
#include "ConfigurationService.hpp" // Found via INCLUDE_DIRS
#include "EventBus.hpp"             // Found via INCLUDE_DIRS
#include "ConfigUpdatedEvent.hpp"   // Found via INCLUDE_DIRS
#include "BaseEvent.hpp"            // Found via INCLUDE_DIRS
#include "EventTypes.hpp"           // Found via INCLUDE_DIRS
#include "esp_check.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;

BalancingAlgorithm::BalancingAlgorithm(ConfigurationService& configService, EventBus& eventBus) :
    m_configService(configService),
    m_eventBus(eventBus),
    m_anglePid("angle"),
    m_speedPidLeft("speed_left"),
    m_speedPidRight("speed_right"),
    m_last_balancing_speed_dps(0.0f) // Initialize member
{
     ESP_LOGI(TAG, "Balancing Algorithm created.");
     // Subscribe to config updates before loading initial config
     m_eventBus.subscribe(EventType::CONFIG_UPDATED, [this](const BaseEvent& ev){ this->handleConfigUpdate(ev); });
     handleConfigUpdate(BaseEvent(EventType::CONFIG_UPDATED)); // Load constants initially
}

esp_err_t BalancingAlgorithm::init() {
    ESP_LOGI(TAG, "Initializing Balancing Algorithm...");
    esp_err_t ret;
    // PID params are already loaded by handleConfigUpdate called in constructor
    ret = m_anglePid.init(m_configService.getAnglePidConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Angle PID");
    ret = m_speedPidLeft.init(m_configService.getSpeedPidLeftConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Speed Left PID");
    ret = m_speedPidRight.init(m_configService.getSpeedPidRightConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Speed Right PID");
    // Subscription already done in constructor
    resetState();
    ESP_LOGI(TAG, "Balancing Algorithm Initialized.");
    return ESP_OK;
}


void BalancingAlgorithm::resetState() {
    //ESP_LOGI(TAG, "Resetting Balancing Algorithm State.");
    m_anglePid.reset();
    m_speedPidLeft.reset();
    m_speedPidRight.reset();
    m_angleSetpoint_rad = 0.0f;
    m_last_balancing_speed_dps = 0.0f; // Reset stored value
}

MotorEffort BalancingAlgorithm::update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                                      float targetLinVel_mps, float targetAngVel_dps)
{
    MotorEffort effort = {0.0f, 0.0f};
    if (dt <= 0) { ESP_LOGW(TAG, "Invalid dt (%.4f)", dt); return effort; }

    // --- Angle Control (Using Degrees) ---
    float angleSetpoint_deg = m_angleSetpoint_rad * RAD_TO_DEG;
    float desiredSpeed_dps = m_anglePid.compute(angleSetpoint_deg, currentPitch_deg, dt); // Already clamped to ±720 by PID

    // --- !!! REMOVED REDUNDANT CLAMP !!! ---
    // float max_target_speed_dps = m_max_target_speed * RAD_TO_DEG;
    // desiredSpeed_dps = std::max(-max_target_speed_dps, std::min(max_target_speed_dps, desiredSpeed_dps));
    // --- !!! END REMOVAL !!! ---

    // Store the (potentially ±720) balancing speed component
    m_last_balancing_speed_dps = desiredSpeed_dps;

    // --- Speed Control (Using Degrees/Sec) ---
    float targetSpeedLin_dps = 0.0f;
    if (m_wheel_radius_m > 1e-5) {
        targetSpeedLin_dps = (targetLinVel_mps / m_wheel_radius_m) * RAD_TO_DEG; // m/s -> rad/s -> dps
    }
    float baseSpeedSetpoint_dps = desiredSpeed_dps + targetSpeedLin_dps;
    float speedDiff_dps = 0.0f;
    if (m_wheel_radius_m > 1e-5 && m_robot_wheelbase_m > 1e-5) { // Add check for wheelbase
        // Ensure wheelbase is positive before using it
        speedDiff_dps = (m_robot_wheelbase_m / (2.0f * m_wheel_radius_m)) * targetAngVel_dps;
    }
    float speedSetpointLeft_dps = baseSpeedSetpoint_dps - speedDiff_dps;
    float speedSetpointRight_dps = baseSpeedSetpoint_dps + speedDiff_dps;

    // Compute effort using speed PIDs (output clamped ±1.0 by PID config)
    effort.left = m_speedPidLeft.compute(speedSetpointLeft_dps, currentSpeedLeft_dps, dt);
    effort.right = m_speedPidRight.compute(speedSetpointRight_dps, currentSpeedRight_dps, dt);

    // Re-clamp effort just in case PID limits weren't exactly ±1.0 (though they are)
    effort.left = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.left));
    effort.right = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.right));

    ESP_LOGV(TAG, "P:%.2f|BSpd:%.1f|LSpd:%.1f RSpd:%.1f|LSet:%.1f RSet:%.1f|LEff:%.2f REff:%.2f",
             currentPitch_deg, desiredSpeed_dps, // Log potentially ±720 Balance Speed
             currentSpeedLeft_dps, currentSpeedRight_dps,
             speedSetpointLeft_dps, speedSetpointRight_dps,
             effort.left, effort.right);

    return effort;
}

void BalancingAlgorithm::handleConfigUpdate(const BaseEvent& event) {
     ESP_LOGI(TAG, "Handling config update event.");
     // Reload PID parameters (this loads the ±720 limits for angle PID)
     m_anglePid.updateParams(m_configService.getAnglePidConfig());
     m_speedPidLeft.updateParams(m_configService.getSpeedPidLeftConfig());
     m_speedPidRight.updateParams(m_configService.getSpeedPidRightConfig());
     // Reload physical parameters
     m_wheel_radius_m = m_configService.getConfigData().encoder.wheel_diameter_mm / 2000.0f;
     ESP_LOGI(TAG, "Internal parameters updated from configuration. Wheel Radius: %.4f m", m_wheel_radius_m);
}