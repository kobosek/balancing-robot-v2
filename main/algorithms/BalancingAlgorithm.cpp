// ================================================
// File: main/algorithms/BalancingAlgorithm.cpp
// ================================================
#include "BalancingAlgorithm.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_PidConfigUpdate.hpp"
#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_check.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;

// Constructor takes initial config structs
BalancingAlgorithm::BalancingAlgorithm(EventBus& eventBus,
                                       const PIDConfig& initialAnglePid,
                                       const PIDConfig& initialSpeedLeftPid,
                                       const PIDConfig& initialSpeedRightPid,
                                       const PIDConfig& initialYawRatePid,
                                       const EncoderConfig& initialEncoder,
                                       const RobotDimensionsConfig& initialDimensions) :
    m_eventBus(eventBus),
    m_anglePid("angle"),
    m_speedPidLeft("speed_left"),
    m_speedPidRight("speed_right"),
    m_yawRatePid("yaw_rate"),
    m_last_speed_setpoint_left_dps(0.0f),
    m_last_speed_setpoint_right_dps(0.0f)
    // Initialize other members with temporary values before applying initial config
    // m_wheel_radius_m, m_robot_wheelbase_m, etc. will be set by applyConfig
{
     ESP_LOGI(TAG, "Balancing Algorithm created.");

     // Create a temporary ConfigData to hold initial values for applyConfig
     ConfigData initial_config_data;
     initial_config_data.pid_angle = initialAnglePid;
     initial_config_data.pid_speed_left = initialSpeedLeftPid;
     initial_config_data.pid_speed_right = initialSpeedRightPid;
     initial_config_data.pid_yaw_rate = initialYawRatePid;
     initial_config_data.encoder = initialEncoder;
     initial_config_data.dimensions = initialDimensions;

     applyConfig(initial_config_data); // Apply initial config values
}

esp_err_t BalancingAlgorithm::init() {
    ESP_LOGI(TAG, "Initializing Balancing Algorithm...");
    // PIDs already initialized in constructor with initial config via applyConfig
    resetState();
    ESP_LOGI(TAG, "Balancing Algorithm Initialized.");
    return ESP_OK;
}

void BalancingAlgorithm::handleEvent(const BaseEvent& event) {
    // Central event handler that dispatches to specific handlers based on event type
    switch (event.type) {
        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;
            
        case EventType::CONFIG_PID_UPDATE:
            handlePIDConfigUpdate(static_cast<const CONFIG_PidConfigUpdate&>(event));
            break;
            
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

void BalancingAlgorithm::resetState() {
    m_anglePid.reset();
    m_speedPidLeft.reset();
    m_speedPidRight.reset();
    m_yawRatePid.reset();
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
}

MotorEffort BalancingAlgorithm::update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                                      float currentYawRate_dps,
                                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                                      float targetPitchOffset_deg, float targetAngVel_dps)
{
    MotorEffort effort = {0.0f, 0.0f};
    if (dt <= 0) { ESP_LOGW(TAG, "Invalid dt (%.4f)", dt); return effort; }

    // --- Angle Control (Determines base speed for balance/tilt) ---
    float baseSpeed_dps = m_anglePid.compute(targetPitchOffset_deg, currentPitch_deg, dt);

    // --- Commanded Turn Speed Difference ---
    float commandedTurnDiff_dps = 0.0f;
    // Use locally stored, config-loaded wheelbase
    if (m_wheel_radius_m > 1e-5 && m_robot_wheelbase_m > 1e-5) {
        commandedTurnDiff_dps = (m_robot_wheelbase_m / (2.0f * m_wheel_radius_m)) * targetAngVel_dps;
    }

    // --- Yaw Rate Stabilization Correction ---
    float yawRateError_dps = targetAngVel_dps - currentYawRate_dps;
    float yawCorrectionDiff_dps = m_yawRatePid.compute(targetAngVel_dps, currentYawRate_dps, dt);

    // --- Combine Components for Wheel Speed Setpoints ---
    float speedSetpointLeft_dps = baseSpeed_dps - commandedTurnDiff_dps;// - yawCorrectionDiff_dps;
    float speedSetpointRight_dps = baseSpeed_dps + commandedTurnDiff_dps;// + yawCorrectionDiff_dps;

    // --- Clamp Final Speed Setpoints ---
    speedSetpointLeft_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointLeft_dps));
    speedSetpointRight_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointRight_dps));

    m_last_speed_setpoint_left_dps = speedSetpointLeft_dps;
    m_last_speed_setpoint_right_dps = speedSetpointRight_dps;

    // --- Compute Motor Effort using Speed PIDs ---
    effort.left = m_speedPidLeft.compute(speedSetpointLeft_dps, currentSpeedLeft_dps, dt);
    effort.right = m_speedPidRight.compute(speedSetpointRight_dps, currentSpeedRight_dps, dt);

    // Final clamp on effort
    effort.left = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.left));
    effort.right = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.right));

    ESP_LOGV(TAG, "P:%.1f|TgtP:%.1f|Yaw:%.1f|TgtAV:%.1f|YawE:%.1f|YawC:%.1f|CmdDiff:%.1f|LSet:%.1f RSet:%.1f|LCur:%.1f RCur:%.1f|LEff:%.2f REff:%.2f",
             currentPitch_deg, targetPitchOffset_deg, currentYawRate_dps, targetAngVel_dps, yawRateError_dps, yawCorrectionDiff_dps, commandedTurnDiff_dps,
             speedSetpointLeft_dps, speedSetpointRight_dps,
             currentSpeedLeft_dps, currentSpeedRight_dps,
             effort.left, effort.right);

    return effort;
}

// Helper to apply config values
void BalancingAlgorithm::applyConfig(const ConfigData& config) {
     ESP_LOGD(TAG, "Applying new config to BalancingAlgorithm.");
     m_anglePid.updateParams(config.pid_angle);
     m_speedPidLeft.updateParams(config.pid_speed_left);
     m_speedPidRight.updateParams(config.pid_speed_right);
     m_yawRatePid.updateParams(config.pid_yaw_rate);

     m_angle_pid_output_min = config.pid_angle.getOutputMin();
     m_angle_pid_output_max = config.pid_angle.getOutputMax();
     ESP_LOGD(TAG, "Stored Angle PID limits: Min=%.1f, Max=%.1f", m_angle_pid_output_min, m_angle_pid_output_max);

     // Update dimensions from config
     updateDimensions(config.encoder, config.dimensions);
}

// Handle config update event
void BalancingAlgorithm::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
     ESP_LOGD(TAG, "Handling general config update event."); // Use DEBUG level
     applyConfig(event.configData); // Apply the full config payload
}

// Handle granular PID config update event
void BalancingAlgorithm::handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event) {
    ESP_LOGD(TAG, "Handling granular PID config update for '%s'", event.pidName.c_str());
    
    // Update the specific PID controller based on the name
    if (event.pidName == "angle") {
        m_anglePid.updateParams(event.config);
        m_angle_pid_output_min = event.config.getOutputMin();
        m_angle_pid_output_max = event.config.getOutputMax();
        ESP_LOGD(TAG, "Updated angle PID controller. Limits: Min=%.1f, Max=%.1f", 
                 m_angle_pid_output_min, m_angle_pid_output_max);
    } 
    else if (event.pidName == "speed_left") {
        m_speedPidLeft.updateParams(event.config);
        ESP_LOGD(TAG, "Updated speed_left PID controller");
    } 
    else if (event.pidName == "speed_right") {
        m_speedPidRight.updateParams(event.config);
        ESP_LOGD(TAG, "Updated speed_right PID controller");
    } 
    else if (event.pidName == "yaw_rate") {
        m_yawRatePid.updateParams(event.config);
        ESP_LOGD(TAG, "Updated yaw_rate PID controller");
    } 
    else {
        ESP_LOGW(TAG, "Received PID config update for unknown controller: %s", event.pidName.c_str());
    }
}

// Helper to update dimensions from config
void BalancingAlgorithm::updateDimensions(const EncoderConfig& encoderConfig, const RobotDimensionsConfig& dimensionsConfig) {
    m_wheel_radius_m = encoderConfig.wheel_diameter_mm / 2000.0f;
    m_robot_wheelbase_m = dimensionsConfig.wheelbase_m;
    ESP_LOGD(TAG, "Internal dimensions updated. Wheel Radius: %.4f m, Wheelbase: %.4f m", 
             m_wheel_radius_m, m_robot_wheelbase_m);
}