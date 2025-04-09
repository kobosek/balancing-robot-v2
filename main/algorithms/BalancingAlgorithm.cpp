#include "BalancingAlgorithm.hpp"
#include "ConfigurationService.hpp"
#include "EventBus.hpp"
#include "ConfigUpdatedEvent.hpp"
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

BalancingAlgorithm::BalancingAlgorithm(ConfigurationService& configService, EventBus& eventBus) :
    m_configService(configService),
    m_eventBus(eventBus),
    m_anglePid("angle"),
    m_speedPidLeft("speed_left"),
    m_speedPidRight("speed_right"),
    m_yawRatePid("yaw_rate"), // <<< Initialize Yaw Rate PID
    m_last_speed_setpoint_left_dps(0.0f),
    m_last_speed_setpoint_right_dps(0.0f)
{
     ESP_LOGI(TAG, "Balancing Algorithm created.");
     m_eventBus.subscribe(EventType::CONFIG_UPDATED, [this](const BaseEvent& ev){ this->handleConfigUpdate(ev); });
     handleConfigUpdate(BaseEvent(EventType::CONFIG_UPDATED));
}

esp_err_t BalancingAlgorithm::init() {
    ESP_LOGI(TAG, "Initializing Balancing Algorithm...");
    esp_err_t ret;
    ret = m_anglePid.init(m_configService.getAnglePidConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Angle PID");
    ret = m_speedPidLeft.init(m_configService.getSpeedPidLeftConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Speed Left PID");
    ret = m_speedPidRight.init(m_configService.getSpeedPidRightConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Speed Right PID");
    ret = m_yawRatePid.init(m_configService.getYawRatePidConfig()); ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Yaw Rate PID"); // <<< Initialize Yaw Rate PID
    resetState();
    ESP_LOGI(TAG, "Balancing Algorithm Initialized.");
    return ESP_OK;
}


void BalancingAlgorithm::resetState() {
    m_anglePid.reset();
    m_speedPidLeft.reset();
    m_speedPidRight.reset();
    m_yawRatePid.reset(); // <<< Reset Yaw Rate PID
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
}

// <<< MODIFIED update signature and logic >>>
MotorEffort BalancingAlgorithm::update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                                      float currentYawRate_dps, // <<< ADDED Yaw Rate Input
                                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                                      float targetPitchOffset_deg, float targetAngVel_dps)
{
    MotorEffort effort = {0.0f, 0.0f};
    if (dt <= 0) { ESP_LOGW(TAG, "Invalid dt (%.4f)", dt); return effort; }

    // --- Angle Control (Determines base speed for balance/tilt) ---
    float baseSpeed_dps = m_anglePid.compute(targetPitchOffset_deg, currentPitch_deg, dt);
    // baseSpeed_dps is clamped by angle PID limits (e.g., +/- 720)

    // --- Commanded Turn Speed Difference ---
    float commandedTurnDiff_dps = 0.0f;
    if (m_wheel_radius_m > 1e-5 && m_robot_wheelbase_m > 1e-5) {
        commandedTurnDiff_dps = (m_robot_wheelbase_m / (2.0f * m_wheel_radius_m)) * targetAngVel_dps;
    }

    // --- Yaw Rate Stabilization Correction ---
    float yawRateError_dps = targetAngVel_dps - currentYawRate_dps;
    // The yaw rate PID output is a *correction* to the speed difference.
    // Its output range should be relatively small compared to the base speed.
    // Let's assume output is e.g. +/- 100 dps correction.
    float yawCorrectionDiff_dps = m_yawRatePid.compute(targetAngVel_dps, currentYawRate_dps, dt);
    // The PID output needs to be applied differentially. A positive error (robot turning slower than commanded right turn, or drifting left)
    // needs a correction causing a right turn (increase right, decrease left).
    // A positive yawCorrectionDiff_dps should increase the right turn effect.

    // --- Combine Components for Wheel Speed Setpoints ---
    // Start with base balancing speed
    // Add commanded turn difference
    // Add yaw stabilization difference correction
    float speedSetpointLeft_dps = baseSpeed_dps - commandedTurnDiff_dps - yawCorrectionDiff_dps; // Subtract yaw correction to turn right
    float speedSetpointRight_dps = baseSpeed_dps + commandedTurnDiff_dps + yawCorrectionDiff_dps; // Add yaw correction to turn right

    // --- Clamp Final Speed Setpoints ---
    // Use angle PID limits as the absolute maximum wheel speed
    speedSetpointLeft_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointLeft_dps));
    speedSetpointRight_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointRight_dps));

    // Store clamped setpoints for telemetry
    m_last_speed_setpoint_left_dps = speedSetpointLeft_dps;
    m_last_speed_setpoint_right_dps = speedSetpointRight_dps;

    // --- Compute Motor Effort using Speed PIDs ---
    effort.left = m_speedPidLeft.compute(speedSetpointLeft_dps, currentSpeedLeft_dps, dt);
    effort.right = m_speedPidRight.compute(speedSetpointRight_dps, currentSpeedRight_dps, dt);

    // Final clamp on effort
    effort.left = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.left));
    effort.right = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.right));

    // Update Log Message (added YawRate, YawErr, YawCorr)
    ESP_LOGV(TAG, "P:%.1f|TgtP:%.1f|Yaw:%.1f|TgtAV:%.1f|YawE:%.1f|YawC:%.1f|CmdDiff:%.1f|LSet:%.1f RSet:%.1f|LCur:%.1f RCur:%.1f|LEff:%.2f REff:%.2f",
             currentPitch_deg, targetPitchOffset_deg, currentYawRate_dps, targetAngVel_dps, yawRateError_dps, yawCorrectionDiff_dps, commandedTurnDiff_dps,
             speedSetpointLeft_dps, speedSetpointRight_dps,
             currentSpeedLeft_dps, currentSpeedRight_dps,
             effort.left, effort.right);

    return effort;
}

void BalancingAlgorithm::handleConfigUpdate(const BaseEvent& event) {
     ESP_LOGI(TAG, "Handling config update event.");
     PIDConfig angleConf = m_configService.getAnglePidConfig();
     m_anglePid.updateParams(angleConf);
     m_speedPidLeft.updateParams(m_configService.getSpeedPidLeftConfig());
     m_speedPidRight.updateParams(m_configService.getSpeedPidRightConfig());
     m_yawRatePid.updateParams(m_configService.getYawRatePidConfig()); // <<< Update Yaw Rate PID

     m_angle_pid_output_min = angleConf.getOutputMin();
     m_angle_pid_output_max = angleConf.getOutputMax();
     ESP_LOGI(TAG, "Stored Angle PID limits: Min=%.1f, Max=%.1f", m_angle_pid_output_min, m_angle_pid_output_max);

     m_wheel_radius_m = m_configService.getConfigData().encoder.wheel_diameter_mm / 2000.0f;
     // m_robot_wheelbase_m = ... // Load from config if added
     ESP_LOGI(TAG, "Internal params updated. Wheel Radius: %.4f m, Wheelbase: %.4f m", m_wheel_radius_m, m_robot_wheelbase_m);
}