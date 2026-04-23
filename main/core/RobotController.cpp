#include "RobotController.hpp"

#include "OrientationEstimator.hpp"
#include "BalancingAlgorithm.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BatteryService.hpp"
#include "StateManager.hpp"
#include "SystemState.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "CommandProcessor.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "MOTION_TargetMovement.hpp"
#include "IMU_OrientationData.hpp"
#include "TELEMETRY_Snapshot.hpp"
#include "TelemetryDataPoint.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

RobotController::RobotController(
    std::shared_ptr<OrientationEstimator> estimator,
    EncoderService& encoderService,
    MotorService& motorService,
    BalancingAlgorithm& algorithm,
    StateManager& stateManager,
    BatteryService& batteryService,
    CommandProcessor& commandProcessor
) :
    m_estimator(estimator),
    m_encoderService(encoderService),
    m_motorService(motorService),
    m_algorithm(algorithm),
    m_stateManager(stateManager),
    m_batteryService(batteryService),
    m_commandProcessor(commandProcessor),
    m_latestTargetPitchOffset_deg(0.0f),
    m_latestTargetAngVel_dps(0.0f)
{
    ESP_LOGI(TAG, "RobotController constructed.");
}

esp_err_t RobotController::init(EventBus& bus) {
    ESP_LOGI(TAG, "Initializing RobotController...");
    m_eventBus = &bus;
    ESP_LOGI(TAG, "RobotController initialized.");
    return ESP_OK;
}

// EventHandler implementation
void RobotController::handleEvent(const BaseEvent& event) {
    // Central event handler that dispatches to specific handlers based on event type
    switch (event.type) {
        case EventType::MOTION_TARGET_SET:
            handleTargetMovementCommand(static_cast<const MOTION_TargetMovement&>(event));
            break;
            
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}


void RobotController::handleTargetMovementCommand(const MOTION_TargetMovement& event) {
    { // Lock scope
        std::lock_guard<std::mutex> lock(m_target_values_mutex);
        m_latestTargetPitchOffset_deg = event.targetPitchOffset_deg;
        m_latestTargetAngVel_dps = event.targetAngularVelocity_dps;
    }
    ESP_LOGV(TAG, "RC Handler: Updated targets: PitchOffset=%.2f, AngVel=%.2f", event.targetPitchOffset_deg, event.targetAngularVelocity_dps);
}

void RobotController::runControlStep(float dt) {
    int64_t startTimeMicros = esp_timer_get_time();

    // Check State
    SystemState currentState = m_stateManager.getCurrentState();
    if (currentState == SystemState::FATAL_ERROR || currentState == SystemState::INIT || currentState == SystemState::SHUTDOWN) {
        m_motorService.setMotorEffort(0.0f, 0.0f);
        m_algorithm.resetState();
        ESP_LOGV(TAG, "Skipping control step due to state: %d", static_cast<int>(currentState));
        return;
    }

    // 1. Get Orientation Data
    const auto orientation = m_estimator->getPitchAndYawRate();
    float pitch_deg = orientation.first;
    float pitch_rate_dps = 0.0f;
    float yaw_rate_dps = orientation.second;

    // Publish orientation data — BalanceMonitor handles fall detection and recovery as an observer
    if (m_eventBus) {
        IMU_OrientationData orientation_event(pitch_deg * OrientationEstimator::DEG_TO_RAD, pitch_rate_dps * OrientationEstimator::DEG_TO_RAD);
        m_eventBus->publish(orientation_event);
    }

    currentState = m_stateManager.getCurrentState(); // Re-fetch after observers may have changed state

    // 2. Update Encoder Speeds
    m_encoderService.update(dt);
    float speedL_dps = m_encoderService.getLeftSpeedDegPerSec();
    float speedR_dps = m_encoderService.getRightSpeedDegPerSec();

    // 3. Get Target Control Values
    float currentTargetPitchOffset_deg;
    float currentTargetAngVel_dps;
    { // Lock scope
        std::lock_guard<std::mutex> lock(m_target_values_mutex);
        currentTargetPitchOffset_deg = m_latestTargetPitchOffset_deg;
        currentTargetAngVel_dps = m_latestTargetAngVel_dps;
    }

    // 4. Run Balancing Algorithm
    MotorEffort effort = {0.0f, 0.0f};
    if (currentState == SystemState::BALANCING) {
         effort = m_algorithm.update(dt, pitch_deg, pitch_rate_dps, yaw_rate_dps,
                                    speedL_dps, speedR_dps,
                                    currentTargetPitchOffset_deg,
                                    currentTargetAngVel_dps);
    } else {
        m_algorithm.resetState();
        effort = {0.0f, 0.0f};
        currentTargetPitchOffset_deg = 0.0f;
        currentTargetAngVel_dps = 0.0f;
    }

    // 5. Set Motor Effort
    m_motorService.setMotorEffort(effort.left, effort.right);

    // 6. Update Telemetry Buffer
    TelemetryDataPoint snapshot = {};
    snapshot.timestamp_us = startTimeMicros;
    snapshot.pitch_deg = pitch_deg;
    snapshot.speedLeft_dps = speedL_dps;
    snapshot.speedRight_dps = speedR_dps;
    snapshot.batteryVoltage = m_batteryService.getLatestStatus().voltage;
    snapshot.systemState = static_cast<int>(currentState);
    snapshot.speedSetpointLeft_dps = m_algorithm.getLastSpeedSetpointLeftDPS();
    snapshot.speedSetpointRight_dps = m_algorithm.getLastSpeedSetpointRightDPS();
    snapshot.desiredAngle_deg = currentTargetPitchOffset_deg;
    snapshot.yawRate_dps = yaw_rate_dps;

    m_eventBus->publish(TELEMETRY_Snapshot(snapshot));

    // Update Log (added Yaw)
    ESP_LOGV(TAG, "Ctrl Step: dt=%.4f, P=%.1f YawR=%.1f | TgtPO=%.1f, TgtAV=%.1f | SSetL=%.1f, SSetR=%.1f | SActL=%.1f, SActR=%.1f | EffL=%.2f, EffR=%.2f",
        dt, pitch_deg, yaw_rate_dps, currentTargetPitchOffset_deg, currentTargetAngVel_dps,
        snapshot.speedSetpointLeft_dps, snapshot.speedSetpointRight_dps,
        speedL_dps, speedR_dps, effort.left, effort.right);
}
