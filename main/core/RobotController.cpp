// main/core/RobotController.cpp
#include "RobotController.hpp"

// --- Include full definitions for classes used ---
#include "OrientationEstimator.hpp"
#include "BalancingAlgorithm.hpp"
#include "FallDetector.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BatteryService.hpp"
#include "StateManager.hpp"
#include "SystemState.hpp"
#include "WebServer.hpp"
#include "TelemetryDataPoint.hpp"
#include "BatteryStatusUpdatedEvent.hpp" // Should be included if used, though not directly here
#include "CommandProcessor.hpp"     // <<<--- Already included previously

#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>


RobotController::RobotController(
    OrientationEstimator& estimator,
    EncoderService& encoderService,
    MotorService& motorService,
    BalancingAlgorithm& algorithm,
    StateManager& stateManager,
    FallDetector& fallDetector,
    WebServer& webServer,
    BatteryService& batteryService,
    CommandProcessor& commandProcessor // <<<--- Already added previously
) : // Use initializer list
    m_estimator(estimator),
    m_encoderService(encoderService),
    m_motorService(motorService),
    m_algorithm(algorithm),
    m_stateManager(stateManager),
    m_fallDetector(fallDetector),
    m_webServer(webServer),
    m_batteryService(batteryService),
    m_commandProcessor(commandProcessor) // <<<--- Already added previously
{
    ESP_LOGI(TAG, "RobotController initialized.");
}

void RobotController::runControlStep(float dt) {
    int64_t startTimeMicros = esp_timer_get_time();

    // --- Check State ---
    SystemState currentState = m_stateManager.getCurrentState();
    if (currentState == SystemState::IMU_RECOVERY || currentState == SystemState::FATAL_ERROR) {
        m_motorService.setMotorEffort(0.0f, 0.0f); // Ensure motors are off
        m_algorithm.resetState(); // Reset algorithm state
        ESP_LOGV(TAG, "Skipping control step due to state: %d", static_cast<int>(currentState));
        return; // Skip control logic in these states
    }

    // 1. Get Orientation Data (Only if not in recovery/error)
    float pitch_deg = m_estimator.getPitchDeg();
    float pitch_rate_dps = 0.0f; // Placeholder - NEEDS IMPLEMENTATION in OrientationEstimator

    // 2. Check for Fall & Update State
    // currentState = m_stateManager.getCurrentState(); // Already fetched above
    if (currentState == SystemState::BALANCING && m_stateManager.isFallDetectionEnabled()) {
         m_fallDetector.check(pitch_deg * OrientationEstimator::DEG_TO_RAD);
    }
    currentState = m_stateManager.getCurrentState(); // Re-fetch state in case fall detector changed it

    // 3. Update Encoder Speeds
    m_encoderService.update(dt);
    float speedL_dps = m_encoderService.getLeftSpeedDegPerSec();
    float speedR_dps = m_encoderService.getRightSpeedDegPerSec();

    // --- 4. Get Target Velocities from CommandProcessor ---
    // Getters now implicitly check for input timeout via checkInputTimeout()
    float targetLinVel_mps = m_commandProcessor.getTargetLinearVelocity();
    float targetAngVel_dps = m_commandProcessor.getTargetAngularVelocity();
    // --- END Get Targets ---

    // 5. Run Balancing Algorithm
    MotorEffort effort = {0.0f, 0.0f};
    if (currentState == SystemState::BALANCING) {
         // Pass pitch_rate_dps once available
         effort = m_algorithm.update(dt, pitch_deg, pitch_rate_dps, /* <--- Needs rate */
                                    speedL_dps, speedR_dps,
                                    targetLinVel_mps, targetAngVel_dps); // Pass fetched targets
    } else {
        m_algorithm.resetState();
    }

    // 6. Set Motor Effort
    m_motorService.setMotorEffort(effort.left, effort.right);

    // 7. Update Telemetry Buffer
    TelemetryDataPoint snapshot = {};
    snapshot.timestamp_us = startTimeMicros;
    snapshot.pitch_deg = pitch_deg;
    snapshot.balanceSpeed_dps = m_algorithm.getLastBalancingSpeedDPS();
    snapshot.targetAngVel_dps = targetAngVel_dps; // Store target DPS
    snapshot.speedLeft_dps = speedL_dps;
    snapshot.speedRight_dps = speedR_dps;
    snapshot.effortLeft = effort.left;
    snapshot.effortRight = effort.right;
    snapshot.batteryVoltage = m_batteryService.getLatestStatus().voltage;
    snapshot.systemState = static_cast<int>(currentState);
    m_webServer.addTelemetrySnapshot(snapshot);

    ESP_LOGV(TAG, "Ctrl Step: dt=%.4f, P=%.1f, SL=%.1f, SR=%.1f | TL=%.2f, TA=%.1f | EL=%.2f, ER=%.2f",
        dt, pitch_deg, speedL_dps, speedR_dps, targetLinVel_mps, targetAngVel_dps, effort.left, effort.right);
}