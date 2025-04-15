#include "RobotController.hpp"

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
#include "BatteryStatusUpdatedEvent.hpp"
#include "CommandProcessor.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "TargetMovementCommand.hpp"
#include "OrientationDataEvent.hpp"

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
    CommandProcessor& commandProcessor
) :
    m_estimator(estimator),
    m_encoderService(encoderService),
    m_motorService(motorService),
    m_algorithm(algorithm),
    m_stateManager(stateManager),
    m_fallDetector(fallDetector),
    m_webServer(webServer),
    m_batteryService(batteryService),
    m_commandProcessor(commandProcessor),
    m_latestTargetPitchOffset_deg(0.0f),
    m_latestTargetAngVel_dps(0.0f)
{
    ESP_LOGI(TAG, "RobotController constructed.");
}

esp_err_t RobotController::init(EventBus& bus) {
    ESP_LOGI(TAG, "Initializing RobotController subscriptions...");
    m_eventBus = &bus; // Store reference to event bus
    bus.subscribe(EventType::TARGET_MOVEMENT_CMD_SET,
        [this](const BaseEvent& ev) {
            this->handleTargetMovementCommand(ev);
        }
    );
    ESP_LOGI(TAG, "Subscribed to TARGET_MOVEMENT_CMD_SET events.");
    return ESP_OK;
}

void RobotController::handleTargetMovementCommand(const BaseEvent& event) {
    if (event.type == EventType::TARGET_MOVEMENT_CMD_SET) {
        const auto& cmd = static_cast<const TargetMovementCommand&>(event);
        { // Lock scope
            std::lock_guard<std::mutex> lock(m_target_values_mutex);
            m_latestTargetPitchOffset_deg = cmd.targetPitchOffset_deg;
            m_latestTargetAngVel_dps = cmd.targetAngularVelocity_dps;
        }
        ESP_LOGV(TAG, "RC Handler: Updated targets: PitchOffset=%.2f, AngVel=%.2f", cmd.targetPitchOffset_deg, cmd.targetAngularVelocity_dps);
    }
}

void RobotController::runControlStep(float dt) {
    int64_t startTimeMicros = esp_timer_get_time();

    // Check State
    SystemState currentState = m_stateManager.getCurrentState();
    if (currentState == SystemState::IMU_RECOVERY || currentState == SystemState::FATAL_ERROR || currentState == SystemState::INIT) {
        m_motorService.setMotorEffort(0.0f, 0.0f);
        m_algorithm.resetState();
        ESP_LOGV(TAG, "Skipping control step due to state: %d", static_cast<int>(currentState));
        return;
    }

    // 1. Get Orientation Data
    float pitch_deg = m_estimator.getPitchDeg();
    float pitch_rate_dps = 0.0f; // Placeholder
    float yaw_rate_dps = m_estimator.getYawRateDPS(); // <<< GET YAW RATE
    
    // Publish orientation data event for auto recovery and other systems
    if (m_eventBus) {
        OrientationDataEvent orientation_event(pitch_deg * OrientationEstimator::DEG_TO_RAD, pitch_rate_dps * OrientationEstimator::DEG_TO_RAD);
        m_eventBus->publish(orientation_event);
    }

    // 2. Check for Fall & Update State
    if (currentState == SystemState::BALANCING && m_stateManager.isFallDetectionEnabled()) {
         m_fallDetector.check(pitch_deg * OrientationEstimator::DEG_TO_RAD);
    }
    currentState = m_stateManager.getCurrentState(); // Re-fetch

    // 3. Update Encoder Speeds
    m_encoderService.update(dt);
    float speedL_dps = m_encoderService.getLeftSpeedDegPerSec();
    float speedR_dps = m_encoderService.getRightSpeedDegPerSec();

    // 4. Get Target Control Values
    float currentTargetPitchOffset_deg;
    float currentTargetAngVel_dps;
    { // Lock scope
        std::lock_guard<std::mutex> lock(m_target_values_mutex);
        currentTargetPitchOffset_deg = m_latestTargetPitchOffset_deg;
        currentTargetAngVel_dps = m_latestTargetAngVel_dps;
    }

    // 5. Run Balancing Algorithm
    MotorEffort effort = {0.0f, 0.0f};
    if (currentState == SystemState::BALANCING) {
         // <<< Pass yaw_rate_dps to algorithm >>>
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

    // 6. Set Motor Effort
    m_motorService.setMotorEffort(effort.left, effort.right);

    // 7. Update Telemetry Buffer
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
    snapshot.yawRate_dps = yaw_rate_dps; // <<< ADD YAW RATE TO TELEMETRY
    m_webServer.addTelemetrySnapshot(snapshot);

    // Update Log (added Yaw)
    ESP_LOGV(TAG, "Ctrl Step: dt=%.4f, P=%.1f YawR=%.1f | TgtPO=%.1f, TgtAV=%.1f | SSetL=%.1f, SSetR=%.1f | SActL=%.1f, SActR=%.1f | EffL=%.2f, EffR=%.2f",
        dt, pitch_deg, yaw_rate_dps, currentTargetPitchOffset_deg, currentTargetAngVel_dps,
        snapshot.speedSetpointLeft_dps, snapshot.speedSetpointRight_dps,
        speedL_dps, speedR_dps, effort.left, effort.right);
}