#include "RobotController.hpp"

#include "OrientationEstimator.hpp"
#include "BalancingAlgorithm.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BatteryService.hpp"
#include "StateManager.hpp"
#include "SystemState.hpp"
#include "PidTuningService.hpp"
#include "GuidedCalibrationService.hpp"
#include "ControlEventDispatcher.hpp"
#include "MOTION_TargetMovement.hpp"
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
    PidTuningService& pidTuningService,
    GuidedCalibrationService& guidedCalibrationService,
    ControlEventDispatcher& controlEventDispatcher
) :
    m_estimator(estimator),
    m_encoderService(encoderService),
    m_motorService(motorService),
    m_algorithm(algorithm),
    m_stateManager(stateManager),
    m_batteryService(batteryService),
    m_pidTuningService(pidTuningService),
    m_guidedCalibrationService(guidedCalibrationService),
    m_controlEventDispatcher(controlEventDispatcher),
    m_latestTargetPitchOffset_deg(0.0f),
    m_latestTargetAngVel_dps(0.0f)
{
    ESP_LOGI(TAG, "RobotController constructed.");
}

// EventHandler implementation
void RobotController::handleEvent(const BaseEvent& event) {
    if (event.is<MOTION_TargetMovement>()) {
        handleTargetMovementCommand(event.as<MOTION_TargetMovement>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
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
    const int64_t startTimeMicros = esp_timer_get_time();

    const SystemState currentState = m_stateManager.getCurrentState();
    if (currentState == SystemState::FATAL_ERROR || currentState == SystemState::INIT || currentState == SystemState::SHUTDOWN) {
        stopControlLoop();
        ESP_LOGV(TAG, "Skipping control step due to state: %d", static_cast<int>(currentState));
        return;
    }

    const auto orientation = m_estimator->getPitchAndYawRate();
    const float pitch_deg = orientation.first;
    const float pitch_rate_dps = 0.0f;
    const float yaw_rate_dps = orientation.second;

    m_controlEventDispatcher.enqueueOrientation(
        pitch_deg * OrientationEstimator::DEG_TO_RAD,
        pitch_rate_dps * OrientationEstimator::DEG_TO_RAD);

    m_encoderService.update(dt);
    const float speedL_dps = m_encoderService.getLeftSpeedDegPerSec();
    const float speedR_dps = m_encoderService.getRightSpeedDegPerSec();

    float currentTargetPitchOffset_deg;
    float currentTargetAngVel_dps;
    {
        std::lock_guard<std::mutex> lock(m_target_values_mutex);
        currentTargetPitchOffset_deg = m_latestTargetPitchOffset_deg;
        currentTargetAngVel_dps = m_latestTargetAngVel_dps;
    }

    const MotorEffort effort = executeControlMode(currentState,
                                                  dt,
                                                  pitch_deg,
                                                  pitch_rate_dps,
                                                  yaw_rate_dps,
                                                  speedL_dps,
                                                  speedR_dps,
                                                  currentTargetPitchOffset_deg,
                                                  currentTargetAngVel_dps);

    m_motorService.setMotorEffort(effort.left, effort.right);

    const TelemetryDataPoint snapshot = buildTelemetrySnapshot(startTimeMicros,
                                                               currentState,
                                                               pitch_deg,
                                                               yaw_rate_dps,
                                                               speedL_dps,
                                                               speedR_dps,
                                                               currentTargetPitchOffset_deg);
    m_controlEventDispatcher.enqueueTelemetry(snapshot);

    ESP_LOGV(TAG, "Ctrl Step: dt=%.4f, P=%.1f YawR=%.1f | TgtPO=%.1f, TgtAV=%.1f | SSetL=%.1f, SSetR=%.1f | SActL=%.1f, SActR=%.1f | EffL=%.2f, EffR=%.2f",
        dt, pitch_deg, yaw_rate_dps, currentTargetPitchOffset_deg, currentTargetAngVel_dps,
        snapshot.speedSetpointLeft_dps, snapshot.speedSetpointRight_dps,
        speedL_dps, speedR_dps, effort.left, effort.right);
}

void RobotController::stopControlLoop() {
    m_motorService.setMotorEffort(0.0f, 0.0f);
    m_algorithm.resetState();
}

MotorEffort RobotController::executeControlMode(SystemState currentState,
                                                float dt,
                                                float pitch_deg,
                                                float pitch_rate_dps,
                                                float yaw_rate_dps,
                                                float speedL_dps,
                                                float speedR_dps,
                                                float& currentTargetPitchOffset_deg,
                                                float& currentTargetAngVel_dps) {
    switch (currentState) {
        case SystemState::BALANCING:
            return m_algorithm.update(dt,
                                      pitch_deg,
                                      pitch_rate_dps,
                                      yaw_rate_dps,
                                      speedL_dps,
                                      speedR_dps,
                                      currentTargetPitchOffset_deg,
                                      currentTargetAngVel_dps);

        case SystemState::PID_TUNING:
            m_algorithm.resetState();
            currentTargetPitchOffset_deg = 0.0f;
            currentTargetAngVel_dps = 0.0f;
            return m_pidTuningService.update(dt, speedL_dps, speedR_dps);

        case SystemState::GUIDED_CALIBRATION:
            m_algorithm.resetState();
            currentTargetPitchOffset_deg = 0.0f;
            currentTargetAngVel_dps = 0.0f;
            return executeGuidedCalibrationMode(dt, pitch_deg, speedL_dps, speedR_dps);

        default:
            stopControlLoop();
            currentTargetPitchOffset_deg = 0.0f;
            currentTargetAngVel_dps = 0.0f;
            return {0.0f, 0.0f};
    }
}

MotorEffort RobotController::executeGuidedCalibrationMode(float dt,
                                                          float pitch_deg,
                                                          float speedL_dps,
                                                          float speedR_dps) const {
    GuidedCalibrationSample guidedSample = {};
    guidedSample.pitch_deg = pitch_deg;
    guidedSample.speedLeft_dps = speedL_dps;
    guidedSample.speedRight_dps = speedR_dps;
    return m_guidedCalibrationService.update(dt, guidedSample);
}

TelemetryDataPoint RobotController::buildTelemetrySnapshot(int64_t timestamp_us,
                                                           SystemState currentState,
                                                           float pitch_deg,
                                                           float yaw_rate_dps,
                                                           float speedL_dps,
                                                           float speedR_dps,
                                                           float currentTargetPitchOffset_deg) const {
    TelemetryDataPoint snapshot = {};
    snapshot.timestamp_us = timestamp_us;
    snapshot.pitch_deg = pitch_deg;
    snapshot.speedLeft_dps = speedL_dps;
    snapshot.speedRight_dps = speedR_dps;
    snapshot.batteryVoltage = m_batteryService.getLatestStatus().voltage;
    snapshot.systemState = static_cast<int>(currentState);
    snapshot.desiredAngle_deg = currentTargetPitchOffset_deg;
    snapshot.yawRate_dps = yaw_rate_dps;

    switch (currentState) {
        case SystemState::PID_TUNING:
            snapshot.speedSetpointLeft_dps = m_pidTuningService.getLastSpeedSetpointLeftDPS();
            snapshot.speedSetpointRight_dps = m_pidTuningService.getLastSpeedSetpointRightDPS();
            break;

        case SystemState::GUIDED_CALIBRATION:
            snapshot.speedSetpointLeft_dps = 0.0f;
            snapshot.speedSetpointRight_dps = 0.0f;
            break;

        default:
            snapshot.speedSetpointLeft_dps = m_algorithm.getLastSpeedSetpointLeftDPS();
            snapshot.speedSetpointRight_dps = m_algorithm.getLastSpeedSetpointRightDPS();
            break;
    }

    return snapshot;
}
