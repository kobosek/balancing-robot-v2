#include "RobotController.hpp"

#include "OrientationEstimator.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BatteryService.hpp"
#include "ControlModeExecutor.hpp"
#include "ControlEventDispatcher.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "MOTION_TargetMovement.hpp"
#include "TelemetryDataPoint.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

RobotController::RobotController(
    std::shared_ptr<OrientationEstimator> estimator,
    EncoderService& encoderService,
    MotorService& motorService,
    BatteryService& batteryService,
    ControlModeExecutor& controlModeExecutor,
    ControlEventDispatcher& controlEventDispatcher
) :
    m_estimator(estimator),
    m_encoderService(encoderService),
    m_motorService(motorService),
    m_batteryService(batteryService),
    m_controlModeExecutor(controlModeExecutor),
    m_controlEventDispatcher(controlEventDispatcher),
    m_latestTargetPitchOffset_deg(0.0f),
    m_latestTargetAngVel_dps(0.0f),
    m_controlMode(ControlRunMode::DISABLED),
    m_telemetryStateCode(0),
    m_telemetryEnabled(false)
{
    ESP_LOGI(TAG, "RobotController constructed.");
}

// EventHandler implementation
void RobotController::handleEvent(const BaseEvent& event) {
    if (event.is<MOTION_TargetMovement>()) {
        handleTargetMovementCommand(event.as<MOTION_TargetMovement>());
    } else if (event.is<CONTROL_RunModeChanged>()) {
        handleControlRunModeChanged(event.as<CONTROL_RunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}


void RobotController::handleTargetMovementCommand(const MOTION_TargetMovement& event) {
    m_latestTargetPitchOffset_deg.store(event.targetPitchOffset_deg, std::memory_order_relaxed);
    m_latestTargetAngVel_dps.store(event.targetAngularVelocity_dps, std::memory_order_relaxed);
    ESP_LOGV(TAG, "RC Handler: Updated targets: PitchOffset=%.2f, AngVel=%.2f", event.targetPitchOffset_deg, event.targetAngularVelocity_dps);
}

void RobotController::handleControlRunModeChanged(const CONTROL_RunModeChanged& event) {
    m_telemetryStateCode.store(event.telemetryStateCode, std::memory_order_relaxed);
    m_telemetryEnabled.store(event.telemetryEnabled, std::memory_order_relaxed);
    m_controlMode.store(event.mode, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Control run mode changed to %d", static_cast<int>(event.mode));
}

void RobotController::runControlStep(float dt) {
    const int64_t startTimeMicros = esp_timer_get_time();

    const bool telemetryEnabled = m_telemetryEnabled.load(std::memory_order_relaxed);
    const int telemetryStateCode = m_telemetryStateCode.load(std::memory_order_relaxed);
    const ControlRunMode currentMode = m_controlMode.load(std::memory_order_relaxed);

    if (!telemetryEnabled) {
        stopControlLoop();
        ESP_LOGV(TAG, "Skipping control step because telemetry is disabled");
        return;
    }

    const OrientationEstimate orientation = m_estimator->getOrientation();
    const float pitch_deg = orientation.pitch_deg;
    const float pitch_rate_dps = orientation.pitch_rate_dps;
    const float yaw_deg = orientation.yaw_deg;
    const float yaw_rate_dps = orientation.yaw_rate_dps;

    m_controlEventDispatcher.enqueueOrientation(
        pitch_deg * OrientationEstimator::DEG_TO_RAD,
        pitch_rate_dps * OrientationEstimator::DEG_TO_RAD);

    m_encoderService.update(dt);
    const float speedL_dps = m_encoderService.getLeftSpeedDegPerSec();
    const float speedR_dps = m_encoderService.getRightSpeedDegPerSec();

    const float currentTargetPitchOffset_deg = m_latestTargetPitchOffset_deg.load(std::memory_order_relaxed);
    const float currentTargetAngVel_dps = m_latestTargetAngVel_dps.load(std::memory_order_relaxed);

    ControlModeInput modeInput = {};
    modeInput.mode = currentMode;
    modeInput.dt = dt;
    modeInput.pitch_deg = pitch_deg;
    modeInput.pitch_rate_dps = pitch_rate_dps;
    modeInput.yaw_deg = yaw_deg;
    modeInput.yaw_rate_dps = yaw_rate_dps;
    modeInput.speedLeft_dps = speedL_dps;
    modeInput.speedRight_dps = speedR_dps;
    modeInput.targetPitchOffset_deg = currentTargetPitchOffset_deg;
    modeInput.targetAngularVelocity_dps = currentTargetAngVel_dps;

    const ControlModeResult modeResult = m_controlModeExecutor.execute(modeInput);

    m_motorService.setMotorEffort(modeResult.effort.left, modeResult.effort.right);

    const TelemetryDataPoint snapshot = buildTelemetrySnapshot(startTimeMicros,
                                                               telemetryStateCode,
                                                               pitch_deg,
                                                               yaw_deg,
                                                               yaw_rate_dps,
                                                               speedL_dps,
                                                               speedR_dps,
                                                               modeResult);
    m_controlEventDispatcher.enqueueTelemetry(snapshot);

    ESP_LOGV(TAG, "Ctrl Step: dt=%.4f, P=%.1f Yaw=%.1f YawR=%.1f | TgtPO=%.1f, CmdYawR=%.1f, TgtYaw=%.1f, DesYawR=%.1f | SSetL=%.1f, SSetR=%.1f | SActL=%.1f, SActR=%.1f | EffL=%.2f, EffR=%.2f",
        dt, pitch_deg, yaw_deg, yaw_rate_dps, modeResult.telemetryTargetPitchOffset_deg, modeResult.telemetryTargetAngularVelocity_dps,
        modeResult.telemetryTargetYaw_deg, modeResult.telemetryDesiredYawRate_dps,
        snapshot.speedSetpointLeft_dps, snapshot.speedSetpointRight_dps,
        speedL_dps, speedR_dps, modeResult.effort.left, modeResult.effort.right);
}

void RobotController::stopControlLoop() {
    m_motorService.setMotorEffort(0.0f, 0.0f);
    m_controlModeExecutor.reset();
}

TelemetryDataPoint RobotController::buildTelemetrySnapshot(int64_t timestamp_us,
                                                           int telemetryStateCode,
                                                           float pitch_deg,
                                                           float yaw_deg,
                                                           float yaw_rate_dps,
                                                           float speedL_dps,
                                                           float speedR_dps,
                                                           const ControlModeResult& modeResult) const {
    TelemetryDataPoint snapshot = {};
    snapshot.timestamp_us = timestamp_us;
    snapshot.pitch_deg = pitch_deg;
    snapshot.speedLeft_dps = speedL_dps;
    snapshot.speedRight_dps = speedR_dps;
    snapshot.batteryVoltage = m_batteryService.getLatestStatus().voltage;
    snapshot.systemState = telemetryStateCode;
    snapshot.desiredAngle_deg = modeResult.telemetryTargetPitchOffset_deg;
    snapshot.yawAngle_deg = yaw_deg;
    snapshot.targetYawAngle_deg = modeResult.telemetryTargetYaw_deg;
    snapshot.yawRate_dps = yaw_rate_dps;
    snapshot.targetYawRate_dps = modeResult.telemetryDesiredYawRate_dps;
    snapshot.speedSetpointLeft_dps = modeResult.speedSetpointLeft_dps;
    snapshot.speedSetpointRight_dps = modeResult.speedSetpointRight_dps;

    return snapshot;
}
