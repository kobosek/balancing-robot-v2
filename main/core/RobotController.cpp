#include "RobotController.hpp"

#include "OrientationEstimator.hpp"
#include "CONFIG_BehaviorConfigUpdate.hpp"
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
#include <algorithm>

RobotController::RobotController(
    std::shared_ptr<OrientationEstimator> estimator,
    EncoderService& encoderService,
    MotorService& motorService,
    BatteryService& batteryService,
    ControlModeExecutor& controlModeExecutor,
    ControlEventDispatcher& controlEventDispatcher,
    const SystemBehaviorConfig& behavior,
    const EncoderConfig& encoderConfig
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
    m_telemetryEnabled(false),
    m_longitudinalOdometry(encoderConfig, behavior.imu_max_sample_age_ms * 1000LL)
{
    m_maxSampleAgeUs = behavior.imu_max_sample_age_ms * 1000LL;
    ESP_LOGI(TAG, "RobotController constructed.");
}

// EventHandler implementation
void RobotController::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_BehaviorConfigUpdate>()) {
        m_maxSampleAgeUs = event.as<CONFIG_BehaviorConfigUpdate>().config.imu_max_sample_age_ms * 1000LL;
    } else if (event.is<MOTION_TargetMovement>()) {
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
    std::lock_guard<std::mutex> lock(m_modeMutex);
    if (event.armId < m_armId) return;
    m_armId = event.armId;
    m_generation = event.generation;
    m_telemetryStateCode.store(event.telemetryStateCode, std::memory_order_relaxed);
    m_telemetryEnabled.store(event.telemetryEnabled, std::memory_order_relaxed);
    m_controlMode.store(event.mode, std::memory_order_relaxed);
    // This mutex is shared with the control task. Never log while holding it.
}

void RobotController::runControlStep(float dt) {
    const int64_t startTimeMicros = esp_timer_get_time();

    bool telemetryEnabled;
    int telemetryStateCode;
    ControlRunMode currentMode;
    uint64_t arm;
    uint32_t generation;
    {
        std::lock_guard<std::mutex> lock(m_modeMutex);
        telemetryEnabled = m_telemetryEnabled;
        telemetryStateCode = m_telemetryStateCode;
        currentMode = m_controlMode;
        arm = m_armId;
        generation = m_generation;
    }

    // Keep PCNT maintenance running even when telemetry/control is disabled;
    // an accumulated counter must not be left unchecked indefinitely.
    m_encoderService.update();
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

    m_controlEventDispatcher.enqueueOrientation(orientation);

    const auto encoders = m_encoderService.getFrame();
    if (!m_hasOdometryArm || arm != m_lastOdometryArm) {
        m_longitudinalOdometry.reset();
        m_lastOdometryArm = arm;
        m_hasOdometryArm = true;
    }
    m_longitudinalOdometry.setMaxSampleAgeUs(m_maxSampleAgeUs.load(std::memory_order_relaxed));
    const auto odometry = m_longitudinalOdometry.update(encoders, esp_timer_get_time());
    const float speedL_dps = encoders.left.speedDps;
    const float speedR_dps = encoders.right.speedDps;
    const bool reusedImuSample = orientation.sample_sequence == m_lastImuSequence &&
        orientation.generation == m_lastImuGeneration;
    m_lastImuSequence = orientation.sample_sequence;
    m_lastImuGeneration = orientation.generation;

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
    modeInput.odometry = odometry;

    const auto fault = [&](const char* cause, esp_err_t error = ESP_OK) {
        const auto observed = esp_timer_get_time();
        const auto latest = m_estimator->getOrientation();
        m_motorService.inhibitImu(arm);
        m_controlModeExecutor.reset();
        if (m_lastFaultArm != arm) {
            m_lastFaultArm = arm;
            m_controlEventDispatcher.latchImuFault({arm, generation, observed, IMUFaultReason::STALE,
                orientation.sample_timestamp_us, latest.sample_timestamp_us, cause, error});
        }
    };
    const bool active = currentMode != ControlRunMode::DISABLED;
    const auto inputsValid = [&] {
        return orientation.fresh(esp_timer_get_time(), m_maxSampleAgeUs.load()) &&
            orientation.generation == generation && encoders.left.valid && encoders.right.valid &&
            esp_timer_get_time() - encoders.left.sampleTimestampUs <= m_maxSampleAgeUs.load() &&
            esp_timer_get_time() - encoders.right.sampleTimestampUs <= m_maxSampleAgeUs.load() && std::isfinite(dt) && dt > 0 &&
            std::isfinite(speedL_dps) && std::isfinite(speedR_dps) &&
            std::isfinite(currentTargetPitchOffset_deg) && std::isfinite(currentTargetAngVel_dps);
    };
    ControlModeResult modeResult{};
    if (active && (!inputsValid() || !m_motorService.isArmAllowed(arm, generation))) {
        fault(!encoders.left.valid || !encoders.right.valid ? "invalid-encoder" : inputsValid() ? "revoked-arm" : "invalid-input");
    } else {
        if (arm != m_lastExecutedArm) { m_controlModeExecutor.reset(); m_lastExecutedArm = arm; }
        modeResult = m_controlModeExecutor.execute(modeInput);
        const auto latest = m_estimator->getOrientation();
        if (active && (!inputsValid() || !latest.valid || latest.generation != generation ||
            !std::isfinite(modeResult.effort.left) || !std::isfinite(modeResult.effort.right))) {
            fault("changed-during-step");
            modeResult = {};
        } else {
            const auto result = m_motorService.setMotorEffort(modeResult.effort.left, modeResult.effort.right, arm, generation,
                active ? std::min(orientation.sample_timestamp_us,
                    std::min(encoders.left.sampleTimestampUs, encoders.right.sampleTimestampUs)) : 0,
                active ? m_maxSampleAgeUs.load() : 0);
            if (active && result != ESP_OK) { fault("motor-commit", result); modeResult = {}; }
        }
    }

    TelemetryDataPoint snapshot = buildTelemetrySnapshot(startTimeMicros,
                                                               telemetryStateCode,
                                                               pitch_deg,
                                                               yaw_deg,
                                                               yaw_rate_dps,
                                                               speedL_dps,
                                                               speedR_dps,
                                                               modeResult);
    snapshot.imuValid = orientation.fresh(esp_timer_get_time(), m_maxSampleAgeUs.load());
    snapshot.imuAgeMs = orientation.sample_timestamp_us > 0 ?
        (esp_timer_get_time() - orientation.sample_timestamp_us) / 1000.0f : -1.0f;
    snapshot.imuGeneration = orientation.generation;
    snapshot.encoderLeftValid = encoders.left.valid;
    snapshot.encoderRightValid = encoders.right.valid;
    snapshot.imuSampleRepeated = reusedImuSample;
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
