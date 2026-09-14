#include "RobotController.hpp"

#include "OrientationEstimator.hpp"
#include "CONFIG_BehaviorConfigUpdate.hpp"
#include "CONFIG_EncoderConfigUpdate.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BatteryService.hpp"
#include "ControlModeExecutor.hpp"
#include "ControlEventDispatcher.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "MOTION_TargetMovement.hpp"
#include "MOTION_TargetLinearVelocity.hpp"
#include "TelemetryDataPoint.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>

namespace {

uint8_t telemetryFaultReason(const char* cause)
{
    if (!cause) return 255;
    if (std::strcmp(cause, "invalid-encoder") == 0) return 1;
    if (std::strcmp(cause, "invalid-input") == 0) return 2;
    if (std::strcmp(cause, "revoked-arm") == 0) return 3;
    if (std::strcmp(cause, "invalid-odometry") == 0) return 4;
    if (std::strcmp(cause, "changed-during-step") == 0) return 5;
    if (std::strcmp(cause, "motor-commit") == 0) return 6;
    return 7;
}

const char* controlFaultCause(BalanceControlFaultReason reason)
{
    switch (reason) {
        case BalanceControlFaultReason::INVALID_ODOMETRY:
        case BalanceControlFaultReason::ODOMETRY_GENERATION:
            return "invalid-odometry";
        case BalanceControlFaultReason::INVALID_COMMAND:
        case BalanceControlFaultReason::INVALID_INPUT:
            return "invalid-input";
        case BalanceControlFaultReason::INVALID_PROFILE:
        case BalanceControlFaultReason::INVALID_PID:
        case BalanceControlFaultReason::NOT_CONFIGURED:
        case BalanceControlFaultReason::UNKNOWN_MODE:
            return "invalid-control";
        case BalanceControlFaultReason::NONE:
        default:
            return nullptr;
    }
}

}

RobotController::RobotController(
    std::shared_ptr<OrientationEstimator> estimator,
    EncoderService& encoderService,
    MotorService& motorService,
    BatteryService& batteryService,
    ControlModeExecutor& controlModeExecutor,
    ControlEventDispatcher& controlEventDispatcher,
    const SystemBehaviorConfig& behavior,
    const EncoderConfig& encoderConfig,
    int controlIntervalMs,
    const LongitudinalCascadeStrategyConfig& longitudinalConfig
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
    m_longitudinalOdometry(longitudinalOdometryConfigFromEncoder(
        encoderConfig,
        behavior.imu_max_sample_age_ms * 1000LL,
        2000,
        longitudinalConfig.left_encoder_forward_sign,
        longitudinalConfig.right_encoder_forward_sign)),
    m_odometryEncoderConfig(encoderConfig),
    m_longitudinalLeftEncoderForwardSign(
        longitudinalConfig.left_encoder_forward_sign < 0 ? -1 : 1),
    m_longitudinalRightEncoderForwardSign(
        longitudinalConfig.right_encoder_forward_sign < 0 ? -1 : 1)
{
    m_maxSampleAgeUs = behavior.imu_max_sample_age_ms * 1000LL;
    m_motionCommandTimeoutUs = behavior.joystick_timeout_ms * 1000LL;
    m_controlIntervalMs = std::max(1, std::min(1000, controlIntervalMs));
    m_configuredBalanceStrategy = m_controlModeExecutor.activeBalanceStrategyId();
    ESP_LOGI(TAG, "RobotController constructed.");
}

// EventHandler implementation
void RobotController::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_BehaviorConfigUpdate>()) {
        const auto& config = event.as<CONFIG_BehaviorConfigUpdate>().config;
        m_maxSampleAgeUs = config.imu_max_sample_age_ms * 1000LL;
        m_motionCommandTimeoutUs = config.joystick_timeout_ms * 1000LL;
    } else if (event.is<CONFIG_EncoderConfigUpdate>()) {
        handleEncoderConfigUpdate(event.as<CONFIG_EncoderConfigUpdate>());
    } else if (event.is<CONFIG_FullConfigUpdate>()) {
        const auto& config = event.as<CONFIG_FullConfigUpdate>().configData;
        m_controlIntervalMs.store(
            std::max(1, std::min(1000, config.mainLoop.interval_ms)),
            std::memory_order_relaxed);
        const auto& longitudinal = config.control.strategies.longitudinal_cascade;
        {
            std::lock_guard<std::mutex> odometryLock(m_odometryMutex);
            const int8_t leftSign = longitudinal.left_encoder_forward_sign < 0 ? -1 : 1;
            const int8_t rightSign = longitudinal.right_encoder_forward_sign < 0 ? -1 : 1;
            if (m_odometryEncoderConfig != config.encoder ||
                m_longitudinalLeftEncoderForwardSign != leftSign ||
                m_longitudinalRightEncoderForwardSign != rightSign) {
                m_longitudinalLeftEncoderForwardSign = leftSign;
                m_longitudinalRightEncoderForwardSign = rightSign;
                m_odometryEncoderConfig = config.encoder;
                m_longitudinalOdometry.configure(
                    longitudinalOdometryConfigFromEncoder(
                        config.encoder,
                        m_maxSampleAgeUs.load(std::memory_order_relaxed),
                        2000,
                        leftSign,
                        rightSign));
                m_hasOdometryArm = false;
            }
        }
        bool strategyChanged = false;
        {
            std::lock_guard<std::mutex> modeLock(m_modeMutex);
            strategyChanged = m_configuredBalanceStrategy !=
                config.control.strategies.active;
            m_configuredBalanceStrategy = config.control.strategies.active;
            if (strategyChanged) {
                // A strategy switch is a command-session boundary even when
                // it occurs while already DISABLED. Preserve the sequence
                // floor, but discard the payload so it cannot be reused when
                // the old strategy is selected again.
                std::lock_guard<std::mutex> commandLock(m_commandMutex);
                const uint64_t latestSequence = m_latestMotionCommand.sequence;
                const uint64_t previousFloor =
                    m_motionCommandFloor.load(std::memory_order_relaxed);
                m_motionCommandFloor.store(std::max(previousFloor,
                                                     latestSequence),
                                           std::memory_order_release);
                m_latestMotionCommand = {};
            }
        }
        if (config.control.strategies.active != BalanceStrategyId::NESTED_PID) {
            // A delayed legacy pitch/yaw callback must not become a latent
            // target when the longitudinal strategy is selected.  Clear the
            // compatibility atomics at the same configuration boundary that
            // changes their consumer.
            m_latestTargetPitchOffset_deg.store(0.0f, std::memory_order_relaxed);
            m_latestTargetAngVel_dps.store(0.0f, std::memory_order_relaxed);
        }
    } else if (event.is<MOTION_TargetMovement>()) {
        BalanceStrategyId configuredStrategy;
        {
            std::lock_guard<std::mutex> modeLock(m_modeMutex);
            configuredStrategy = m_configuredBalanceStrategy;
        }
        if (configuredStrategy == BalanceStrategyId::NESTED_PID) {
            handleTargetMovementCommand(event.as<MOTION_TargetMovement>());
        } else {
            ESP_LOGW(TAG, "Ignoring legacy pitch/yaw command while longitudinal strategy is active");
        }
    } else if (event.is<MOTION_TargetLinearVelocity>()) {
        handleTargetLinearVelocityCommand(event.as<MOTION_TargetLinearVelocity>());
    } else if (event.is<CONTROL_RunModeChanged>()) {
        handleControlRunModeChanged(event.as<CONTROL_RunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void RobotController::handleEncoderConfigUpdate(
    const CONFIG_EncoderConfigUpdate& event)
{
    std::lock_guard<std::mutex> lock(m_odometryMutex);
    // Geometry and count scale belong to the same validated encoder snapshot
    // consumed by EncoderService.  Reconfigure atomically and invalidate the
    // old position base; the next coherent frame establishes a new one.
    m_longitudinalOdometry.configure(longitudinalOdometryConfigFromEncoder(
        event.config, m_maxSampleAgeUs.load(std::memory_order_relaxed), 2000,
        m_longitudinalLeftEncoderForwardSign,
        m_longitudinalRightEncoderForwardSign));
    m_odometryEncoderConfig = event.config;
    m_hasOdometryArm = false;
}


void RobotController::handleTargetMovementCommand(const MOTION_TargetMovement& event) {
    uint64_t currentArm = 0;
    BalanceStrategyId configuredStrategy = BalanceStrategyId::NESTED_PID;
    {
        std::lock_guard<std::mutex> modeLock(m_modeMutex);
        currentArm = m_armId;
        configuredStrategy = m_configuredBalanceStrategy;
    }
    if (configuredStrategy != BalanceStrategyId::NESTED_PID ||
        event.armId != currentArm) {
        ESP_LOGW(TAG, "Ignoring legacy pitch/yaw command from arm=%llu current=%llu strategy=%s",
                 static_cast<unsigned long long>(event.armId),
                 static_cast<unsigned long long>(currentArm),
                 balanceStrategyIdToString(configuredStrategy));
        return;
    }
    m_latestTargetPitchOffset_deg.store(event.targetPitchOffset_deg, std::memory_order_relaxed);
    m_latestTargetAngVel_dps.store(event.targetAngularVelocity_dps, std::memory_order_relaxed);
    ESP_LOGV(TAG, "RC Handler: Updated targets: PitchOffset=%.2f, AngVel=%.2f", event.targetPitchOffset_deg, event.targetAngularVelocity_dps);
}

void RobotController::handleTargetLinearVelocityCommand(
    const MOTION_TargetLinearVelocity& event)
{
    // Keep the mode lock while accepting the command. This makes the
    // arm/session check atomic with a mode transition that clears the command
    // snapshot; a delayed callback cannot slip in between those operations.
    const char* warning = nullptr;
    uint64_t currentArm = 0;
    uint64_t floor = 0;
    uint64_t latestSequence = 0;
    int64_t nowUs = 0;
    bool invalidTimestamp = false;
    bool invalidVelocity = false;
    BalanceStrategyId configuredStrategy = BalanceStrategyId::NESTED_PID;
    {
        std::lock_guard<std::mutex> modeLock(m_modeMutex);
        currentArm = m_armId;
        configuredStrategy = m_configuredBalanceStrategy;
        if (configuredStrategy != BalanceStrategyId::LONGITUDINAL_CASCADE) {
            warning = "wrong-strategy";
        } else if (event.sequence == 0 || event.armId != currentArm) {
            warning = "wrong-session";
        } else {
            floor = m_motionCommandFloor.load(std::memory_order_acquire);
            if (event.sequence <= floor) {
                warning = "stale-sequence";
            } else {
                std::lock_guard<std::mutex> lock(m_commandMutex);
                if (event.sequence <= m_motionCommandFloor.load(std::memory_order_relaxed)) {
                    warning = "stale-sequence";
                } else if (m_latestMotionCommand.valid &&
                           event.sequence <= m_latestMotionCommand.sequence) {
                    latestSequence = m_latestMotionCommand.sequence;
                    warning = "out-of-order";
                } else {
                    nowUs = esp_timer_get_time();
                    invalidTimestamp = event.receivedTimestampUs <= 0 ||
                        event.receivedTimestampUs > nowUs;
                    invalidVelocity = !std::isfinite(event.targetVelocityMps);
                    if (invalidTimestamp || invalidVelocity) {
                        // Consume the sequence and clear the previous command so malformed or
                        // future-dated input cannot leave an older drive request fresh until
                        // its normal timeout expires.
                        m_motionCommandFloor.store(event.sequence, std::memory_order_release);
                        m_latestMotionCommand = {};
                        warning = "invalid";
                    } else {
                        m_latestMotionCommand.valid = true;
                        m_latestMotionCommand.fresh = true;
                        m_latestMotionCommand.stop = event.stop;
                        m_latestMotionCommand.targetVelocityMps = event.targetVelocityMps;
                        m_latestMotionCommand.sequence = event.sequence;
                        m_latestMotionCommand.receivedTimestampUs = event.receivedTimestampUs;
                        m_latestMotionCommand.armId = event.armId;
                    }
                }
            }
        }
    }

    // Logging is intentionally outside both the mode and command mutexes. The
    // control task shares these locks and must not inherit a slow log path.
    if (!warning) {
        ESP_LOGV(TAG, "RC Handler: Updated linear target: %.3f m/s stop=%d seq=%llu",
                 event.targetVelocityMps, event.stop ? 1 : 0,
                 static_cast<unsigned long long>(event.sequence));
    } else if (std::strcmp(warning, "wrong-strategy") == 0) {
        ESP_LOGW(TAG, "Ignoring linear command while configured strategy is %s",
                 balanceStrategyIdToString(configuredStrategy));
    } else if (std::strcmp(warning, "wrong-session") == 0) {
        ESP_LOGW(TAG, "Ignoring linear command from wrong session: arm=%llu current=%llu seq=%llu",
                 static_cast<unsigned long long>(event.armId),
                 static_cast<unsigned long long>(currentArm),
                 static_cast<unsigned long long>(event.sequence));
    } else if (std::strcmp(warning, "stale-sequence") == 0) {
        ESP_LOGW(TAG, "Ignoring stale linear command sequence=%llu floor=%llu",
                 static_cast<unsigned long long>(event.sequence),
                 static_cast<unsigned long long>(floor));
    } else if (std::strcmp(warning, "out-of-order") == 0) {
        ESP_LOGW(TAG, "Ignoring out-of-order linear command sequence=%llu latest=%llu",
                 static_cast<unsigned long long>(event.sequence),
                 static_cast<unsigned long long>(latestSequence));
    } else {
        ESP_LOGW(TAG, "Rejecting invalid linear command: seq=%llu timestamp=%lld now=%lld finite=%d",
                 static_cast<unsigned long long>(event.sequence),
                 static_cast<long long>(event.receivedTimestampUs),
                 static_cast<long long>(nowUs),
                 invalidVelocity ? 0 : 1);
    }
}

void RobotController::handleControlRunModeChanged(const CONTROL_RunModeChanged& event) {
    std::lock_guard<std::mutex> modeLock(m_modeMutex);
    if (event.armId < m_armId) return;
    const bool sessionChanged = event.armId != m_armId ||
        event.generation != m_generation || event.mode != m_controlMode;
    if (sessionChanged) {
        std::lock_guard<std::mutex> commandLock(m_commandMutex);
        const uint64_t latestSequence = m_latestMotionCommand.sequence;
        const uint64_t previousFloor = m_motionCommandFloor.load(std::memory_order_relaxed);
        m_motionCommandFloor.store(std::max(previousFloor, latestSequence),
                                   std::memory_order_release);
        m_latestMotionCommand = {};
    }
    m_armId = event.armId;
    m_generation = event.generation;
    m_telemetryStateCode.store(event.telemetryStateCode, std::memory_order_relaxed);
    m_telemetryEnabled.store(event.telemetryEnabled, std::memory_order_relaxed);
    m_controlMode.store(event.mode, std::memory_order_relaxed);
    // This mutex is shared with the control task. Never log while holding it.
}

void RobotController::runControlStep(float dt) {
    const int64_t startTimeMicros = esp_timer_get_time();
    int32_t motorCommitResult = 0;
    bool motorCommitAttempted = false;
    bool motorCommitSucceeded = false;
    uint8_t faultReason = 0;
    bool faultLatched = false;

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
    LongitudinalOdometryResult odometry;
    {
        std::lock_guard<std::mutex> lock(m_odometryMutex);
        if (!m_hasOdometryArm || arm != m_lastOdometryArm) {
            m_longitudinalOdometry.reset();
            m_lastOdometryArm = arm;
            m_hasOdometryArm = true;
        }
        m_longitudinalOdometry.setMaxSampleAgeUs(
            m_maxSampleAgeUs.load(std::memory_order_relaxed));
        odometry = m_longitudinalOdometry.update(encoders, esp_timer_get_time());
    }
    const float speedL_dps = encoders.left.speedDps;
    const float speedR_dps = encoders.right.speedDps;
    const bool reusedImuSample = orientation.sample_sequence == m_lastImuSequence &&
        orientation.generation == m_lastImuGeneration;
    m_lastImuSequence = orientation.sample_sequence;
    m_lastImuGeneration = orientation.generation;

    const float currentTargetPitchOffset_deg = m_latestTargetPitchOffset_deg.load(std::memory_order_relaxed);
    const float currentTargetAngVel_dps = m_latestTargetAngVel_dps.load(std::memory_order_relaxed);
    LongitudinalMotionCommand motionCommand;
    {
        std::lock_guard<std::mutex> lock(m_commandMutex);
        motionCommand = m_latestMotionCommand;
    }
    const int64_t nowUs = esp_timer_get_time();
    if (motionCommand.valid) {
        const int64_t timeoutUs = m_motionCommandTimeoutUs.load(std::memory_order_relaxed);
        const int64_t ageUs = nowUs - motionCommand.receivedTimestampUs;
        motionCommand.fresh = motionCommand.receivedTimestampUs > 0 &&
            ageUs >= 0 && (timeoutUs <= 0 || ageUs <= timeoutUs);
    }

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
    modeInput.nowUs = nowUs;
    modeInput.motionTimeoutUs = m_motionCommandTimeoutUs.load(std::memory_order_relaxed);
    modeInput.motion = motionCommand;
    modeInput.odometry = odometry;
    modeInput.controlArmId = arm;

    const auto fault = [&](const char* cause, esp_err_t error = ESP_OK) {
        const auto observed = esp_timer_get_time();
        const auto latest = m_estimator->getOrientation();
        faultReason = telemetryFaultReason(cause);
        faultLatched = true;
        if (error != ESP_OK) motorCommitResult = static_cast<int32_t>(error);
        m_motorService.inhibitImu(arm);
        m_controlModeExecutor.reset();
        if (m_lastFaultArm != arm) {
            m_lastFaultArm = arm;
            m_controlEventDispatcher.latchImuFault({arm, generation, observed, IMUFaultReason::STALE,
                orientation.sample_timestamp_us, latest.sample_timestamp_us, cause, error});
        }
    };
    const bool active = currentMode != ControlRunMode::DISABLED;
    const BalanceStrategyId activeBalanceStrategy =
        m_controlModeExecutor.activeBalanceStrategyId();
    const bool legacyTargetsRequired = currentMode != ControlRunMode::BALANCING ||
        activeBalanceStrategy == BalanceStrategyId::NESTED_PID;
    const auto inputsValid = [&] {
        const int64_t now = esp_timer_get_time();
        const int64_t maxSampleAgeUs = m_maxSampleAgeUs.load(std::memory_order_relaxed);
        const bool encodersFresh = encoders.left.sampleTimestampUs > 0 &&
            encoders.right.sampleTimestampUs > 0 &&
            now >= encoders.left.sampleTimestampUs &&
            now >= encoders.right.sampleTimestampUs &&
            now - encoders.left.sampleTimestampUs <= maxSampleAgeUs &&
            now - encoders.right.sampleTimestampUs <= maxSampleAgeUs;
        const bool legacyTargetsFinite = !legacyTargetsRequired ||
            (std::isfinite(currentTargetPitchOffset_deg) &&
             std::isfinite(currentTargetAngVel_dps));
        return orientation.fresh(now, maxSampleAgeUs) &&
            orientation.generation == generation && encoders.left.valid && encoders.right.valid &&
            encodersFresh && std::isfinite(dt) && dt > 0 &&
            std::isfinite(speedL_dps) && std::isfinite(speedR_dps) &&
            legacyTargetsFinite;
    };
    ControlModeResult modeResult{};
    if (active && (!inputsValid() || !m_motorService.isArmAllowed(arm, generation))) {
        const char* cause = !encoders.left.valid || !encoders.right.valid
            ? "invalid-encoder" : inputsValid() ? "revoked-arm" : "invalid-input";
        fault(cause, ESP_ERR_INVALID_STATE);
        modeResult.strategyId = activeBalanceStrategy;
        modeResult.strategyRevision = m_controlModeExecutor.activeBalanceStrategyRevision();
        modeResult.configRevision = m_controlModeExecutor.appliedConfigRevision();
        modeResult.diagnostics.strategyId = activeBalanceStrategy;
        modeResult.diagnostics.phase = BalanceControlPhase::FAULT;
        modeResult.diagnostics.loopMode = activeBalanceStrategy ==
            BalanceStrategyId::NESTED_PID ? -1 : modeResult.diagnostics.loopMode;
        modeResult.valid = false;
    } else {
        if (arm != m_lastExecutedArm) { m_controlModeExecutor.reset(); m_lastExecutedArm = arm; }
        modeResult = m_controlModeExecutor.execute(modeInput);
        const auto latest = m_estimator->getOrientation();
        const auto controlFailure = modeResult.diagnostics.faultReason;
        const bool longitudinalStrategy = active &&
            modeResult.diagnostics.strategyId == BalanceStrategyId::LONGITUDINAL_CASCADE;
        const bool longitudinalOdometryInvalid = longitudinalStrategy &&
            (controlFailure == BalanceControlFaultReason::INVALID_ODOMETRY ||
             controlFailure == BalanceControlFaultReason::ODOMETRY_GENERATION ||
             (modeResult.diagnostics.velocityLoopEnabled && !odometry.odometryValid));
        if (active && (!modeResult.valid || longitudinalOdometryInvalid || !inputsValid() || !latest.valid || latest.generation != generation ||
            !std::isfinite(modeResult.effort.left) || !std::isfinite(modeResult.effort.right))) {
            const char* strategyFault = controlFaultCause(controlFailure);
            fault(strategyFault ? strategyFault :
                (longitudinalOdometryInvalid ? "invalid-odometry" :
                 "changed-during-step"));
            modeResult.valid = false;
            modeResult.effort = {};
        } else {
            motorCommitAttempted = active;
            const auto result = m_motorService.setMotorEffort(modeResult.effort.left, modeResult.effort.right, arm, generation,
                active ? std::min(orientation.sample_timestamp_us,
                    std::min(encoders.left.sampleTimestampUs, encoders.right.sampleTimestampUs)) : 0,
                active ? m_maxSampleAgeUs.load() : 0);
            motorCommitResult = static_cast<int32_t>(result);
            motorCommitSucceeded = active && result == ESP_OK;
            if (active && result != ESP_OK) {
                fault("motor-commit", result);
                modeResult.valid = false;
                modeResult.effort = {};
            }
        }
    }

    TelemetryDataPoint snapshot = buildTelemetrySnapshot(startTimeMicros,
                                                               telemetryStateCode,
                                                               pitch_deg,
                                                               yaw_deg,
                                                               yaw_rate_dps,
                                                               speedL_dps,
                                                               speedR_dps,
                                                               arm,
                                                               generation,
                                                               odometry,
                                                               modeResult);
    snapshot.imuValid = orientation.fresh(esp_timer_get_time(), m_maxSampleAgeUs.load());
    snapshot.imuAgeMs = orientation.sample_timestamp_us > 0 ?
        (esp_timer_get_time() - orientation.sample_timestamp_us) / 1000.0f : -1.0f;
    snapshot.imuGeneration = orientation.generation;
    snapshot.encoderLeftValid = encoders.left.valid;
    snapshot.encoderRightValid = encoders.right.valid;
    snapshot.imuSampleRepeated = reusedImuSample;
    snapshot.motorCommitAttempted = motorCommitAttempted;
    snapshot.motorCommitSucceeded = motorCommitSucceeded;
    snapshot.motorCommitResult = motorCommitResult;
    snapshot.faultReason = faultReason;
    snapshot.faultLatched = faultLatched;
    snapshot.imuSampleSequence = orientation.sample_sequence;
    const int64_t stepCostUs = std::max<int64_t>(
        0, esp_timer_get_time() - startTimeMicros);
    snapshot.controlStepCostUs = static_cast<uint32_t>(std::min<int64_t>(
        stepCostUs, std::numeric_limits<uint32_t>::max()));
    snapshot.controlStepLate = stepCostUs >
        static_cast<int64_t>(controlIntervalMs()) * 1000;
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
                                                           uint64_t commandSessionId,
                                                           uint32_t controlGeneration,
                                                           const LongitudinalOdometryResult& odometry,
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

    const auto& diagnostics = modeResult.diagnostics;
    const BalanceStrategyId telemetryStrategy = modeResult.valid
        ? modeResult.strategyId : m_controlModeExecutor.activeBalanceStrategyId();
    snapshot.strategyId = static_cast<uint8_t>(telemetryStrategy);
    snapshot.loopMode = diagnostics.loopMode;
    snapshot.controlPhase = static_cast<uint8_t>(diagnostics.phase);
    snapshot.strategyRevision = modeResult.valid ? modeResult.strategyRevision
        : m_controlModeExecutor.activeBalanceStrategyRevision();
    snapshot.configRevision = modeResult.configRevision != 0 ? modeResult.configRevision
        : m_controlModeExecutor.appliedConfigRevision();
    snapshot.commandSessionId = commandSessionId;
    snapshot.controlGeneration = controlGeneration;
    snapshot.odometryGeneration = odometry.generation;
    snapshot.odometrySequence = odometry.odometrySequence;
    snapshot.controlValid = modeResult.valid;
    snapshot.targetPitchValid = diagnostics.targetPitchValid;
    snapshot.targetPitchClamped = diagnostics.targetPitchClamped;
    snapshot.targetPitchRateLimited = diagnostics.targetPitchRateLimited;
    snapshot.positionTargetValid = diagnostics.positionTargetValid;
    snapshot.positionHoldActive = diagnostics.positionHoldActive;
    snapshot.synchronizationTargetValid = diagnostics.synchronizationTargetValid;
    snapshot.velocityLoopEnabled = diagnostics.velocityLoopEnabled;
    snapshot.positionLoopEnabled = diagnostics.positionLoopEnabled;
    snapshot.synchronizationEnabled = diagnostics.synchronizationEnabled;
    snapshot.motionCommandValid = diagnostics.motionCommandValid;
    snapshot.motionCommandFresh = diagnostics.motionCommandFresh;
    snapshot.velocityFeedbackValid = diagnostics.velocityFeedbackValid;
    snapshot.velocityTargetClamped = diagnostics.velocityTargetClamped;
    snapshot.velocityOutputSaturated = diagnostics.velocityOutputSaturated;
    snapshot.velocityAntiWindup = diagnostics.velocityAntiWindup;
    snapshot.pitchPidSaturated = diagnostics.pitchPidSaturated;
    snapshot.balanceSaturated = diagnostics.balanceSaturated;
    snapshot.mixerSaturated = diagnostics.mixerSaturated;
    snapshot.syncLimited = diagnostics.syncLimited;
    snapshot.motionRequestLimited = diagnostics.motionRequestLimited;
    snapshot.targetPitch_deg = diagnostics.targetPitchValid
        ? diagnostics.targetPitch_deg : snapshot.desiredAngle_deg;
    snapshot.targetVelocityMps = diagnostics.targetVelocityMps;
    snapshot.commandVelocityMps = diagnostics.commandVelocityMps;
    snapshot.measuredVelocityMps = diagnostics.measuredVelocityMps;
    snapshot.holdVelocityRequestMps = diagnostics.holdVelocityRequestMps;
    snapshot.holdVelocityTargetMps = diagnostics.holdVelocityTargetMps;
    snapshot.positionM = diagnostics.positionM;
    snapshot.holdPositionM = diagnostics.holdPositionM;
    snapshot.positionErrorM = diagnostics.positionErrorM;
    snapshot.distanceDifferenceM = diagnostics.distanceDifferenceM;
    snapshot.distanceDifferenceTargetM = diagnostics.distanceDifferenceTargetM;
    snapshot.syncVelocityDifferenceMps = diagnostics.syncVelocityDifferenceMps;
    snapshot.requestedBalanceEffort = diagnostics.requestedBalanceEffort;
    snapshot.balanceEffort = diagnostics.balanceEffort;
    snapshot.requestedSyncEffort = diagnostics.requestedSyncEffort;
    snapshot.syncEffort = diagnostics.syncEffort;
    snapshot.leftEffort = diagnostics.leftEffort;
    snapshot.rightEffort = diagnostics.rightEffort;
    snapshot.yawControlAvailable = telemetryStrategy == BalanceStrategyId::NESTED_PID;
    snapshot.yawTargetValid = snapshot.yawControlAvailable &&
        diagnostics.yawControlEnabled;
    if (!modeResult.valid) {
        snapshot.phaseReason = 5; // FAULT/invalid control result.
    } else if (diagnostics.phase == BalanceControlPhase::BRAKE) {
        snapshot.phaseReason = diagnostics.motionCommandFresh ? 2 : 1;
    } else if (diagnostics.motionRequestLimited) {
        snapshot.phaseReason = 6;
    } else if (diagnostics.velocityFeedbackValid == false &&
               diagnostics.velocityLoopEnabled) {
        snapshot.phaseReason = 4;
    } else if (diagnostics.phase == BalanceControlPhase::HOLD) {
        snapshot.phaseReason = diagnostics.positionHoldActive ? 0 : 3;
    }

    return snapshot;
}
