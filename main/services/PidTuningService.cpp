#include "PidTuningService.hpp"

#include "ConfigurationService.hpp"
#include "EncoderService.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "PID_TuningFinished.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "SystemState.hpp"
#include "UI_CancelPidTuning.hpp"
#include "UI_DiscardPidTuning.hpp"
#include "UI_SavePidTuning.hpp"
#include "UI_StartPidTuning.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float MIN_IDENTIFICATION_TIME_S = 0.02f;
constexpr float MIN_EXTRA_STEP_DELTA = 0.03f;
constexpr float MAX_SETTLE_WAIT_S = 1.5f;
constexpr float MAX_RESPONSE_GAIN_RATIO = 2.5f;
constexpr float MAX_RESPONSE_TIME_RATIO = 4.0f;
constexpr float MAX_DIRECTION_GAIN_RATIO = 3.5f;

float averagePositive(float a, float b) {
    return (a + b) * 0.5f;
}
}

PidTuningService::PidTuningService(EventBus& eventBus,
                                   ConfigurationService& configService,
                                   EncoderService& encoderService,
                                   const PidTuningConfig& initialConfig)
    : m_eventBus(eventBus),
      m_configService(configService),
      m_encoderService(encoderService),
      m_config(initialConfig),
      m_validationPidLeft("tuning_validate_left"),
      m_validationPidRight("tuning_validate_right") {
    resetRuntimeLocked();
}

esp_err_t PidTuningService::init() {
    ESP_LOGI(TAG, "PidTuningService initialized.");
    return ESP_OK;
}

PidTuningStatus PidTuningService::getStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_status;
}

void PidTuningService::handleEvent(const BaseEvent& event) {
    switch (event.type) {
        case EventType::UI_START_PID_TUNING:
            handleStartCommand(static_cast<const UI_StartPidTuning&>(event));
            break;
        case EventType::UI_CANCEL_PID_TUNING:
            handleCancelCommand(static_cast<const UI_CancelPidTuning&>(event));
            break;
        case EventType::UI_SAVE_PID_TUNING:
            handleSaveCommand(static_cast<const UI_SavePidTuning&>(event));
            break;
        case EventType::UI_DISCARD_PID_TUNING:
            handleDiscardCommand(static_cast<const UI_DiscardPidTuning&>(event));
            break;
        case EventType::SYSTEM_STATE_CHANGED:
            handleSystemStateChanged(static_cast<const SYSTEM_StateChanged&>(event));
            break;
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d",
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

MotorEffort PidTuningService::update(float dt, float speedLeft_dps, float speedRight_dps) {
    bool shouldPublishFinished = false;
    bool shouldApplyPreview = false;
    PidTuningState finishedState = PidTuningState::IDLE;
    std::string finishedMessage;
    MotorEffort effort;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        effort = updateLocked(dt,
                              speedLeft_dps,
                              speedRight_dps,
                              shouldPublishFinished,
                              finishedState,
                              finishedMessage,
                              shouldApplyPreview);
    }

    if (shouldApplyPreview) {
        applyPreview();
    }

    if (shouldPublishFinished) {
        publishFinished(finishedState, finishedMessage);
    }

    return effort;
}

MotorEffort PidTuningService::updateLocked(float dt,
                                           float speedLeft_dps,
                                           float speedRight_dps,
                                           bool& shouldPublishFinished,
                                           PidTuningState& finishedState,
                                           std::string& finishedMessage,
                                           bool& shouldApplyPreview) {
    MotorEffort effort = {0.0f, 0.0f};
    m_lastSpeedSetpointLeft_dps = 0.0f;
    m_lastSpeedSetpointRight_dps = 0.0f;

    if (m_status.state != PidTuningState::RUNNING) {
        return effort;
    }

    if (dt <= 0.0f) {
        return effort;
    }

    if (m_cancelRequested) {
        cancelRunLocked("Tuning canceled");
        shouldPublishFinished = true;
        finishedState = PidTuningState::CANCELED;
        finishedMessage = m_status.message;
        return effort;
    }

    if (std::fabs(speedLeft_dps) > m_config.max_speed_dps ||
        std::fabs(speedRight_dps) > m_config.max_speed_dps) {
        failRunLocked("Tuning aborted: overspeed");
        shouldPublishFinished = true;
        finishedState = PidTuningState::FAILED;
        finishedMessage = m_status.message;
        return effort;
    }

    m_phaseElapsed_s += dt;
    updateProgressLocked();

    switch (m_status.phase) {
        case PidTuningPhase::RESET:
            if (!m_resetDone) {
                m_encoderService.reset();
                m_resetDone = true;
            }
            transitionToRestLocked();
            break;

        case PidTuningPhase::REST_BEFORE_STEP:
            if (m_phaseElapsed_s >= (m_config.rest_duration_ms / 1000.0f)) {
                if (m_stepIndex >= m_stepPlan.size()) {
                    transitionToComputeLocked();
                    break;
                }
                const float activeSpeed = currentStepActiveSpeed(speedLeft_dps, speedRight_dps);
                const float settleLimit = std::max(25.0f, m_config.min_response_dps * 0.75f);
                if (std::fabs(activeSpeed) > settleLimit) {
                    if (m_phaseElapsed_s < (m_config.rest_duration_ms / 1000.0f) + MAX_SETTLE_WAIT_S) {
                        setMessageLocked("Waiting for wheel speed to settle");
                        break;
                    }
                    failRunLocked("Tuning aborted: wheel did not settle between steps");
                    shouldPublishFinished = true;
                    finishedState = PidTuningState::FAILED;
                    finishedMessage = m_status.message;
                    return effort;
                }
                transitionToStepLocked(activeSpeed);
            }
            break;

        case PidTuningPhase::LEFT_FORWARD:
        case PidTuningPhase::LEFT_REVERSE:
        case PidTuningPhase::RIGHT_FORWARD:
        case PidTuningPhase::RIGHT_REVERSE:
            effort = currentStepEffortLocked();
            m_stepSamples.push_back({m_phaseElapsed_s, currentStepActiveSpeed(speedLeft_dps, speedRight_dps)});
            if (m_phaseElapsed_s >= (m_config.step_duration_ms / 1000.0f)) {
                StepResult result = analyzeCurrentStepLocked();
                if (!result.valid) {
                    failRunLocked("Tuning aborted: invalid motor response");
                    shouldPublishFinished = true;
                    finishedState = PidTuningState::FAILED;
                    finishedMessage = m_status.message;
                    return {0.0f, 0.0f};
                }
                m_stepResults[m_stepIndex] = result;
                ++m_stepIndex;
                if (m_stepIndex >= m_stepPlan.size()) {
                    transitionToComputeLocked();
                } else {
                    transitionToRestLocked();
                }
                return {0.0f, 0.0f};
            }
            break;

        case PidTuningPhase::COMPUTE:
            {
                std::string error;
                if (!computeCandidateLocked(error)) {
                    failRunLocked(error);
                    shouldPublishFinished = true;
                    finishedState = PidTuningState::FAILED;
                    finishedMessage = m_status.message;
                    return effort;
                }
                if (isTargetLeftWheelLocked()) {
                    transitionToValidateLeftLocked();
                } else {
                    transitionToValidateRightLocked();
                }
            }
            break;

        case PidTuningPhase::VALIDATE_LEFT:
            m_lastSpeedSetpointLeft_dps = m_config.validation_target_dps;
            effort.left = clampAbs(m_validationPidLeft.compute(m_config.validation_target_dps, speedLeft_dps, dt), m_config.max_effort);
            effort.right = 0.0f;
            if (m_phaseElapsed_s >= (m_config.step_duration_ms / 1000.0f) * 0.75f) {
                m_validationFinalSum_dps += speedLeft_dps;
                ++m_validationFinalCount;
            }
            if (m_phaseElapsed_s >= (m_config.step_duration_ms / 1000.0f)) {
                std::string error;
                if (!analyzeValidationLocked(m_config.validation_target_dps, "left", error)) {
                    failRunLocked(error);
                    shouldPublishFinished = true;
                    finishedState = PidTuningState::FAILED;
                    finishedMessage = m_status.message;
                    return {0.0f, 0.0f};
                }
                completeRunLocked();
                shouldApplyPreview = true;
                shouldPublishFinished = true;
                finishedState = PidTuningState::PREVIEW_READY;
                finishedMessage = m_status.message;
                return {0.0f, 0.0f};
            }
            break;

        case PidTuningPhase::REST_BEFORE_VALIDATE_RIGHT:
            if (m_phaseElapsed_s >= (m_config.rest_duration_ms / 1000.0f)) {
                transitionToValidateRightLocked();
            }
            break;

        case PidTuningPhase::VALIDATE_RIGHT:
            m_lastSpeedSetpointRight_dps = m_config.validation_target_dps;
            effort.left = 0.0f;
            effort.right = clampAbs(m_validationPidRight.compute(m_config.validation_target_dps, speedRight_dps, dt), m_config.max_effort);
            if (m_phaseElapsed_s >= (m_config.step_duration_ms / 1000.0f) * 0.75f) {
                m_validationFinalSum_dps += speedRight_dps;
                ++m_validationFinalCount;
            }
            if (m_phaseElapsed_s >= (m_config.step_duration_ms / 1000.0f)) {
                std::string error;
                if (!analyzeValidationLocked(m_config.validation_target_dps, "right", error)) {
                    failRunLocked(error);
                    shouldPublishFinished = true;
                    finishedState = PidTuningState::FAILED;
                    finishedMessage = m_status.message;
                    return {0.0f, 0.0f};
                }
                completeRunLocked();
                shouldApplyPreview = true;
                shouldPublishFinished = true;
                finishedState = PidTuningState::PREVIEW_READY;
                finishedMessage = m_status.message;
                return {0.0f, 0.0f};
            }
            break;

        default:
            break;
    }

    return effort;
}

void PidTuningService::handleStartCommand(const UI_StartPidTuning& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_requestedTarget = event.target;
    m_startRequested = true;
}

void PidTuningService::handleCancelCommand(const UI_CancelPidTuning& event) {
    (void)event;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_status.state == PidTuningState::RUNNING) {
        m_cancelRequested = true;
    }
}

void PidTuningService::handleSaveCommand(const UI_SavePidTuning& event) {
    (void)event;
    PIDConfig tunedConfig;
    PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
    bool shouldSave = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_previewAvailable && m_status.state == PidTuningState::PREVIEW_READY) {
            target = m_status.target;
            tunedConfig = isLeftWheelTarget(target) ? m_candidateLeft : m_candidateRight;
            shouldSave = true;
            m_status.state = PidTuningState::SAVED;
            m_status.phase = PidTuningPhase::IDLE;
            m_status.progress = 1.0f;
            m_status.message = std::string("Tuned ") + targetWheelNameLocked() + " motor PID saved";
            m_previewAvailable = false;
        }
    }

    if (shouldSave) {
        (void)m_configService.applyPidConfig(isLeftWheelTarget(target) ? "speed_left" : "speed_right",
                                             tunedConfig,
                                             true);
    }
}

void PidTuningService::handleDiscardCommand(const UI_DiscardPidTuning& event) {
    (void)event;
    PIDConfig originalConfig;
    PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
    bool shouldRestore = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_previewAvailable && m_status.state == PidTuningState::PREVIEW_READY) {
            target = m_status.target;
            originalConfig = isLeftWheelTarget(target) ? m_originalLeft : m_originalRight;
            shouldRestore = true;
            m_status.state = PidTuningState::DISCARDED;
            m_status.phase = PidTuningPhase::IDLE;
            m_status.progress = 0.0f;
            m_status.message = std::string("Tuned ") + targetWheelNameLocked() + " motor PID discarded";
            m_previewAvailable = false;
            m_status.hasCandidate = false;
        }
    }

    if (shouldRestore) {
        (void)m_configService.applyPidConfig(isLeftWheelTarget(target) ? "speed_left" : "speed_right",
                                             originalConfig,
                                             false);
    }
}

void PidTuningService::handleSystemStateChanged(const SYSTEM_StateChanged& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (event.newState == SystemState::PID_TUNING) {
        const PidTuningTarget target = m_startRequested ? m_requestedTarget : PidTuningTarget::MOTOR_SPEED_LEFT;
        beginRunLocked(target);
        m_startRequested = false;
        return;
    }

    if (event.previousState == SystemState::PID_TUNING &&
        event.newState != SystemState::PID_TUNING &&
        m_status.state == PidTuningState::RUNNING) {
        cancelRunLocked("Tuning stopped");
    }
}

void PidTuningService::beginRunLocked(PidTuningTarget target) {
    m_config = m_configService.getPidTuningConfig();
    m_originalLeft = m_configService.getPidSpeedLeftConfig();
    m_originalRight = m_configService.getPidSpeedRightConfig();
    m_candidateLeft = m_originalLeft;
    m_candidateRight = m_originalRight;
    resetRuntimeLocked();
    m_status.target = target;
    buildStepPlanLocked();
    m_status.state = PidTuningState::RUNNING;
    m_status.phase = PidTuningPhase::RESET;
    m_status.progress = 0.0f;
    m_status.message = std::string("Preparing ") + targetWheelNameLocked() + " motor PID tuning";
    m_previewAvailable = false;
    m_startRequested = false;
    ESP_LOGI(TAG, "Started PID tuning target=%d", static_cast<int>(target));
}

void PidTuningService::cancelRunLocked(const std::string& message) {
    m_status.state = PidTuningState::CANCELED;
    m_status.phase = PidTuningPhase::IDLE;
    m_status.progress = 0.0f;
    m_status.message = message;
    resetRuntimeLocked();
}

void PidTuningService::failRunLocked(const std::string& message) {
    m_status.state = PidTuningState::FAILED;
    m_status.phase = PidTuningPhase::IDLE;
    m_status.progress = 0.0f;
    m_status.message = message;
    resetRuntimeLocked();
    ESP_LOGW(TAG, "%s", message.c_str());
}

void PidTuningService::completeRunLocked() {
    m_status.state = PidTuningState::PREVIEW_READY;
    m_status.phase = PidTuningPhase::PREVIEW;
    m_status.progress = 1.0f;
    m_status.message = std::string("Tuned ") + targetWheelNameLocked() + " motor PID ready for preview";
    m_status.hasCandidate = true;
    m_status.candidateLeft = m_candidateLeft;
    m_status.candidateRight = m_candidateRight;
    m_previewAvailable = true;
    m_lastSpeedSetpointLeft_dps = 0.0f;
    m_lastSpeedSetpointRight_dps = 0.0f;
}

void PidTuningService::transitionToRestLocked() {
    m_status.phase = PidTuningPhase::REST_BEFORE_STEP;
    m_phaseElapsed_s = 0.0f;
    if (m_stepIndex < m_stepPlan.size()) {
        const StepDefinition& step = m_stepPlan[m_stepIndex];
        setMessageLocked(std::string("Resting before ") +
                         (step.leftWheel ? "left" : "right") +
                         (step.effort >= 0.0f ? " forward" : " reverse") +
                         " step");
    } else {
        setMessageLocked("Resting before computing tuned motor PID");
    }
}

void PidTuningService::transitionToStepLocked(float currentSpeed_dps) {
    m_status.phase = currentStepPhaseLocked();
    m_phaseElapsed_s = 0.0f;
    m_stepBaseline_dps = currentSpeed_dps;
    m_stepSamples.clear();
    m_stepSamples.reserve(400);
    const StepDefinition& step = m_stepPlan[m_stepIndex];
    char message[96];
    snprintf(message,
             sizeof(message),
             "Running %s at %.2f effort",
             phaseToString(m_status.phase),
             std::fabs(step.effort));
    setMessageLocked(message);
}

void PidTuningService::transitionToComputeLocked() {
    m_status.phase = PidTuningPhase::COMPUTE;
    m_phaseElapsed_s = 0.0f;
    setMessageLocked("Computing tuned motor PID");
}

void PidTuningService::transitionToValidateLeftLocked() {
    m_status.phase = PidTuningPhase::VALIDATE_LEFT;
    m_phaseElapsed_s = 0.0f;
    m_validationFinalSum_dps = 0.0f;
    m_validationFinalCount = 0;
    m_validationPidLeft.init(m_candidateLeft);
    m_validationPidLeft.reset();
    setMessageLocked("Validating left tuned PID");
}

void PidTuningService::transitionToValidateRightLocked() {
    m_status.phase = PidTuningPhase::VALIDATE_RIGHT;
    m_phaseElapsed_s = 0.0f;
    m_validationFinalSum_dps = 0.0f;
    m_validationFinalCount = 0;
    m_validationPidRight.init(m_candidateRight);
    m_validationPidRight.reset();
    setMessageLocked("Validating right tuned PID");
}

void PidTuningService::buildStepPlanLocked() {
    m_stepPlan.clear();

    const float stepEffort = std::max(0.01f, std::min(m_config.step_effort, m_config.max_effort));
    const float aggressiveEffort = std::max(stepEffort, m_config.max_effort);
    const bool tuneLeft = isTargetLeftWheelLocked();
    auto addLevel = [&](float effort) {
        m_stepPlan.push_back({tuneLeft, effort, tuneLeft ? PidTuningPhase::LEFT_FORWARD : PidTuningPhase::RIGHT_FORWARD});
        m_stepPlan.push_back({tuneLeft, -effort, tuneLeft ? PidTuningPhase::LEFT_REVERSE : PidTuningPhase::RIGHT_REVERSE});
    };

    addLevel(stepEffort);
    if ((aggressiveEffort - stepEffort) >= MIN_EXTRA_STEP_DELTA) {
        addLevel(aggressiveEffort);
    }

    m_stepResults.assign(m_stepPlan.size(), StepResult());
}

MotorEffort PidTuningService::currentStepEffortLocked() const {
    MotorEffort effort = {0.0f, 0.0f};
    if (m_stepIndex >= m_stepPlan.size()) {
        return effort;
    }

    const StepDefinition& step = m_stepPlan[m_stepIndex];
    const float command = clampAbs(step.effort, m_config.max_effort);
    if (step.leftWheel) {
        effort.left = command;
    } else {
        effort.right = command;
    }
    return effort;
}

float PidTuningService::currentStepActiveSpeed(float left_dps, float right_dps) const {
    if (m_stepIndex >= m_stepPlan.size()) {
        return 0.0f;
    }
    return m_stepPlan[m_stepIndex].leftWheel ? left_dps : right_dps;
}

PidTuningPhase PidTuningService::currentStepPhaseLocked() const {
    if (m_stepIndex >= m_stepPlan.size()) {
        return PidTuningPhase::IDLE;
    }
    return m_stepPlan[m_stepIndex].phase;
}

PidTuningService::StepResult PidTuningService::analyzeCurrentStepLocked() const {
    StepResult result;
    if (m_stepSamples.empty()) {
        return result;
    }

    const float duration_s = m_config.step_duration_ms / 1000.0f;
    const float finalWindowStart_s = std::max(duration_s * 0.75f, duration_s - 0.35f);
    float finalSum = 0.0f;
    int finalCount = 0;
    for (const auto& sample : m_stepSamples) {
        if (sample.time_s >= finalWindowStart_s) {
            finalSum += sample.speed_dps;
            ++finalCount;
        }
    }
    if (finalCount == 0) {
        return result;
    }

    const float steadySpeed = finalSum / static_cast<float>(finalCount);
    float maxFinalDeviation = 0.0f;
    for (const auto& sample : m_stepSamples) {
        if (sample.time_s >= finalWindowStart_s) {
            maxFinalDeviation = std::max(maxFinalDeviation, std::fabs(sample.speed_dps - steadySpeed));
        }
    }
    const float delta = steadySpeed - m_stepBaseline_dps;
    if (m_stepIndex >= m_stepPlan.size()) {
        return result;
    }

    const float effort = m_stepPlan[m_stepIndex].effort;
    const float absDelta = std::fabs(delta);
    if (absDelta < m_config.min_response_dps || delta * effort <= 0.0f) {
        return result;
    }

    const float maxAllowedDeviation = std::max(20.0f, absDelta * 0.35f);
    if (maxFinalDeviation > maxAllowedDeviation) {
        ESP_LOGW(TAG,
                 "Rejecting unstable step response: final deviation %.1f dps exceeds %.1f dps",
                 maxFinalDeviation,
                 maxAllowedDeviation);
        return result;
    }

    float t10 = duration_s;
    float t632 = duration_s;
    bool found10 = false;
    bool found632 = false;
    for (const auto& sample : m_stepSamples) {
        const float response = std::fabs(sample.speed_dps - m_stepBaseline_dps);
        if (!found10 && response >= absDelta * 0.10f) {
            t10 = sample.time_s;
            found10 = true;
        }
        if (!found632 && response >= absDelta * 0.632f) {
            t632 = sample.time_s;
            found632 = true;
        }
    }

    if (!found10 || !found632) {
        return result;
    }

    result.valid = true;
    result.gain_dps_per_effort = absDelta / std::fabs(effort);
    result.deadTime_s = std::max(t10, MIN_IDENTIFICATION_TIME_S);
    result.timeConstant_s = std::max(t632 - result.deadTime_s, MIN_IDENTIFICATION_TIME_S);
    result.steadySpeed_dps = steadySpeed;
    return result;
}

bool PidTuningService::averageStepResultsLocked(bool leftWheel, bool forward, StepResult& averageResult, std::string& error) const {
    float gainSum = 0.0f;
    float deadSum = 0.0f;
    float timeConstantSum = 0.0f;
    float steadySpeedSum = 0.0f;
    float minGain = 0.0f;
    float maxGain = 0.0f;
    float minDead = 0.0f;
    float maxDead = 0.0f;
    float minTimeConstant = 0.0f;
    float maxTimeConstant = 0.0f;
    int count = 0;

    for (size_t i = 0; i < m_stepPlan.size() && i < m_stepResults.size(); ++i) {
        const StepDefinition& step = m_stepPlan[i];
        const StepResult& result = m_stepResults[i];
        const bool stepForward = step.effort > 0.0f;
        if (step.leftWheel != leftWheel || stepForward != forward || !result.valid) {
            continue;
        }

        gainSum += result.gain_dps_per_effort;
        deadSum += result.deadTime_s;
        timeConstantSum += result.timeConstant_s;
        steadySpeedSum += result.steadySpeed_dps;
        if (count == 0) {
            minGain = maxGain = result.gain_dps_per_effort;
            minDead = maxDead = result.deadTime_s;
            minTimeConstant = maxTimeConstant = result.timeConstant_s;
        } else {
            minGain = std::min(minGain, result.gain_dps_per_effort);
            maxGain = std::max(maxGain, result.gain_dps_per_effort);
            minDead = std::min(minDead, result.deadTime_s);
            maxDead = std::max(maxDead, result.deadTime_s);
            minTimeConstant = std::min(minTimeConstant, result.timeConstant_s);
            maxTimeConstant = std::max(maxTimeConstant, result.timeConstant_s);
        }
        ++count;
    }

    if (count <= 0) {
        error = "Tuning aborted: missing motor response samples";
        return false;
    }

    if (count > 1) {
        if (minGain <= 0.0f || (maxGain / minGain) > MAX_RESPONSE_GAIN_RATIO) {
            error = "Tuning aborted: motor gain changed too much between effort levels";
            return false;
        }
        if (minDead <= 0.0f || (maxDead / minDead) > MAX_RESPONSE_TIME_RATIO ||
            minTimeConstant <= 0.0f || (maxTimeConstant / minTimeConstant) > MAX_RESPONSE_TIME_RATIO) {
            error = "Tuning aborted: motor timing changed too much between effort levels";
            return false;
        }
    }

    const float invCount = 1.0f / static_cast<float>(count);
    averageResult.valid = true;
    averageResult.gain_dps_per_effort = gainSum * invCount;
    averageResult.deadTime_s = deadSum * invCount;
    averageResult.timeConstant_s = timeConstantSum * invCount;
    averageResult.steadySpeed_dps = steadySpeedSum * invCount;
    return true;
}

bool PidTuningService::computeCandidateLocked(std::string& error) {
    for (const auto& result : m_stepResults) {
        if (!result.valid ||
            !std::isfinite(result.gain_dps_per_effort) ||
            !std::isfinite(result.deadTime_s) ||
            !std::isfinite(result.timeConstant_s)) {
            error = "Tuning aborted: incomplete step response";
            return false;
        }
    }

    const bool tuneLeft = isTargetLeftWheelLocked();
    StepResult forward;
    StepResult reverse;
    if (!averageStepResultsLocked(tuneLeft, true, forward, error) ||
        !averageStepResultsLocked(tuneLeft, false, reverse, error)) {
        if (error.empty()) {
            error = "Tuning aborted: missing averaged motor response";
        }
        return false;
    }

    auto directionsAreConsistent = [](const StepResult& forwardResult, const StepResult& reverseResult) -> bool {
        const float minGain = std::min(forwardResult.gain_dps_per_effort, reverseResult.gain_dps_per_effort);
        const float maxGain = std::max(forwardResult.gain_dps_per_effort, reverseResult.gain_dps_per_effort);
        return minGain > 0.0f && (maxGain / minGain) <= MAX_DIRECTION_GAIN_RATIO;
    };

    if (!directionsAreConsistent(forward, reverse)) {
        error = "Tuning aborted: forward/reverse motor gains are inconsistent";
        return false;
    }

    auto computePid = [&](const StepResult& forwardResult,
                          const StepResult& reverseResult,
                          PIDConfig base,
                          PidTuningResponseMetrics& metrics) -> PIDConfig {
        const float gain = averagePositive(forwardResult.gain_dps_per_effort, reverseResult.gain_dps_per_effort);
        const float dead = std::max(averagePositive(forwardResult.deadTime_s, reverseResult.deadTime_s), MIN_IDENTIFICATION_TIME_S);
        const float timeConstant = std::max(averagePositive(forwardResult.timeConstant_s, reverseResult.timeConstant_s), MIN_IDENTIFICATION_TIME_S);
        const float kp = m_config.gain_scale * 1.2f * timeConstant / (gain * dead);
        base.pid_kp = std::max(0.0f, std::min(500.0f, kp));
        base.pid_ki = std::max(0.0f, std::min(500.0f, base.pid_kp / (2.0f * dead)));
        base.pid_kd = std::max(0.0f, std::min(500.0f, base.pid_kp * 0.5f * dead));

        metrics.valid = true;
        metrics.forwardGain_dps_per_effort = forwardResult.gain_dps_per_effort;
        metrics.reverseGain_dps_per_effort = reverseResult.gain_dps_per_effort;
        metrics.deadTime_s = dead;
        metrics.timeConstant_s = timeConstant;
        metrics.steadySpeedForward_dps = forwardResult.steadySpeed_dps;
        metrics.steadySpeedReverse_dps = reverseResult.steadySpeed_dps;
        return base;
    };

    PidTuningResponseMetrics& metrics = tuneLeft ? m_status.leftMetrics : m_status.rightMetrics;
    PIDConfig candidate = computePid(forward,
                                     reverse,
                                     tuneLeft ? m_originalLeft : m_originalRight,
                                     metrics);

    if (!std::isfinite(candidate.pid_kp) ||
        !std::isfinite(candidate.pid_ki) ||
        !std::isfinite(candidate.pid_kd) ||
        candidate.pid_kp <= 1e-6f) {
        error = "Tuning aborted: computed invalid PID gains";
        return false;
    }

    if (tuneLeft) {
        m_candidateLeft = candidate;
    } else {
        m_candidateRight = candidate;
    }
    m_status.candidateLeft = m_candidateLeft;
    m_status.candidateRight = m_candidateRight;
    m_status.hasCandidate = true;
    return true;
}

bool PidTuningService::analyzeValidationLocked(float target_dps, const std::string& wheelName, std::string& error) const {
    if (m_validationFinalCount <= 0) {
        error = "Tuning aborted: no validation samples";
        return false;
    }

    const float averageSpeed = m_validationFinalSum_dps / static_cast<float>(m_validationFinalCount);
    if (averageSpeed * target_dps <= 0.0f ||
        std::fabs(averageSpeed) < std::max(m_config.min_response_dps, std::fabs(target_dps) * 0.25f)) {
        error = "Tuning aborted: " + wheelName + " validation response too weak";
        return false;
    }

    return true;
}

void PidTuningService::updateProgressLocked() {
    const float rest_s = m_config.rest_duration_ms / 1000.0f;
    const float step_s = m_config.step_duration_ms / 1000.0f;
    const float openLoopSteps = static_cast<float>(std::max<size_t>(1, m_stepPlan.size()));
    const float denom = std::max(0.1f, (openLoopSteps * (rest_s + step_s)) + step_s);
    float completed = 0.0f;

    switch (m_status.phase) {
        case PidTuningPhase::RESET:
            completed = 0.0f;
            break;
        case PidTuningPhase::REST_BEFORE_STEP:
            completed = static_cast<float>(m_stepIndex) * (rest_s + step_s) + std::min(m_phaseElapsed_s, rest_s);
            break;
        case PidTuningPhase::LEFT_FORWARD:
        case PidTuningPhase::LEFT_REVERSE:
        case PidTuningPhase::RIGHT_FORWARD:
        case PidTuningPhase::RIGHT_REVERSE:
            completed = static_cast<float>(m_stepIndex) * (rest_s + step_s) + rest_s + std::min(m_phaseElapsed_s, step_s);
            break;
        case PidTuningPhase::COMPUTE:
            completed = openLoopSteps * (rest_s + step_s);
            break;
        case PidTuningPhase::VALIDATE_LEFT:
            completed = openLoopSteps * (rest_s + step_s) + std::min(m_phaseElapsed_s, step_s);
            break;
        case PidTuningPhase::REST_BEFORE_VALIDATE_RIGHT:
            completed = openLoopSteps * (rest_s + step_s);
            break;
        case PidTuningPhase::VALIDATE_RIGHT:
            completed = openLoopSteps * (rest_s + step_s) + std::min(m_phaseElapsed_s, step_s);
            break;
        case PidTuningPhase::PREVIEW:
            completed = denom;
            break;
        default:
            completed = 0.0f;
            break;
    }

    m_status.progress = std::max(0.0f, std::min(1.0f, completed / denom));
}

void PidTuningService::setMessageLocked(const std::string& message) {
    m_status.message = message;
}

void PidTuningService::resetRuntimeLocked() {
    m_cancelRequested = false;
    m_resetDone = false;
    m_stepIndex = 0;
    m_phaseElapsed_s = 0.0f;
    m_stepBaseline_dps = 0.0f;
    m_stepPlan.clear();
    m_stepSamples.clear();
    m_stepResults.clear();
    m_validationFinalSum_dps = 0.0f;
    m_validationFinalCount = 0;
    m_lastSpeedSetpointLeft_dps = 0.0f;
    m_lastSpeedSetpointRight_dps = 0.0f;
    m_status.hasCandidate = false;
    m_status.candidateLeft = PIDConfig();
    m_status.candidateRight = PIDConfig();
    m_status.leftMetrics = PidTuningResponseMetrics();
    m_status.rightMetrics = PidTuningResponseMetrics();
}

void PidTuningService::applyPreview() {
    PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
    PIDConfig candidate;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        target = m_status.target;
        candidate = isLeftWheelTarget(target) ? m_candidateLeft : m_candidateRight;
    }
    (void)m_configService.applyPidConfig(isLeftWheelTarget(target) ? "speed_left" : "speed_right",
                                         candidate,
                                         false);
}

void PidTuningService::publishFinished(PidTuningState state, const std::string& message) {
    PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        target = m_status.target;
    }
    PID_TuningFinished event(target, state, message);
    m_eventBus.publish(event);
}

bool PidTuningService::isTargetLeftWheelLocked() const {
    return isLeftWheelTarget(m_status.target);
}

const char* PidTuningService::targetWheelNameLocked() const {
    return isTargetLeftWheelLocked() ? "left" : "right";
}

const char* PidTuningService::phaseToString(PidTuningPhase phase) {
    switch (phase) {
        case PidTuningPhase::RESET: return "reset";
        case PidTuningPhase::REST_BEFORE_STEP: return "rest";
        case PidTuningPhase::LEFT_FORWARD: return "left forward";
        case PidTuningPhase::LEFT_REVERSE: return "left reverse";
        case PidTuningPhase::RIGHT_FORWARD: return "right forward";
        case PidTuningPhase::RIGHT_REVERSE: return "right reverse";
        case PidTuningPhase::COMPUTE: return "compute";
        case PidTuningPhase::VALIDATE_LEFT: return "validate left";
        case PidTuningPhase::REST_BEFORE_VALIDATE_RIGHT: return "rest before right validation";
        case PidTuningPhase::VALIDATE_RIGHT: return "validate right";
        case PidTuningPhase::PREVIEW: return "preview";
        case PidTuningPhase::IDLE:
        default: return "idle";
    }
}

float PidTuningService::clampAbs(float value, float limit) {
    return std::max(-limit, std::min(limit, value));
}

bool PidTuningService::isLeftWheelTarget(PidTuningTarget target) {
    return target == PidTuningTarget::MOTOR_SPEED_LEFT;
}
