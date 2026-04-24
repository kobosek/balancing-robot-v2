#include "PidTuningService.hpp"

#include "ConfigurationService.hpp"
#include "EncoderService.hpp"
#include "EventBus.hpp"
#include "PID_TuningFinished.hpp"
#include "PID_TuningRunModeChanged.hpp"
#include "UI_CancelPidTuning.hpp"
#include "UI_DiscardPidTuning.hpp"
#include "UI_SavePidTuning.hpp"
#include "UI_StartPidTuning.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float MIN_EXTRA_STEP_DELTA = 0.03f;
constexpr float MAX_SETTLE_WAIT_S = 1.5f;
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
    if (event.is<UI_StartPidTuning>()) {
        handleStartCommand(event.as<UI_StartPidTuning>());
    } else if (event.is<UI_CancelPidTuning>()) {
        handleCancelCommand(event.as<UI_CancelPidTuning>());
    } else if (event.is<UI_SavePidTuning>()) {
        handleSaveCommand(event.as<UI_SavePidTuning>());
    } else if (event.is<UI_DiscardPidTuning>()) {
        handleDiscardCommand(event.as<UI_DiscardPidTuning>());
    } else if (event.is<PID_TuningRunModeChanged>()) {
        handleRunModeChanged(event.as<PID_TuningRunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
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

void PidTuningService::handleRunModeChanged(const PID_TuningRunModeChanged& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (event.active) {
        const PidTuningTarget target = m_startRequested ? m_requestedTarget : PidTuningTarget::MOTOR_SPEED_LEFT;
        beginRunLocked(target);
        m_startRequested = false;
        return;
    }

    if (m_status.state == PidTuningState::RUNNING) {
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
    if (m_stepIndex >= m_stepPlan.size()) {
        return StepResult();
    }

    return PidTuningStepAnalyzer::analyzeStep(m_stepSamples,
                                              m_stepBaseline_dps,
                                              m_stepPlan[m_stepIndex].effort,
                                              m_config);
}

bool PidTuningService::computeCandidateLocked(std::string& error) {
    const bool tuneLeft = isTargetLeftWheelLocked();
    const PidTuningCandidateResult result = PidTuningCandidateCalculator::compute(
        m_stepPlan,
        m_stepResults,
        tuneLeft,
        tuneLeft ? m_originalLeft : m_originalRight,
        m_config);
    if (!result.success) {
        error = result.error;
        return false;
    }

    if (tuneLeft) {
        m_candidateLeft = result.candidate;
        m_status.leftMetrics = result.metrics;
    } else {
        m_candidateRight = result.candidate;
        m_status.rightMetrics = result.metrics;
    }
    m_status.candidateLeft = m_candidateLeft;
    m_status.candidateRight = m_candidateRight;
    m_status.hasCandidate = true;
    return true;
}

bool PidTuningService::analyzeValidationLocked(float target_dps, const std::string& wheelName, std::string& error) const {
    return PidTuningStepAnalyzer::analyzeValidation(m_validationFinalSum_dps,
                                                    m_validationFinalCount,
                                                    target_dps,
                                                    m_config.min_response_dps,
                                                    wheelName,
                                                    error);
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
