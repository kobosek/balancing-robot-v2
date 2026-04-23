#include "GuidedCalibrationService.hpp"

#include "CONFIG_FullConfigUpdate.hpp"
#include "EventBus.hpp"
#include "GUIDED_CalibrationFinished.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "UI_CancelGuidedCalibration.hpp"
#include "UI_StartGuidedCalibration.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr float STILLNESS_DURATION_S = 1.2f;
constexpr float DIRECTION_DURATION_S = 0.8f;
constexpr float DEADZONE_SWEEP_DURATION_S = 2.0f;
constexpr float SUMMARY_DURATION_S = 0.3f;
constexpr float SAFE_PITCH_DEG = 25.0f;
constexpr float STILL_PITCH_DEG = 8.0f;
constexpr float SETTLED_SPEED_DPS = 20.0f;
constexpr float MIN_DIRECTION_RESPONSE_DPS = 25.0f;
}

GuidedCalibrationService::GuidedCalibrationService(EventBus& eventBus,
                                                   const PidTuningConfig& initialTuningConfig,
                                                   const MotorConfig& initialMotorConfig)
    : m_eventBus(eventBus),
      m_tuningConfig(initialTuningConfig),
      m_motorConfig(initialMotorConfig) {
    applyConfig(initialTuningConfig, initialMotorConfig);
}

esp_err_t GuidedCalibrationService::init() {
    ESP_LOGI(TAG, "Guided calibration service initialized");
    return ESP_OK;
}

GuidedCalibrationStatus GuidedCalibrationService::getStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_status;
}

void GuidedCalibrationService::handleEvent(const BaseEvent& event) {
    if (event.is<UI_StartGuidedCalibration>()) {
        handleStartCommand(event.as<UI_StartGuidedCalibration>());
    } else if (event.is<UI_CancelGuidedCalibration>()) {
        handleCancelCommand(event.as<UI_CancelGuidedCalibration>());
    } else if (event.is<SYSTEM_StateChanged>()) {
        handleSystemStateChanged(event.as<SYSTEM_StateChanged>());
    } else if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    }
}

MotorEffort GuidedCalibrationService::update(float dt, const GuidedCalibrationSample& sample) {
    MotorEffort effort = {0.0f, 0.0f};
    bool shouldPublishFinished = false;
    bool finishedSuccess = false;
    std::string finishedMessage;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_status.state != GuidedCalibrationState::RUNNING || dt <= 0.0f) {
            return effort;
        }

        if (m_cancelRequested) {
            cancelRunLocked("Guided calibration canceled");
            shouldPublishFinished = true;
            finishedSuccess = false;
            finishedMessage = m_status.message;
        } else if (std::fabs(sample.pitch_deg) > SAFE_PITCH_DEG) {
            failRunLocked("Guided calibration aborted: robot tilted too far");
            shouldPublishFinished = true;
            finishedSuccess = false;
            finishedMessage = m_status.message;
        }

        if (shouldPublishFinished) {
            // Leave motors off.
        } else {
            m_phaseElapsed_s += dt;
            updateProgressLocked();

            switch (m_status.phase) {
                case GuidedCalibrationPhase::IMU_STILLNESS:
                    if (std::fabs(sample.pitch_deg) > STILL_PITCH_DEG ||
                        std::fabs(sample.speedLeft_dps) > SETTLED_SPEED_DPS ||
                        std::fabs(sample.speedRight_dps) > SETTLED_SPEED_DPS) {
                        m_phaseElapsed_s = 0.0f;
                        m_status.message = "Hold robot upright and keep wheels still";
                    } else if (m_phaseElapsed_s >= STILLNESS_DURATION_S) {
                        transitionToPhaseLocked(GuidedCalibrationPhase::LEFT_DIRECTION, "Checking left motor direction");
                    }
                    break;

                case GuidedCalibrationPhase::LEFT_DIRECTION:
                    effort.left = clampAbs(m_tuningConfig.step_effort, m_tuningConfig.max_effort);
                    m_phaseMaxSignedSpeed_dps = sample.speedLeft_dps;
                    m_phaseMaxAbsSpeed_dps = std::max(m_phaseMaxAbsSpeed_dps, std::fabs(sample.speedLeft_dps));
                    if (m_phaseElapsed_s >= DIRECTION_DURATION_S) {
                        m_status.leftDirectionOk = m_phaseMaxAbsSpeed_dps >= MIN_DIRECTION_RESPONSE_DPS && m_phaseMaxSignedSpeed_dps > 0.0f;
                        if (!m_status.leftDirectionOk) {
                            failRunLocked("Left motor/encoder direction check failed");
                            shouldPublishFinished = true;
                            finishedSuccess = false;
                            finishedMessage = m_status.message;
                        } else {
                            transitionToPhaseLocked(GuidedCalibrationPhase::RIGHT_DIRECTION, "Checking right motor direction");
                        }
                    }
                    break;

                case GuidedCalibrationPhase::RIGHT_DIRECTION:
                    effort.right = clampAbs(m_tuningConfig.step_effort, m_tuningConfig.max_effort);
                    m_phaseMaxSignedSpeed_dps = sample.speedRight_dps;
                    m_phaseMaxAbsSpeed_dps = std::max(m_phaseMaxAbsSpeed_dps, std::fabs(sample.speedRight_dps));
                    if (m_phaseElapsed_s >= DIRECTION_DURATION_S) {
                        m_status.rightDirectionOk = m_phaseMaxAbsSpeed_dps >= MIN_DIRECTION_RESPONSE_DPS && m_phaseMaxSignedSpeed_dps > 0.0f;
                        if (!m_status.rightDirectionOk) {
                            failRunLocked("Right motor/encoder direction check failed");
                            shouldPublishFinished = true;
                            finishedSuccess = false;
                            finishedMessage = m_status.message;
                        } else {
                            transitionToPhaseLocked(GuidedCalibrationPhase::LEFT_DEADZONE, "Sweeping left motor deadzone");
                        }
                    }
                    break;

                case GuidedCalibrationPhase::LEFT_DEADZONE:
                    effort.left = currentSweepEffortLocked();
                    if (m_status.leftDeadzoneEffort <= 0.0f &&
                        std::fabs(sample.speedLeft_dps) >= MIN_DIRECTION_RESPONSE_DPS) {
                        m_status.leftDeadzoneEffort = effort.left;
                    }
                    if (m_phaseElapsed_s >= DEADZONE_SWEEP_DURATION_S) {
                        transitionToPhaseLocked(GuidedCalibrationPhase::RIGHT_DEADZONE, "Sweeping right motor deadzone");
                    }
                    break;

                case GuidedCalibrationPhase::RIGHT_DEADZONE:
                    effort.right = currentSweepEffortLocked();
                    if (m_status.rightDeadzoneEffort <= 0.0f &&
                        std::fabs(sample.speedRight_dps) >= MIN_DIRECTION_RESPONSE_DPS) {
                        m_status.rightDeadzoneEffort = effort.right;
                    }
                    if (m_phaseElapsed_s >= DEADZONE_SWEEP_DURATION_S) {
                        transitionToPhaseLocked(GuidedCalibrationPhase::SUMMARY, "Finishing guided calibration");
                    }
                    break;

                case GuidedCalibrationPhase::SUMMARY:
                    if (m_phaseElapsed_s >= SUMMARY_DURATION_S) {
                        completeRunLocked();
                        shouldPublishFinished = true;
                        finishedSuccess = true;
                        finishedMessage = m_status.message;
                    }
                    break;

                default:
                    break;
            }
        }
    }

    if (shouldPublishFinished) {
        publishFinished(finishedSuccess, finishedMessage);
        return {0.0f, 0.0f};
    }

    return effort;
}

void GuidedCalibrationService::applyConfig(const PidTuningConfig& tuningConfig, const MotorConfig& motorConfig) {
    m_tuningConfig = tuningConfig;
    m_motorConfig = motorConfig;
}

void GuidedCalibrationService::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    applyConfig(event.configData.pid_tuning, event.configData.motor);
}

void GuidedCalibrationService::handleSystemStateChanged(const SYSTEM_StateChanged& event) {
    bool shouldPublishCanceled = false;
    std::string message;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_systemState = event.newState;
        if (event.newState == SystemState::GUIDED_CALIBRATION) {
            beginRunLocked();
            return;
        }
        if (event.previousState == SystemState::GUIDED_CALIBRATION &&
            m_status.state == GuidedCalibrationState::RUNNING) {
            cancelRunLocked("Guided calibration stopped");
            shouldPublishCanceled = true;
            message = m_status.message;
        }
    }

    if (shouldPublishCanceled) {
        publishFinished(false, message);
    }
}

void GuidedCalibrationService::handleStartCommand(const UI_StartGuidedCalibration& event) {
    (void)event;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_startRequested = true;
}

void GuidedCalibrationService::handleCancelCommand(const UI_CancelGuidedCalibration& event) {
    (void)event;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_status.state == GuidedCalibrationState::RUNNING) {
        m_cancelRequested = true;
    }
}

void GuidedCalibrationService::beginRunLocked() {
    (void)m_startRequested;
    m_status = GuidedCalibrationStatus();
    m_status.state = GuidedCalibrationState::RUNNING;
    transitionToPhaseLocked(GuidedCalibrationPhase::IMU_STILLNESS, "Hold robot upright and keep wheels still");
    m_startRequested = false;
    m_cancelRequested = false;
}

void GuidedCalibrationService::cancelRunLocked(const std::string& message) {
    m_status.state = GuidedCalibrationState::CANCELED;
    m_status.phase = GuidedCalibrationPhase::IDLE;
    m_status.progress = 0.0f;
    m_status.message = message;
    m_cancelRequested = false;
}

void GuidedCalibrationService::failRunLocked(const std::string& message) {
    m_status.state = GuidedCalibrationState::FAILED;
    m_status.phase = GuidedCalibrationPhase::IDLE;
    m_status.progress = 0.0f;
    m_status.message = message;
    m_cancelRequested = false;
    ESP_LOGW(TAG, "%s", message.c_str());
}

void GuidedCalibrationService::completeRunLocked() {
    m_status.state = GuidedCalibrationState::COMPLETE;
    m_status.phase = GuidedCalibrationPhase::SUMMARY;
    m_status.progress = 1.0f;
    m_status.message = "Guided calibration complete";
}

void GuidedCalibrationService::transitionToPhaseLocked(GuidedCalibrationPhase phase, const std::string& message) {
    m_status.phase = phase;
    m_status.message = message;
    m_phaseElapsed_s = 0.0f;
    m_phaseMaxAbsSpeed_dps = 0.0f;
    m_phaseMaxSignedSpeed_dps = 0.0f;
    updateProgressLocked();
    ESP_LOGI(TAG, "Guided calibration phase: %s", phaseToString(phase));
}

void GuidedCalibrationService::updateProgressLocked() {
    float base = 0.0f;
    float span = 0.0f;
    float duration = 1.0f;

    switch (m_status.phase) {
        case GuidedCalibrationPhase::IMU_STILLNESS:
            base = 0.00f; span = 0.20f; duration = STILLNESS_DURATION_S; break;
        case GuidedCalibrationPhase::LEFT_DIRECTION:
            base = 0.20f; span = 0.15f; duration = DIRECTION_DURATION_S; break;
        case GuidedCalibrationPhase::RIGHT_DIRECTION:
            base = 0.35f; span = 0.15f; duration = DIRECTION_DURATION_S; break;
        case GuidedCalibrationPhase::LEFT_DEADZONE:
            base = 0.50f; span = 0.20f; duration = DEADZONE_SWEEP_DURATION_S; break;
        case GuidedCalibrationPhase::RIGHT_DEADZONE:
            base = 0.70f; span = 0.20f; duration = DEADZONE_SWEEP_DURATION_S; break;
        case GuidedCalibrationPhase::SUMMARY:
            base = 0.90f; span = 0.10f; duration = SUMMARY_DURATION_S; break;
        default:
            m_status.progress = 0.0f;
            return;
    }

    m_status.progress = std::clamp(base + span * std::min(1.0f, m_phaseElapsed_s / duration), 0.0f, 1.0f);
}

void GuidedCalibrationService::publishFinished(bool success, const std::string& message) {
    GUIDED_CalibrationFinished event(success, message);
    m_eventBus.publish(event);
}

float GuidedCalibrationService::currentSweepEffortLocked() const {
    constexpr float MIN_SWEEP_EFFORT = 0.04f;
    const float maxEffort = std::max(MIN_SWEEP_EFFORT, std::min(m_tuningConfig.max_effort, 0.6f));
    const float ratio = std::clamp(m_phaseElapsed_s / DEADZONE_SWEEP_DURATION_S, 0.0f, 1.0f);
    return MIN_SWEEP_EFFORT + (maxEffort - MIN_SWEEP_EFFORT) * ratio;
}

const char* GuidedCalibrationService::phaseToString(GuidedCalibrationPhase phase) {
    switch (phase) {
        case GuidedCalibrationPhase::IDLE: return "IDLE";
        case GuidedCalibrationPhase::IMU_STILLNESS: return "IMU_STILLNESS";
        case GuidedCalibrationPhase::LEFT_DIRECTION: return "LEFT_DIRECTION";
        case GuidedCalibrationPhase::RIGHT_DIRECTION: return "RIGHT_DIRECTION";
        case GuidedCalibrationPhase::LEFT_DEADZONE: return "LEFT_DEADZONE";
        case GuidedCalibrationPhase::RIGHT_DEADZONE: return "RIGHT_DEADZONE";
        case GuidedCalibrationPhase::SUMMARY: return "SUMMARY";
        default: return "UNKNOWN";
    }
}

float GuidedCalibrationService::clampAbs(float value, float limit) {
    return std::max(-limit, std::min(limit, value));
}
