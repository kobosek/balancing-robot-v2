#pragma once

#include "BalancingAlgorithm.hpp"
#include "EventHandler.hpp"
#include "PIDController.hpp"
#include "PidTuningTypes.hpp"
#include "config/PidTuningConfig.hpp"
#include "esp_err.h"
#include <mutex>
#include <string>
#include <vector>

class BaseEvent;
class ConfigurationService;
class EncoderService;
class EventBus;
class PID_TuningRunModeChanged;
class UI_CancelPidTuning;
class UI_DiscardPidTuning;
class UI_SavePidTuning;
class UI_StartPidTuning;

struct PidTuningResponseMetrics {
    bool valid = false;
    float forwardGain_dps_per_effort = 0.0f;
    float reverseGain_dps_per_effort = 0.0f;
    float deadTime_s = 0.0f;
    float timeConstant_s = 0.0f;
    float steadySpeedForward_dps = 0.0f;
    float steadySpeedReverse_dps = 0.0f;
};

struct PidTuningStatus {
    PidTuningState state = PidTuningState::IDLE;
    PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
    PidTuningPhase phase = PidTuningPhase::IDLE;
    float progress = 0.0f;
    std::string message = "Idle";
    bool hasCandidate = false;
    PIDConfig candidateLeft;
    PIDConfig candidateRight;
    PidTuningResponseMetrics leftMetrics;
    PidTuningResponseMetrics rightMetrics;
};

class PidTuningService : public EventHandler {
public:
    PidTuningService(EventBus& eventBus,
                     ConfigurationService& configService,
                     EncoderService& encoderService,
                     const PidTuningConfig& initialConfig);

    esp_err_t init();
    MotorEffort update(float dt, float speedLeft_dps, float speedRight_dps);

    PidTuningStatus getStatus() const;
    float getLastSpeedSetpointLeftDPS() const { return m_lastSpeedSetpointLeft_dps; }
    float getLastSpeedSetpointRightDPS() const { return m_lastSpeedSetpointRight_dps; }

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    struct StepSample {
        float time_s = 0.0f;
        float speed_dps = 0.0f;
    };

    struct StepResult {
        bool valid = false;
        float gain_dps_per_effort = 0.0f;
        float deadTime_s = 0.0f;
        float timeConstant_s = 0.0f;
        float steadySpeed_dps = 0.0f;
    };

    struct StepDefinition {
        bool leftWheel = true;
        float effort = 0.0f;
        PidTuningPhase phase = PidTuningPhase::IDLE;
    };

    static constexpr const char* TAG = "PidTuning";

    EventBus& m_eventBus;
    ConfigurationService& m_configService;
    EncoderService& m_encoderService;
    PidTuningConfig m_config;

    mutable std::mutex m_mutex;
    PidTuningStatus m_status;

    bool m_startRequested = false;
    PidTuningTarget m_requestedTarget = PidTuningTarget::MOTOR_SPEED_LEFT;
    bool m_cancelRequested = false;
    bool m_resetDone = false;
    bool m_previewAvailable = false;

    PIDConfig m_originalLeft;
    PIDConfig m_originalRight;
    PIDConfig m_candidateLeft;
    PIDConfig m_candidateRight;

    PIDController m_validationPidLeft;
    PIDController m_validationPidRight;

    size_t m_stepIndex = 0;
    float m_phaseElapsed_s = 0.0f;
    float m_stepBaseline_dps = 0.0f;
    std::vector<StepDefinition> m_stepPlan;
    std::vector<StepSample> m_stepSamples;
    std::vector<StepResult> m_stepResults;
    float m_validationFinalSum_dps = 0.0f;
    int m_validationFinalCount = 0;

    float m_lastSpeedSetpointLeft_dps = 0.0f;
    float m_lastSpeedSetpointRight_dps = 0.0f;

    MotorEffort updateLocked(float dt,
                             float speedLeft_dps,
                             float speedRight_dps,
                             bool& shouldPublishFinished,
                             PidTuningState& finishedState,
                             std::string& finishedMessage,
                             bool& shouldApplyPreview);

    void handleStartCommand(const UI_StartPidTuning& event);
    void handleCancelCommand(const UI_CancelPidTuning& event);
    void handleSaveCommand(const UI_SavePidTuning& event);
    void handleDiscardCommand(const UI_DiscardPidTuning& event);
    void handleRunModeChanged(const PID_TuningRunModeChanged& event);

    void beginRunLocked(PidTuningTarget target);
    void cancelRunLocked(const std::string& message);
    void failRunLocked(const std::string& message);
    void completeRunLocked();

    void transitionToRestLocked();
    void transitionToStepLocked(float currentSpeed_dps);
    void transitionToComputeLocked();
    void transitionToValidateLeftLocked();
    void transitionToValidateRightLocked();

    void buildStepPlanLocked();
    MotorEffort currentStepEffortLocked() const;
    float currentStepActiveSpeed(float left_dps, float right_dps) const;
    PidTuningPhase currentStepPhaseLocked() const;
    StepResult analyzeCurrentStepLocked() const;
    bool averageStepResultsLocked(bool leftWheel, bool forward, StepResult& averageResult, std::string& error) const;
    bool computeCandidateLocked(std::string& error);
    bool analyzeValidationLocked(float target_dps, const std::string& wheelName, std::string& error) const;
    bool isTargetLeftWheelLocked() const;
    const char* targetWheelNameLocked() const;

    void updateProgressLocked();
    void setMessageLocked(const std::string& message);
    void resetRuntimeLocked();
    void applyPreview();
    void publishFinished(PidTuningState state, const std::string& message);

    static const char* phaseToString(PidTuningPhase phase);
    static float clampAbs(float value, float limit);
    static bool isLeftWheelTarget(PidTuningTarget target);
};
