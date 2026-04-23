#pragma once

#include "ConfigData.hpp"
#include "EventHandler.hpp"
#include "BalancingAlgorithm.hpp"
#include "SystemState.hpp"
#include "esp_err.h"
#include <mutex>
#include <string>

class EventBus;
class BaseEvent;
class CONFIG_FullConfigUpdate;
class SYSTEM_StateChanged;
class UI_StartGuidedCalibration;
class UI_CancelGuidedCalibration;

enum class GuidedCalibrationState {
    IDLE,
    RUNNING,
    COMPLETE,
    FAILED,
    CANCELED
};

enum class GuidedCalibrationPhase {
    IDLE,
    IMU_STILLNESS,
    LEFT_DIRECTION,
    RIGHT_DIRECTION,
    LEFT_DEADZONE,
    RIGHT_DEADZONE,
    SUMMARY
};

struct GuidedCalibrationStatus {
    GuidedCalibrationState state = GuidedCalibrationState::IDLE;
    GuidedCalibrationPhase phase = GuidedCalibrationPhase::IDLE;
    float progress = 0.0f;
    std::string message = "Idle";
    bool leftDirectionOk = false;
    bool rightDirectionOk = false;
    float leftDeadzoneEffort = 0.0f;
    float rightDeadzoneEffort = 0.0f;
};

struct GuidedCalibrationSample {
    float pitch_deg = 0.0f;
    float speedLeft_dps = 0.0f;
    float speedRight_dps = 0.0f;
};

class GuidedCalibrationService : public EventHandler {
public:
    GuidedCalibrationService(EventBus& eventBus,
                             const PidTuningConfig& initialTuningConfig,
                             const MotorConfig& initialMotorConfig);

    esp_err_t init();
    GuidedCalibrationStatus getStatus() const;

    MotorEffort update(float dt, const GuidedCalibrationSample& sample);

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    static constexpr const char* TAG = "GuidedCal";

    EventBus& m_eventBus;
    mutable std::mutex m_mutex;
    GuidedCalibrationStatus m_status;
    PidTuningConfig m_tuningConfig;
    MotorConfig m_motorConfig;
    SystemState m_systemState = SystemState::INIT;
    bool m_startRequested = false;
    bool m_cancelRequested = false;
    float m_phaseElapsed_s = 0.0f;
    float m_phaseMaxAbsSpeed_dps = 0.0f;
    float m_phaseMaxSignedSpeed_dps = 0.0f;

    void applyConfig(const PidTuningConfig& tuningConfig, const MotorConfig& motorConfig);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handleSystemStateChanged(const SYSTEM_StateChanged& event);
    void handleStartCommand(const UI_StartGuidedCalibration& event);
    void handleCancelCommand(const UI_CancelGuidedCalibration& event);

    void beginRunLocked();
    void cancelRunLocked(const std::string& message);
    void failRunLocked(const std::string& message);
    void completeRunLocked();
    void transitionToPhaseLocked(GuidedCalibrationPhase phase, const std::string& message);
    void updateProgressLocked();
    void publishFinished(bool success, const std::string& message);

    float currentSweepEffortLocked() const;
    static const char* phaseToString(GuidedCalibrationPhase phase);
    static float clampAbs(float value, float limit);
};
