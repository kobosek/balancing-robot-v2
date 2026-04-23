#pragma once

#include <mutex>
#include "esp_err.h"
#include "EventHandler.hpp"
#include "SystemState.hpp"
#include <memory>

// Forward Declarations
class OrientationEstimator;
class BalancingAlgorithm;
class EncoderService;
class MotorService;
class BatteryService;
class StateManager;
class PidTuningService;
class GuidedCalibrationService;
class ControlEventDispatcher;
class BaseEvent;
class MOTION_TargetMovement;
struct MotorEffort;
struct TelemetryDataPoint;
struct GuidedCalibrationSample;

class RobotController : public EventHandler {
public:
    RobotController(
        std::shared_ptr<OrientationEstimator> estimator,
        EncoderService& encoderService,
        MotorService& motorService,
        BalancingAlgorithm& algorithm,
        StateManager& stateManager,
        BatteryService& batteryService,
        PidTuningService& pidTuningService,
        GuidedCalibrationService& guidedCalibrationService,
        ControlEventDispatcher& controlEventDispatcher
    );

    void runControlStep(float dt);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    static constexpr const char* TAG = "RobotController";

    // References to components
    std::shared_ptr<OrientationEstimator> m_estimator;
    EncoderService& m_encoderService;
    MotorService& m_motorService;
    BalancingAlgorithm& m_algorithm;
    StateManager& m_stateManager;
    BatteryService& m_batteryService;
    PidTuningService& m_pidTuningService;
    GuidedCalibrationService& m_guidedCalibrationService;
    ControlEventDispatcher& m_controlEventDispatcher;

    // Latest command values (thread-safe)
    float m_latestTargetPitchOffset_deg = 0.0f;
    float m_latestTargetAngVel_dps = 0.0f;
    std::mutex m_target_values_mutex;

    void stopControlLoop();
    MotorEffort executeControlMode(SystemState currentState,
                                   float dt,
                                   float pitch_deg,
                                   float pitch_rate_dps,
                                   float yaw_rate_dps,
                                   float speedL_dps,
                                   float speedR_dps,
                                   float& currentTargetPitchOffset_deg,
                                   float& currentTargetAngVel_dps);
    MotorEffort executeGuidedCalibrationMode(float dt,
                                             float pitch_deg,
                                             float speedL_dps,
                                             float speedR_dps) const;
    TelemetryDataPoint buildTelemetrySnapshot(int64_t timestamp_us,
                                              SystemState currentState,
                                              float pitch_deg,
                                              float yaw_rate_dps,
                                              float speedL_dps,
                                              float speedR_dps,
                                              float currentTargetPitchOffset_deg) const;
    void handleTargetMovementCommand(const MOTION_TargetMovement& event);
};
