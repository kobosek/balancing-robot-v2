#pragma once

#include <mutex>
#include "esp_err.h"
#include "EventHandler.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include <memory>

// Forward Declarations
class OrientationEstimator;
class EncoderService;
class MotorService;
class BatteryService;
class ControlEventDispatcher;
class ControlModeExecutor;
class BaseEvent;
class MOTION_TargetMovement;
struct MotorEffort;
struct TelemetryDataPoint;
struct ControlModeResult;

class RobotController : public EventHandler {
public:
    RobotController(
        std::shared_ptr<OrientationEstimator> estimator,
        EncoderService& encoderService,
        MotorService& motorService,
        BatteryService& batteryService,
        ControlModeExecutor& controlModeExecutor,
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
    BatteryService& m_batteryService;
    ControlModeExecutor& m_controlModeExecutor;
    ControlEventDispatcher& m_controlEventDispatcher;

    // Latest command values (thread-safe)
    float m_latestTargetPitchOffset_deg = 0.0f;
    float m_latestTargetAngVel_dps = 0.0f;
    std::mutex m_target_values_mutex;
    ControlRunMode m_controlMode = ControlRunMode::DISABLED;
    int m_telemetryStateCode = 0;
    bool m_telemetryEnabled = false;
    std::mutex m_control_mode_mutex;

    void stopControlLoop();
    TelemetryDataPoint buildTelemetrySnapshot(int64_t timestamp_us,
                                              int telemetryStateCode,
                                              float pitch_deg,
                                              float yaw_rate_dps,
                                              float speedL_dps,
                                              float speedR_dps,
                                              const ControlModeResult& modeResult) const;
    void handleTargetMovementCommand(const MOTION_TargetMovement& event);
    void handleControlRunModeChanged(const CONTROL_RunModeChanged& event);
};
