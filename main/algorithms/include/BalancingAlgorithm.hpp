
#pragma once

#include "BalanceControlTypes.hpp"
#include "ConfigData.hpp" // Include full definition
#include "EventBus.hpp"
#include "EventHandler.hpp" // Include for EventHandler base class
#include "IBalanceControlStrategy.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "esp_log.h"
#include <memory>
#include <mutex>

// Forward declarations for event classes
class CONFIG_PidConfigUpdate;
class CONFIG_FullConfigUpdate;

class BalancingAlgorithm : public EventHandler {
public:
    // Constructor takes initial config structs
    BalancingAlgorithm(EventBus& eventBus,
                       const PIDConfig& initialAnglePid,
                       const PIDConfig& initialSpeedLeftPid,
                       const PIDConfig& initialSpeedRightPid,
                       const PIDConfig& initialYawAnglePid,
                       const PIDConfig& initialYawRatePid,
                       const ControlConfig& initialControl,
                       const EncoderConfig& initialEncoder,
                       const RobotDimensionsConfig& initialDimensions);

    esp_err_t init();
    MotorEffort update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                      float currentYaw_deg,
                      float currentYawRate_dps,
                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                      float targetPitchOffset_deg, float targetAngVel_dps,
                      const LongitudinalOdometryResult& odometry,
                      int64_t nowUs = 0,
                      int64_t motionTimeoutUs = 0,
                      const LongitudinalMotionCommand& motion = {});
    void resetState();

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return "BalancingAlgorithm"; }

    // --- Getters for Telemetry ---
    float getLastSpeedSetpointLeftDPS() const;
    float getLastSpeedSetpointRightDPS() const;
    float getLastTargetYawDeg() const;
    float getLastDesiredYawRateDPS() const;
    bool isYawControlEnabled() const;
    BalanceControlDiagnostics getDiagnostics() const;
    BalanceStrategyId getActiveStrategyId() const;
    // --- End Getters ---

private:
    static constexpr const char* TAG = "BalancingAlgo";
    EventBus& m_eventBus;
    mutable std::mutex m_strategyMutex;
    std::unique_ptr<IBalanceControlStrategy> m_strategy;
    BalanceStrategyId m_activeStrategyId = BalanceStrategyId::NESTED_PID;
    ControlRunMode m_controlMode = ControlRunMode::DISABLED;
    uint64_t m_controlArmId = 0;

    // Internal helpers to apply config from events
    void applyConfig(const ConfigData& config);
    std::unique_ptr<IBalanceControlStrategy> createStrategy(BalanceStrategyId id) const;
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event);
    void handleRunModeChanged(const CONTROL_RunModeChanged& event);
};
