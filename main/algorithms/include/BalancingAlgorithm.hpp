
#pragma once

#include "BalanceControlTypes.hpp"
#include "ConfigData.hpp" // Include full definition
#include "EventBus.hpp"
#include "EventHandler.hpp" // Include for EventHandler base class
#include "IBalanceControlStrategy.hpp"
#include "esp_log.h"
#include <memory>

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
                      float targetPitchOffset_deg, float targetAngVel_dps);
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
    // --- End Getters ---

private:
    static constexpr const char* TAG = "BalancingAlgo";
    EventBus& m_eventBus;
    std::unique_ptr<IBalanceControlStrategy> m_strategy;

    // Internal helpers to apply config from events
    void applyConfig(const ConfigData& config);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event);
};
