#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for PIDConfig struct

class CONFIG_PidConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_PidConfigUpdate)
    const std::string pidName;  // "angle", "speed_left", "speed_right", "yaw_angle", "yaw_rate"
    const PIDConfig config;
    // Granular updates are emitted from a complete configuration transaction.
    // Carrying the owning strategy and both revision scopes prevents a delayed
    // event for an inactive/older controller from mutating the active runtime.
    const BalanceStrategyId strategyId;
    const uint32_t configRevision;
    const uint32_t strategyRevision;

    CONFIG_PidConfigUpdate(const std::string& name,
                           const PIDConfig& data,
                           BalanceStrategyId strategy = BalanceStrategyId::NESTED_PID,
                           uint32_t documentRevision = 0,
                           uint32_t ownerRevision = 0) :
        BaseEvent(),
        pidName(name),
        config(data),
        strategyId(strategy),
        configRevision(documentRevision),
        strategyRevision(ownerRevision) {}
};

