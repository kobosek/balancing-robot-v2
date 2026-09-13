#pragma once

#include "ConfigData.hpp"
#include <string>

struct BalanceStrategyCapability {
    BalanceStrategyId strategyId = BalanceStrategyId::NESTED_PID;
    bool configured = true;
    bool canActivate = false;
    bool active = false;
    LongitudinalLoopMode loopMode = LongitudinalLoopMode::PITCH_ONLY;
    uint32_t revision = 0;
    std::string reason;
};

class ConfigValidator {
public:
    bool validate(const ConfigData& config, std::string& error) const;

    // Describe a candidate without mutating the persisted/runtime snapshot.
    // The readiness decision intentionally goes through the same validator as
    // a full configuration operation, so the UI cannot advertise a strategy
    // that the backend would later reject.
    bool describeStrategy(const ConfigData& config,
                          BalanceStrategyId strategyId,
                          BalanceStrategyCapability& capability) const;
};
