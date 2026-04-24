#pragma once

#include "BalanceControlTypes.hpp"
#include "ConfigData.hpp"
#include "config/PIDConfig.hpp"
#include <string>

class IBalanceControlStrategy {
public:
    virtual ~IBalanceControlStrategy() = default;

    virtual const char* name() const = 0;
    virtual MotorEffort update(const BalanceControlInput& input) = 0;
    virtual void reset() = 0;
    virtual void applyConfig(const ConfigData& config) = 0;
    virtual void updatePidConfig(const std::string& pidName, const PIDConfig& config) = 0;
    virtual float getLastSpeedSetpointLeftDPS() const = 0;
    virtual float getLastSpeedSetpointRightDPS() const = 0;
    virtual bool isYawControlEnabled() const = 0;
};

