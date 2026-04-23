#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for BatteryConfig struct

class CONFIG_BatteryConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_BatteryConfigUpdate)
    const BatteryConfig& config;
    const bool requiresHardwareInit;  // Flag indicating if hardware re-init is needed

    CONFIG_BatteryConfigUpdate(const BatteryConfig& data, bool hardwareInit) :
        BaseEvent(),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};

