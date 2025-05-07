#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for BatteryConfig struct

class CONFIG_BatteryConfigUpdate : public BaseEvent {
public:
    const BatteryConfig& config;
    const bool requiresHardwareInit;  // Flag indicating if hardware re-init is needed

    CONFIG_BatteryConfigUpdate(const BatteryConfig& data, bool hardwareInit) :
        BaseEvent(EventType::CONFIG_BATTERY_UPDATE),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};
