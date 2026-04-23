#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for MotorConfig struct

class CONFIG_MotorConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_MotorConfigUpdate)
    const MotorConfig& config;
    const bool requiresHardwareInit;  // Flag indicating if hardware re-init is needed

    CONFIG_MotorConfigUpdate(const MotorConfig& data, bool hardwareInit) :
        BaseEvent(),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};

