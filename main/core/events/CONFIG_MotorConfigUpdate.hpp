#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for MotorConfig struct

class CONFIG_MotorConfigUpdate : public BaseEvent {
public:
    const MotorConfig& config;
    const bool requiresHardwareInit;  // Flag indicating if hardware re-init is needed

    CONFIG_MotorConfigUpdate(const MotorConfig& data, bool hardwareInit) :
        BaseEvent(EventType::CONFIG_MOTOR_UPDATE),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};
