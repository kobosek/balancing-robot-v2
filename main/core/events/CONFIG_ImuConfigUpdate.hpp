#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for MPU6050Config struct

class CONFIG_ImuConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_ImuConfigUpdate)
    const MPU6050Config& config;
    const bool requiresHardwareInit;

    CONFIG_ImuConfigUpdate(const MPU6050Config& data, bool hardwareInit) :
        BaseEvent(),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};

