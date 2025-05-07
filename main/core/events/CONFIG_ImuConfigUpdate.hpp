#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for MPU6050Config struct

class CONFIG_ImuConfigUpdate : public BaseEvent {
public:
    const MPU6050Config& config;
    const bool requiresHardwareInit;

    CONFIG_ImuConfigUpdate(const MPU6050Config& data, bool hardwareInit) :
        BaseEvent(EventType::CONFIG_IMU_UPDATE),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};
