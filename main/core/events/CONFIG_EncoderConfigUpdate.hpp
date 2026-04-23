#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for EncoderConfig struct

class CONFIG_EncoderConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_EncoderConfigUpdate)
    const EncoderConfig& config;
    const bool requiresHardwareInit;

    CONFIG_EncoderConfigUpdate(const EncoderConfig& data, bool hardwareInit) :
        BaseEvent(),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};

