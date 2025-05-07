#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for EncoderConfig struct

class CONFIG_EncoderConfigUpdate : public BaseEvent {
public:
    const EncoderConfig& config;
    const bool requiresHardwareInit;

    CONFIG_EncoderConfigUpdate(const EncoderConfig& data, bool hardwareInit) :
        BaseEvent(EventType::CONFIG_ENCODER_UPDATE),
        config(data),
        requiresHardwareInit(hardwareInit) {}
};
