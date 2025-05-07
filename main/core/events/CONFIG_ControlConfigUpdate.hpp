#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for ControlConfig struct

class CONFIG_ControlConfigUpdate : public BaseEvent {
public:
    const ControlConfig& config;

    CONFIG_ControlConfigUpdate(const ControlConfig& data) :
        BaseEvent(EventType::CONFIG_CONTROL_UPDATE),
        config(data) {}
};
