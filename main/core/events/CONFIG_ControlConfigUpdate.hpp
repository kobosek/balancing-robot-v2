#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for ControlConfig struct

class CONFIG_ControlConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_ControlConfigUpdate)
    const ControlConfig& config;

    CONFIG_ControlConfigUpdate(const ControlConfig& data) :
        BaseEvent(),
        config(data) {}
};

