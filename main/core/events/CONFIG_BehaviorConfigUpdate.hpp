#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for SystemBehaviorConfig struct

class CONFIG_BehaviorConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_BehaviorConfigUpdate)
    const SystemBehaviorConfig& config;

    CONFIG_BehaviorConfigUpdate(const SystemBehaviorConfig& data) :
        BaseEvent(),
        config(data) {}
};

