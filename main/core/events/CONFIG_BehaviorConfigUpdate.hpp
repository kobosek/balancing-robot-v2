#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for SystemBehaviorConfig struct

class CONFIG_BehaviorConfigUpdate : public BaseEvent {
public:
    const SystemBehaviorConfig& config;

    CONFIG_BehaviorConfigUpdate(const SystemBehaviorConfig& data) :
        BaseEvent(EventType::CONFIG_BEHAVIOR_UPDATE),
        config(data) {}
};
