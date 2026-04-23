#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for PIDConfig struct

class CONFIG_PidConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_PidConfigUpdate)
    const std::string pidName;  // "angle", "speed_left", "speed_right", "yaw_rate"
    const PIDConfig config;

    CONFIG_PidConfigUpdate(const std::string& name, const PIDConfig& data) :
        BaseEvent(),
        pidName(name),
        config(data) {}
};

