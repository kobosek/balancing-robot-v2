#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for WiFiConfig struct

class CONFIG_WiFiConfigUpdate : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONFIG_WiFiConfigUpdate)
    const WiFiConfig& config;

    CONFIG_WiFiConfigUpdate(const WiFiConfig& data) :
        BaseEvent(),
        config(data) {}
};

