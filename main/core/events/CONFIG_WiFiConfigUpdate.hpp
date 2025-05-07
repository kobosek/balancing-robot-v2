#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include for WiFiConfig struct

class CONFIG_WiFiConfigUpdate : public BaseEvent {
public:
    const WiFiConfig& config;

    CONFIG_WiFiConfigUpdate(const WiFiConfig& data) :
        BaseEvent(EventType::CONFIG_WIFI_UPDATE),
        config(data) {}
};
