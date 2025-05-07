#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include full definition for payload

class CONFIG_FullConfigUpdate : public BaseEvent {
public:
    // Add reference to the updated config data payload
    const ConfigData& configData;

    // Constructor takes the config data
    explicit CONFIG_FullConfigUpdate(const ConfigData& data) :
        BaseEvent(EventType::CONFIG_FULL_UPDATE),
        configData(data) {}
};