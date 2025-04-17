#pragma once
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Include full definition for payload

class ConfigUpdatedEvent : public BaseEvent {
public:
    // Add reference to the updated config data payload
    const ConfigData& configData;

    // Constructor takes the config data
    explicit ConfigUpdatedEvent(const ConfigData& data) :
        BaseEvent(EventType::CONFIG_UPDATED),
        configData(data) {}
};