#pragma once
#include "BaseEvent.hpp"
// #include "ConfigData.hpp" // Use forward declaration if only needed as reference/pointer
// Forward declare if payload reference/pointer is added later
struct ConfigData; // Defined in config/include

class ConfigUpdatedEvent : public BaseEvent {
public:
    // Optional: Add reference to the updated config data if needed by subscribers
    // const ConfigData& updatedConfig;

    ConfigUpdatedEvent(/* const ConfigData& config */) :
        BaseEvent(EventType::CONFIG_UPDATED)
        /* , updatedConfig(config) */ {}
};