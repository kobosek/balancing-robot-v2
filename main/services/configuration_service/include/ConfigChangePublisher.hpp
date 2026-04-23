#pragma once

#include "ConfigData.hpp"

class EventBus;

class ConfigChangePublisher {
public:
    explicit ConfigChangePublisher(EventBus& eventBus);

    void publishFullConfig(const ConfigData& config) const;
    void publishImuConfig(const MPU6050Config& config, bool requiresHardwareInit) const;
    void publishChanges(const ConfigData& oldConfig, const ConfigData& newConfig) const;

private:
    EventBus& m_eventBus;
};
