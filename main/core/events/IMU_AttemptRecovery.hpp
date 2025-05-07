#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "ConfigData.hpp"

class IMU_AttemptRecovery : public BaseEvent {
public:
    IMU_AttemptRecovery(bool minimal_interruption = false, 
                        int max_attempts = 3, 
                        int retry_delay_ms = 500) 
        : BaseEvent(EventType::IMU_ATTEMPT_RECOVERY),
          config{minimal_interruption, max_attempts, retry_delay_ms} {}

    IMURecoveryConfig config;
};
