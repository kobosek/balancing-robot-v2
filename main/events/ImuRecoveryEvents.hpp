#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_err.h"

class AttemptImuRecoveryCommand : public BaseEvent {
public:
    AttemptImuRecoveryCommand() : BaseEvent(EventType::ATTEMPT_IMU_RECOVERY_COMMAND) {}
    // No payload needed for this command
};

class ImuRecoverySucceededEvent : public BaseEvent {
public:
    ImuRecoverySucceededEvent() : BaseEvent(EventType::IMU_RECOVERY_SUCCEEDED) {}
    // No payload needed for this event
};

class ImuRecoveryFailedEvent : public BaseEvent {
public:
    /**
     * @brief Construct a new Imu Recovery Failed Event object
     * 
     * @param code The error code indicating the reason for failure.
     */
    explicit ImuRecoveryFailedEvent(esp_err_t code)
        : BaseEvent(EventType::IMU_RECOVERY_FAILED), errorCode(code) {}

    /// The error code indicating why recovery failed.
    const esp_err_t errorCode;
};
