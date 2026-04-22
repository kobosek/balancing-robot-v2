#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class IMU_AvailabilityChanged : public BaseEvent {
public:
    explicit IMU_AvailabilityChanged(bool imuAvailable)
        : BaseEvent(EventType::IMU_AVAILABILITY_CHANGED),
          available(imuAvailable) {}

    bool available;
};
