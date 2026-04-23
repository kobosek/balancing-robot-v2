#pragma once

#include "BaseEvent.hpp"

class IMU_AvailabilityChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_AvailabilityChanged)
    explicit IMU_AvailabilityChanged(bool imuAvailable)
        : BaseEvent(),
          available(imuAvailable) {}

    bool available;
};

