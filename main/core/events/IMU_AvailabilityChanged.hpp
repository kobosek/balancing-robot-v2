#pragma once
#include "BaseEvent.hpp"
#include <cstdint>
class IMU_AvailabilityChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_AvailabilityChanged)
    const bool available;
    const uint32_t generation;
    const uint64_t revision;
    IMU_AvailabilityChanged(bool ready, uint32_t stream = 0, uint64_t change = 0)
        : available(ready), generation(stream), revision(change) {}
};
