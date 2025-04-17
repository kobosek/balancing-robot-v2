#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class GyroOffsetsUpdatedEvent : public BaseEvent {
    public:
        GyroOffsetsUpdatedEvent(float x_offset_dps, float y_offset_dps, float z_offset_dps)
            : BaseEvent(EventType::GYRO_OFFSETS_UPDATED),
              x_dps(x_offset_dps),
              y_dps(y_offset_dps),
              z_dps(z_offset_dps)
        {}
    
        const float x_dps;
        const float y_dps;
        const float z_dps;
};