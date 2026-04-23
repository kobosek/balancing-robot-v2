#pragma once

#include "BaseEvent.hpp"

class IMU_GyroOffsetsUpdated : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_GyroOffsetsUpdated)
        IMU_GyroOffsetsUpdated(float x_offset_dps, float y_offset_dps, float z_offset_dps)
            : BaseEvent(),
              x_dps(x_offset_dps),
              y_dps(y_offset_dps),
              z_dps(z_offset_dps)
        {}
    
        const float x_dps;
        const float y_dps;
        const float z_dps;
};

