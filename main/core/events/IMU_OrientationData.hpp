#pragma once
#include "BaseEvent.hpp"
#include "IMUDataTypes.hpp"
class IMU_OrientationData : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_OrientationData)
    const OrientationEstimate estimate;
    const float pitch_rad, pitch_rate_radps;
    explicit IMU_OrientationData(const OrientationEstimate& value)
        : estimate(value), pitch_rad(value.pitch_deg * 0.01745329252f),
          pitch_rate_radps(value.pitch_rate_dps * 0.01745329252f) {}
};
