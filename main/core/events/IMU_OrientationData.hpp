// main/events/IMU_OrientationData.hpp
#pragma once
#include "BaseEvent.hpp"

class IMU_OrientationData : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_OrientationData)
    const float pitch_rad;
    const float pitch_rate_radps;

    IMU_OrientationData(float p_rad, float pr_rps) :
        BaseEvent(), // Use the specific type
        pitch_rad(p_rad),
        pitch_rate_radps(pr_rps)
    {}
};

