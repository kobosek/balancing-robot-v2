// main/events/IMU_OrientationData.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Make sure ORIENTATION_DATA_READY is defined

class IMU_OrientationData : public BaseEvent {
public:
    const float pitch_rad;
    const float pitch_rate_radps;

    IMU_OrientationData(float p_rad, float pr_rps) :
        BaseEvent(EventType::IMU_ORIENTATION_DATA), // Use the specific type
        pitch_rad(p_rad),
        pitch_rate_radps(pr_rps)
    {}
};