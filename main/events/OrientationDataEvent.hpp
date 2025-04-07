// main/include/OrientationDataEvent.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Make sure ORIENTATION_DATA_READY is defined

class OrientationDataEvent : public BaseEvent {
public:
    const float pitch_rad;
    const float roll_rad;
    const float pitch_rate_radps;
    const float roll_rate_radps; // Add if needed

    OrientationDataEvent(float p_rad, float r_rad, float pr_rps, float rr_rps) :
        BaseEvent(EventType::ORIENTATION_DATA_READY), // Use the specific type
        pitch_rad(p_rad),
        roll_rad(r_rad),
        pitch_rate_radps(pr_rps),
        roll_rate_radps(rr_rps)
    {}
};