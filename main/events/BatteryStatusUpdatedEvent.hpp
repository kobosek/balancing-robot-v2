#pragma once
#include "BaseEvent.hpp" // Keep relative if BaseEvent is in same dir

// Define BatteryStatus struct here or move to its own header in events/
struct BatteryStatus {
    float voltage = 0.0f;
    int percentage = 0; // 0-100%
    bool isLow = false;
};

class BatteryStatusUpdatedEvent : public BaseEvent {
public:
    const BatteryStatus status;

    BatteryStatusUpdatedEvent(const BatteryStatus& battStatus) :
        BaseEvent(EventType::BATTERY_STATUS_UPDATED), // EventTypes included via BaseEvent? Check BaseEvent.hpp
        status(battStatus) {}
};