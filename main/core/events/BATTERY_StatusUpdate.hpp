#pragma once
#include "BaseEvent.hpp" // Keep relative if BaseEvent is in same dir

// Define BatteryStatus struct here or move to its own header in events/
struct BatteryStatus {
    float voltage = 0.0f;
    float adcPinVoltage = 0.0f;
    int percentage = 0; // 0-100%
    bool isLow = false;
    bool isCritical = false;
    bool adcCalibrated = false;
};

class BATTERY_StatusUpdate : public BaseEvent {
public:
    const BatteryStatus status;

    BATTERY_StatusUpdate(const BatteryStatus& battStatus) :
        BaseEvent(EventType::BATTERY_STATUS_UPDATE), // EventTypes included via BaseEvent
        status(battStatus) {}
};
