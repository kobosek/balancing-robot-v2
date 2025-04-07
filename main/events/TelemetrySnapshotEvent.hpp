#pragma once
#include "BaseEvent.hpp"
#include "include/WebServer.hpp" // For TelemetryDataPoint definition? Better to define it separately.

 // Define TelemetryDataPoint structure separately if not already done
struct TelemetryDataPoint {
    float pitch = 0.0f;
    float desiredSpeed = 0.0f; // Or target speed?
    float currentSpeedLeft = 0.0f;
    float currentSpeedRight = 0.0f;
    float effortLeft = 0.0f;
    float effortRight = 0.0f;
    float batteryVoltage = 0.0f;
     // Add other relevant data (roll, rates, PID terms?)
};


class TelemetrySnapshotEvent : public BaseEvent {
public:
    const TelemetryDataPoint snapshot;

    TelemetrySnapshotEvent(const TelemetryDataPoint& data) :
        BaseEvent(EventType::PID_DIAGNOSTICS_UPDATED), // Or a dedicated Telemetry type
        snapshot(data)
    {}
};