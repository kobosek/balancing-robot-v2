#pragma once
#include "BaseEvent.hpp"
#include "TelemetryDataPoint.hpp"

class TELEMETRY_Snapshot : public BaseEvent {
public:
    const TelemetryDataPoint snapshot;

    TELEMETRY_Snapshot(const TelemetryDataPoint& data) :
        BaseEvent(EventType::TELEMETRY_SNAPSHOT),
        snapshot(data)
    {}
};