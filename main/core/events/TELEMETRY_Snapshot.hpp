#pragma once
#include "BaseEvent.hpp"
#include "TelemetryDataPoint.hpp"

class TELEMETRY_Snapshot : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(TELEMETRY_Snapshot)
    const TelemetryDataPoint snapshot;

    TELEMETRY_Snapshot(const TelemetryDataPoint& data) :
        BaseEvent(),
        snapshot(data)
    {}
};

