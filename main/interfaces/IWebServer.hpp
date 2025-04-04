#pragma once
#include "interfaces/IConfigObserver.hpp"

// Structure to hold a single snapshot of telemetry data
struct TelemetryDataPoint {
    float pitch;
    float desiredSpeed;
    float currentSpeedLeft;
    float currentSpeedRight;
    float rmseLeft;
    float rmseRight;
    // Optional: uint64_t timestamp; // Could add later if needed
};

class IWebServer : public IConfigObserver {
    public:
        // Method to add a new telemetry data point to the buffer
        virtual void addTelemetryData(const TelemetryDataPoint& data) = 0;
        virtual ~IWebServer() = default;
};
