#pragma once
#include "BaseEvent.hpp"
#include <cstdint>
enum class ControlRunMode { DISABLED, BALANCING, PID_TUNING, GUIDED_CALIBRATION };
class CONTROL_RunModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONTROL_RunModeChanged)
    const ControlRunMode mode;
    const int telemetryStateCode;
    const bool telemetryEnabled;
    const uint64_t armId;
    const uint32_t generation;
    CONTROL_RunModeChanged(ControlRunMode mode_, int state, bool telemetry, uint64_t arm, uint32_t stream)
        : mode(mode_), telemetryStateCode(state), telemetryEnabled(telemetry), armId(arm), generation(stream) {}
};
