#pragma once

#include "BaseEvent.hpp"

enum class ControlRunMode {
    DISABLED,
    BALANCING,
    PID_TUNING,
    GUIDED_CALIBRATION
};

class CONTROL_RunModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONTROL_RunModeChanged)

    const ControlRunMode mode;
    const int telemetryStateCode;
    const bool telemetryEnabled;

    CONTROL_RunModeChanged(ControlRunMode mode_, int telemetryStateCode_, bool telemetryEnabled_) :
        BaseEvent(),
        mode(mode_),
        telemetryStateCode(telemetryStateCode_),
        telemetryEnabled(telemetryEnabled_) {}
};
