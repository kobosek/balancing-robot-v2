#pragma once

#include "BaseEvent.hpp"

class BALANCE_MonitorModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(BALANCE_MonitorModeChanged)

    const bool fallDetectionActive;
    const bool autoBalancingActive;

    BALANCE_MonitorModeChanged(bool fallDetectionActive_, bool autoBalancingActive_) :
        BaseEvent(),
        fallDetectionActive(fallDetectionActive_),
        autoBalancingActive(autoBalancingActive_) {}
};
