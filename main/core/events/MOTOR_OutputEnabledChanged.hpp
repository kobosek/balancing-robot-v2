#pragma once

#include "BaseEvent.hpp"

class MOTOR_OutputEnabledChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTOR_OutputEnabledChanged)

    const bool enabled;

    explicit MOTOR_OutputEnabledChanged(bool enabled_) :
        BaseEvent(),
        enabled(enabled_) {}
};
