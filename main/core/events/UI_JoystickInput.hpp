// main/events/UI_JoystickInput.hpp
#pragma once
#include "BaseEvent.hpp"
#include <cstdint>

// Event published when raw joystick input is received from any source (WS, BT etc)
// Coordinates are typically normalized from -1.0 to +1.0
class UI_JoystickInput : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_JoystickInput)
    const float x; // Left/Right axis (-1.0 to +1.0)
    const float y; // Forward/Backward axis (-1.0 to +1.0)
    // The web client echoes the command session advertised by /api/state.
    // NestedPid keeps its legacy input path; longitudinal commands require
    // this token to match the current arm.
    const uint64_t sessionId;
    // Monotonic sequence assigned by the producer (the web UI).  The
    // CommandProcessor checks it before refreshing command freshness so a
    // reordered packet cannot revive an older longitudinal request.
    const uint64_t sourceSequence;

    UI_JoystickInput(float joystick_x, float joystick_y,
                     uint64_t sessionId_ = 0,
                     uint64_t sourceSequence_ = 0) :
        BaseEvent(),
        x(joystick_x),
        y(joystick_y),
        sessionId(sessionId_),
        sourceSequence(sourceSequence_)
        {}
};

