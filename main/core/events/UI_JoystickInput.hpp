// main/events/UI_JoystickInput.hpp
#pragma once
#include "BaseEvent.hpp"

// Event published when raw joystick input is received from any source (WS, BT etc)
// Coordinates are typically normalized from -1.0 to +1.0
class UI_JoystickInput : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_JoystickInput)
    const float x; // Left/Right axis (-1.0 to +1.0)
    const float y; // Forward/Backward axis (-1.0 to +1.0)

    UI_JoystickInput(float joystick_x, float joystick_y) :
        BaseEvent(),
        x(joystick_x),
        y(joystick_y)
        {}
};

