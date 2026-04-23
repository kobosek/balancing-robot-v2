#pragma once

#include "BaseEvent.hpp"

class IMU_AttachRequested : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_AttachRequested)
    IMU_AttachRequested() : BaseEvent() {}
};

