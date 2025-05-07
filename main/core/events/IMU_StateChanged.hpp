#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "IMUState.hpp" // For IMUState enum


// Event for notifying that IMU state has changed
class IMU_StateChanged : public BaseEvent {
public:
    IMU_StateChanged(IMUState oldState, IMUState newState)
        : BaseEvent(EventType::IMU_STATE_CHANGED), 
          oldState(oldState), 
          newState(newState) {}

    const IMUState oldState;
    const IMUState newState;
};
