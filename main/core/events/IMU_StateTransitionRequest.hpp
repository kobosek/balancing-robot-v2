#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "IMUState.hpp"

class IMU_StateTransitionRequest : public BaseEvent {
public:
    IMU_StateTransitionRequest(IMUState requested_state) 
        : BaseEvent(EventType::IMU_STATE_TRANSITION_REQUEST), 
          requestedState(requested_state) {}
    
    IMUState requestedState;
};
