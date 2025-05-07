#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class MOTION_UsingFallbackValues : public BaseEvent {
public:
    MOTION_UsingFallbackValues(bool using_fallback = true) 
        : BaseEvent(EventType::MOTION_USING_FALLBACK_VALUES), 
          usingFallbackValues(using_fallback) {}
    
    bool usingFallbackValues;
};
