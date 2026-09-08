#pragma once
#include "BaseEvent.hpp"
#include <cstdint>
class MOTOR_OutputEnabledChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTOR_OutputEnabledChanged)
    const bool enabled;
    const uint64_t armId;
    const uint32_t generation;
    MOTOR_OutputEnabledChanged(bool enable, uint64_t arm, uint32_t stream)
        : enabled(enable), armId(arm), generation(stream) {}
};
