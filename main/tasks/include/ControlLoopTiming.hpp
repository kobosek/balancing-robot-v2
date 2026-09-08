#pragma once
#include "freertos/FreeRTOS.h"
#include <cstdint>

// The wake anchor follows actual execution, never a backlog of missed periods.
// Kept separate from task execution so delayed-wakeup behavior can be tested.
class ControlLoopTiming {
public:
    ControlLoopTiming(TickType_t tick, int64_t timestampUs)
        : m_wakeTick(tick), m_previousStepUs(timestampUs) {}
    TickType_t wakeTick() const { return m_wakeTick; }
    float beginStep(TickType_t tick, int64_t timestampUs) {
        const auto elapsedUs = timestampUs - m_previousStepUs;
        m_wakeTick = tick;
        m_previousStepUs = timestampUs;
        return static_cast<float>(elapsedUs) / 1000000.0f;
    }
private:
    TickType_t m_wakeTick;
    int64_t m_previousStepUs;
};
