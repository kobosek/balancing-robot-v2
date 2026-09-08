#pragma once
#include "BaseEvent.hpp"
class BALANCE_AutoBalanceReady : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(BALANCE_AutoBalanceReady)
    const uint32_t generation;
    const int64_t sampleTimestampUs;
    BALANCE_AutoBalanceReady(uint32_t stream, int64_t sampleTime)
        : generation(stream), sampleTimestampUs(sampleTime) {}
};
