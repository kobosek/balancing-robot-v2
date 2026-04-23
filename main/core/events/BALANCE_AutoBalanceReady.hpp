// main/core/events/BALANCE_AutoBalanceReady.hpp
#pragma once
#include "BaseEvent.hpp"

class BALANCE_AutoBalanceReady : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(BALANCE_AutoBalanceReady)
    BALANCE_AutoBalanceReady() :
        BaseEvent() {}
};

