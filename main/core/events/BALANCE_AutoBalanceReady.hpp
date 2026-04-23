// main/core/events/BALANCE_AutoBalanceReady.hpp
#pragma once
#include "BaseEvent.hpp"

class BALANCE_AutoBalanceReady : public BaseEvent {
public:
    BALANCE_AutoBalanceReady() :
        BaseEvent(EventType::BALANCE_AUTO_BALANCE_READY) {}
};
