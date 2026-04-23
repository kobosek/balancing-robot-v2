#pragma once

#include "BaseEvent.hpp"
#include <string>

class EventHandler {
public:
    virtual ~EventHandler() = default;

    virtual void handleEvent(const BaseEvent& event) = 0;

    virtual std::string getHandlerName() const = 0;
};
