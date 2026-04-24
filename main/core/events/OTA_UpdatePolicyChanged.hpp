#pragma once

#include "BaseEvent.hpp"

class OTA_UpdatePolicyChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(OTA_UpdatePolicyChanged)

    const bool updateAllowed;

    explicit OTA_UpdatePolicyChanged(bool updateAllowed_) :
        BaseEvent(),
        updateAllowed(updateAllowed_) {}
};
