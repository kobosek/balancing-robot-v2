#pragma once

#include "BaseEvent.hpp"

class IMU_SystemPolicyChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_SystemPolicyChanged)

    const bool calibrationAllowed;
    const bool autoAttachAllowed;
    const bool hardwareConfigApplyAllowed;

    IMU_SystemPolicyChanged(bool calibrationAllowed_,
                            bool autoAttachAllowed_,
                            bool hardwareConfigApplyAllowed_) :
        BaseEvent(),
        calibrationAllowed(calibrationAllowed_),
        autoAttachAllowed(autoAttachAllowed_),
        hardwareConfigApplyAllowed(hardwareConfigApplyAllowed_) {}
};
