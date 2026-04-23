#pragma once

#include "BaseEvent.hpp"

class IMU_CalibrationRequestRejected : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_CalibrationRequestRejected)
    enum class Reason {
        NOT_IDLE,           // Calibration rejected because system not in IDLE state
        RECOVERY_IN_PROGRESS, // Calibration rejected due to ongoing recovery
        OTHER               // Other unspecified reasons
    };

    IMU_CalibrationRequestRejected(Reason reject_reason = Reason::OTHER, bool retry_later = false) 
        : BaseEvent(), 
          reason(reject_reason),
          retryWhenPossible(retry_later) {}
    
    Reason reason;
    bool retryWhenPossible; // If true, system will attempt calibration when state permits
};

