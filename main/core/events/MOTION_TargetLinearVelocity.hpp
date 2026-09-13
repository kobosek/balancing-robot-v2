#pragma once

#include "BaseEvent.hpp"
#include <cstdint>

// Validated longitudinal command.  It deliberately has a different event
// type from MOTION_TargetMovement so m/s can never be mistaken for degrees.
class MOTION_TargetLinearVelocity : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTION_TargetLinearVelocity)

    const float targetVelocityMps;
    const bool stop;
    const uint64_t sequence;
    const int64_t receivedTimestampUs;
    // The StateManager arm that authorized this command. A sequence number
    // alone cannot protect a new control session from a delayed callback.
    const uint64_t armId;

    MOTION_TargetLinearVelocity(float targetVelocityMps_,
                                bool stop_,
                                uint64_t sequence_ = 0,
                                int64_t receivedTimestampUs_ = 0,
                                uint64_t armId_ = 0)
        : BaseEvent(),
          targetVelocityMps(targetVelocityMps_),
          stop(stop_),
          sequence(sequence_),
          receivedTimestampUs(receivedTimestampUs_ > 0 ?
                              receivedTimestampUs_ : timestamp),
          armId(armId_) {}
};
