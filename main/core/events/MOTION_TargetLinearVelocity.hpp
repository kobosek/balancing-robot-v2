#pragma once

#include "BaseEvent.hpp"
#include <cmath>
#include <cstdint>

// Validated longitudinal command.  It deliberately has a different event
// type from MOTION_TargetMovement so m/s can never be mistaken for degrees.
class MOTION_TargetLinearVelocity : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTION_TargetLinearVelocity)

    const float targetVelocityMps;
    // Compatibility spellings keep the unit explicit for callers outside the
    // current application while the canonical field remains camelCase.
    const float targetVelocity_mps;
    const float targetLinearVelocityMps;
    const bool stop;
    const bool drive;
    const uint64_t sequence;
    const int64_t receivedTimestampUs;

    MOTION_TargetLinearVelocity(float targetVelocityMps_,
                                bool stop_,
                                uint64_t sequence_ = 0,
                                int64_t receivedTimestampUs_ = 0)
        : BaseEvent(),
          targetVelocityMps(targetVelocityMps_),
          targetVelocity_mps(targetVelocityMps_),
          targetLinearVelocityMps(targetVelocityMps_),
          stop(stop_),
          drive(!stop_ && std::isfinite(targetVelocityMps_) &&
                std::fabs(targetVelocityMps_) > 1e-5f),
          sequence(sequence_),
          receivedTimestampUs(receivedTimestampUs_ > 0 ?
                              receivedTimestampUs_ : timestamp) {}
};

