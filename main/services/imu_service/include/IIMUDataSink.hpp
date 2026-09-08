#pragma once
#include "IMUDataTypes.hpp"
class IIMUDataSink {
public:
    virtual ~IIMUDataSink() = default;
    virtual bool processSample(float ax, float ay, float az, float gx, float gy, float gz,
                               int64_t sampleTimestampUs, uint32_t generation,
                               const IMUSampleMetadata& metadata = {}) = 0;
    virtual void recordFifoLoss(uint64_t knownDiscarded, bool uncertain) = 0;
};
