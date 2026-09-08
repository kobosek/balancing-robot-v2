#pragma once
#include "IMUDataTypes.hpp"
#include "IIMUDataSink.hpp"
#include "MPU6050Profile.hpp"
class MPU6050Driver;
// Called only by IMUTask. No recovery callbacks or interrupt ownership.
class FIFOProcessor {
public:
    FIFOProcessor(MPU6050Driver& driver, IIMUDataSink& sink) : m_driver(driver), m_sink(sink) {}
    void configure(const MPU6050Profile& profile, int threshold);
    FIFOResult processFIFO(uint32_t generation, int64_t retryDeadlineUs);
    esp_err_t resync();
    static int64_t sampleTimestamp(int64_t start, unsigned packets, unsigned index, int64_t period) {
        // N complete packets span at most N periods before the count-read start.
        // The caller adds a period when the count contains a partial packet.
        return start - (packets - index + 0LL) * period;
    }
private:
    MPU6050Driver& m_driver;
    IIMUDataSink& m_sink;
    MPU6050Profile m_profile;
    unsigned m_threshold = 1, m_chunk = 1;
};
