#pragma once
#include <atomic>
#include <cstdint>
namespace sensor_fake {
extern std::atomic<bool> absent, noSamples, overflow;
extern std::atomic<int> failReadRegister, readFailures, failWriteRegister, writeFailures;
extern std::atomic<int> fifoFailureConsume, fixedCount, fifoFillByte;
extern std::atomic<int64_t> clockUs;
extern std::atomic<unsigned> irqSetupDelayMs, fifoSaturateAxes;
extern std::atomic<int> fifoPose, calibrationGyroZ, calibrationAccelX;
extern std::atomic<bool> calibrationAlternating;
extern std::atomic<unsigned> fifoReads, countReads, resets, opens, closes, wrongOwner;
void reset();
}
