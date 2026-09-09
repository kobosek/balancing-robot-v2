#pragma once
#include "esp_err.h"
#include <cstdint>
#include <cmath>
enum class SensorFIFOState : uint8_t { STOPPED, STREAMING, BACKLOG, DISCONTINUITY };
struct IMUSampleMetadata {
    uint16_t fifoRemainingPackets = 0; // Count observation, excludes packets arriving during I2C.
    uint8_t saturationMask = 0; // Bits 0..5: ax, ay, az, gx, gy, gz at ADC rails.
};
struct SensorFrame {
    float pitch_deg = 0, pitch_rate_dps = 0, yaw_deg = 0, yaw_rate_dps = 0;
    int64_t sample_timestamp_us = 0;
    uint64_t sample_sequence = 0;
    uint32_t generation = 0;
    bool valid = false; // IMU validity; all fields are copied under one cross-core lock.
    float ax_g = 0, ay_g = 0, az_g = 0;
    uint8_t saturationMask = 0;
    SensorFIFOState fifoState = SensorFIFOState::STOPPED;
    uint16_t fifoRemainingPackets = 0;
    uint64_t lostSamples = 0; // Cumulative known discarded complete packets.
    bool lossCountUncertain = false; // Sticky: actual loss may exceed this lower bound.
    float tilt_deg = 180.0f; // Total inclination from +Z, not the pitch Euler angle.
    bool gravityReferenceValid = false; // A usable accelerometer observation for a new arm/hold.
    bool gyroContinuityLost = false; // Sticky until owner starts a new estimator generation.
    bool withinTilt(float limitDeg) const {
        return valid && std::isfinite(tilt_deg) && tilt_deg >= 0.0f && tilt_deg < limitDeg;
    }
    bool fresh(int64_t now, int64_t maxAge) const {
        return valid && !gyroContinuityLost && !(saturationMask & 0x38) &&
            sample_timestamp_us > 0 && now >= sample_timestamp_us &&
            now - sample_timestamp_us <= maxAge && std::isfinite(pitch_deg) &&
            std::isfinite(pitch_rate_dps) && std::isfinite(yaw_deg) && std::isfinite(yaw_rate_dps);
    }
};
using OrientationEstimate = SensorFrame; // Compatibility for existing event/control consumers.

enum class IMUFaultReason : uint8_t { NONE, STALE, TRANSPORT, FIFO_ALIGNMENT, OVERFLOW, VALIDATION, CONFIGURATION, CALIBRATION, STOPPED };
enum class FIFOOutcome : uint8_t { NO_DATA, ACCEPTED, READ_FAILURE, LOST_ALIGNMENT, HARDWARE_FAILURE };
struct FIFOResult {
    FIFOOutcome outcome = FIFOOutcome::NO_DATA;
    esp_err_t error = ESP_OK;
    IMUFaultReason reason = IMUFaultReason::NONE;
    unsigned accepted = 0;
    bool retried = false;
    bool moreData = false;
};
struct IMUStatusSnapshot {
    const char* state = "INITIALIZED";
    bool ready = false, busy = false, configurationPending = false;
    uint32_t generation = 0;
    uint64_t revision = 0;
    int64_t sampleTimestampUs = 0, stateChangedUs = 0;
    esp_err_t lastError = ESP_OK;
    IMUFaultReason lastReason = IMUFaultReason::NONE;
    uint32_t transportErrors = 0, fifoResyncs = 0, reconnectAttempts = 0, reconnectSuccesses = 0, irqFallbacks = 0;
    uint32_t busFrequencyHz = 0, samplePeriodUs = 0;
    uint64_t sampleSequence = 0;
    uint16_t fifoRemainingPackets = 0;
    uint32_t gyroClippingResets = 0;
};
