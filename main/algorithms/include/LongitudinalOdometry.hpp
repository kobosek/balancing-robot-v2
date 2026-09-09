#pragma once

#include "config/EncoderConfig.hpp"
#include <cstdint>

struct EncoderFrame;

enum class LongitudinalOdometryUpdateStatus : uint8_t {
    UNINITIALIZED = 0,
    ACCEPTED,
    DUPLICATE,
    OUT_OF_ORDER,
    INVALID_FRAME
};

struct LongitudinalOdometryConfig {
    double leftWheelRadiusM = 0.0325;
    double rightWheelRadiusM = 0.0325;
    // Effective count scale used by EncoderService; no implicit quadrature
    // multiplier is applied here.
    double metersPerCountLeft = 0.0;
    double metersPerCountRight = 0.0;
    // Current encoder wiring reports a forward motion as +left/-right.
    int8_t leftForwardSign = 1;
    int8_t rightForwardSign = -1;
    int64_t maxSampleAgeUs = 20000;
    int64_t maxWheelTimestampSkewUs = 2000;
};

LongitudinalOdometryConfig longitudinalOdometryConfigFromEncoder(
    const EncoderConfig& encoderConfig,
    int64_t maxSampleAgeUs = 20000,
    int64_t maxWheelTimestampSkewUs = 2000);

struct LongitudinalOdometryResult {
    LongitudinalOdometryUpdateStatus status = LongitudinalOdometryUpdateStatus::UNINITIALIZED;
    bool frameAccepted = false;
    bool sampleTimingValid = false;
    bool leftFeedbackValid = false;
    bool rightFeedbackValid = false;
    bool velocityValid = false;
    bool continuityValid = false;
    bool positionValid = false;
    bool odometryValid = false;

    uint64_t sequence = 0;
    uint64_t odometrySequence = 0;
    uint32_t generation = 0;
    uint32_t leftContinuityEpoch = 0;
    uint32_t rightContinuityEpoch = 0;
    int64_t sampleTimestampUs = 0;

    double leftPositionM = 0.0;
    double rightPositionM = 0.0;
    double positionM = 0.0;
    double distanceDifferenceM = 0.0;
    float leftVelocityMps = 0.0f;
    float rightVelocityMps = 0.0f;
    float velocityMps = 0.0f;
    float velocityDifferenceMps = 0.0f;
};

class LongitudinalOdometry {
public:
    explicit LongitudinalOdometry(const LongitudinalOdometryConfig& config = {});
    explicit LongitudinalOdometry(const EncoderConfig& encoderConfig,
                                  int64_t maxSampleAgeUs = 20000,
                                  int64_t maxWheelTimestampSkewUs = 2000);

    void configure(const LongitudinalOdometryConfig& config);
    void setMaxSampleAgeUs(int64_t maxSampleAgeUs);
    void reset();

    const LongitudinalOdometryConfig& config() const { return m_config; }
    LongitudinalOdometryResult update(const EncoderFrame& frame, int64_t nowUs);
    const LongitudinalOdometryResult& latest() const { return m_latest; }

private:
    LongitudinalOdometryConfig m_config;
    bool m_scalingValid = false;
    bool m_hasSequence = false;
    bool m_hasEpoch = false;
    bool m_hasBase = false;
    bool m_continuityBroken = false;
    uint64_t m_lastSequence = 0;
    int64_t m_lastSampleTimestampUs = 0;
    uint32_t m_leftEpoch = 0;
    uint32_t m_rightEpoch = 0;
    int64_t m_baseLeftCount = 0;
    int64_t m_baseRightCount = 0;
    uint32_t m_generation = 0;
    LongitudinalOdometryResult m_latest;

    void refreshScalingValidity();
    void markContinuityBroken(uint32_t leftEpoch, uint32_t rightEpoch,
                              bool epochChanged);
    LongitudinalOdometryResult statusResult(const EncoderFrame& frame,
                                            LongitudinalOdometryUpdateStatus status,
                                            int64_t sampleTimestampUs) const;
};
