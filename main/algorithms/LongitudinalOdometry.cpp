#include "LongitudinalOdometry.hpp"

#include "EncoderService.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
}

LongitudinalOdometryConfig longitudinalOdometryConfigFromEncoder(
    const EncoderConfig& encoderConfig,
    int64_t maxSampleAgeUs,
    int64_t maxWheelTimestampSkewUs)
{
    LongitudinalOdometryConfig config;
    config.leftWheelRadiusM = static_cast<double>(encoderConfig.wheel_diameter_mm) / 2000.0;
    config.rightWheelRadiusM = config.leftWheelRadiusM;
    if (encoderConfig.pulses_per_revolution_motor > 0.0f &&
        encoderConfig.gear_ratio > 0.0f) {
        // EncoderService uses the configured pulses value as the effective
        // decoded count and does not add an automatic x4 multiplier.
        const double countsPerWheelRevolution =
            static_cast<double>(encoderConfig.pulses_per_revolution_motor) *
            static_cast<double>(encoderConfig.gear_ratio);
        const double circumference = 2.0 * PI * config.leftWheelRadiusM;
        config.metersPerCountLeft = circumference / countsPerWheelRevolution;
        config.metersPerCountRight = config.metersPerCountLeft;
    }
    config.maxSampleAgeUs = std::max<int64_t>(0, maxSampleAgeUs);
    config.maxWheelTimestampSkewUs = std::max<int64_t>(0, maxWheelTimestampSkewUs);
    return config;
}

LongitudinalOdometry::LongitudinalOdometry(const LongitudinalOdometryConfig& config)
{
    configure(config);
}

LongitudinalOdometry::LongitudinalOdometry(const EncoderConfig& encoderConfig,
                                           int64_t maxSampleAgeUs,
                                           int64_t maxWheelTimestampSkewUs)
{
    configure(longitudinalOdometryConfigFromEncoder(
        encoderConfig, maxSampleAgeUs, maxWheelTimestampSkewUs));
}

void LongitudinalOdometry::configure(const LongitudinalOdometryConfig& config)
{
    m_config = config;
    m_config.leftForwardSign = m_config.leftForwardSign < 0 ? -1 : 1;
    m_config.rightForwardSign = m_config.rightForwardSign < 0 ? -1 : 1;
    m_config.maxSampleAgeUs = std::max<int64_t>(0, m_config.maxSampleAgeUs);
    m_config.maxWheelTimestampSkewUs = std::max<int64_t>(0, m_config.maxWheelTimestampSkewUs);
    refreshScalingValidity();
    m_generation = 0;
    reset();
}

void LongitudinalOdometry::refreshScalingValidity()
{
    m_scalingValid = std::isfinite(m_config.leftWheelRadiusM) &&
        std::isfinite(m_config.rightWheelRadiusM) &&
        std::isfinite(m_config.metersPerCountLeft) &&
        std::isfinite(m_config.metersPerCountRight) &&
        m_config.leftWheelRadiusM > 0.0 &&
        m_config.rightWheelRadiusM > 0.0 &&
        m_config.metersPerCountLeft > 0.0 &&
        m_config.metersPerCountRight > 0.0;
}

void LongitudinalOdometry::setMaxSampleAgeUs(int64_t maxSampleAgeUs)
{
    m_config.maxSampleAgeUs = std::max<int64_t>(0, maxSampleAgeUs);
}

void LongitudinalOdometry::reset()
{
    ++m_generation;
    m_hasSequence = false;
    m_hasEpoch = false;
    m_hasBase = false;
    m_continuityBroken = false;
    m_lastSequence = 0;
    m_lastSampleTimestampUs = 0;
    m_leftEpoch = 0;
    m_rightEpoch = 0;
    m_baseLeftCount = 0;
    m_baseRightCount = 0;
    m_latest = {};
    m_latest.generation = m_generation;
}

void LongitudinalOdometry::markContinuityBroken(uint32_t leftEpoch,
                                                 uint32_t rightEpoch,
                                                 bool epochChanged)
{
    if (!m_continuityBroken || epochChanged) {
        ++m_generation;
    }
    m_continuityBroken = true;
    m_hasBase = false;
    m_leftEpoch = leftEpoch;
    m_rightEpoch = rightEpoch;
    m_hasEpoch = true;
}

LongitudinalOdometryResult LongitudinalOdometry::statusResult(
    const EncoderFrame& frame,
    LongitudinalOdometryUpdateStatus status,
    int64_t sampleTimestampUs) const
{
    LongitudinalOdometryResult result = m_latest;
    result.status = status;
    result.frameAccepted = status == LongitudinalOdometryUpdateStatus::ACCEPTED ||
        status == LongitudinalOdometryUpdateStatus::INVALID_FRAME;
    result.sequence = frame.sequence;
    result.sampleTimestampUs = sampleTimestampUs;
    result.generation = m_generation;
    result.leftContinuityEpoch = frame.left.continuityEpoch;
    result.rightContinuityEpoch = frame.right.continuityEpoch;
    return result;
}

LongitudinalOdometryResult LongitudinalOdometry::update(const EncoderFrame& frame,
                                                        int64_t nowUs)
{
    const int64_t leftTimestampUs = frame.left.sampleTimestampUs;
    const int64_t rightTimestampUs = frame.right.sampleTimestampUs;
    const int64_t derivedTimestampUs = std::max(leftTimestampUs, rightTimestampUs);
    const int64_t sampleTimestampUs = frame.sampleTimestampUs > 0
        ? frame.sampleTimestampUs : derivedTimestampUs;

    if (m_hasSequence && frame.sequence == m_lastSequence) {
        auto result = statusResult(frame, LongitudinalOdometryUpdateStatus::DUPLICATE,
                                   sampleTimestampUs);
        result.frameAccepted = false;
        return result;
    }
    if (m_hasSequence && frame.sequence < m_lastSequence) {
        auto result = statusResult(frame, LongitudinalOdometryUpdateStatus::OUT_OF_ORDER,
                                   sampleTimestampUs);
        result.frameAccepted = false;
        return result;
    }
    if (sampleTimestampUs <= 0 ||
        (m_hasSequence && sampleTimestampUs <= m_lastSampleTimestampUs)) {
        auto result = statusResult(frame, LongitudinalOdometryUpdateStatus::OUT_OF_ORDER,
                                   sampleTimestampUs);
        result.frameAccepted = false;
        return result;
    }

    // A monotonically sequenced frame is consumed once, even when its timing
    // is invalid. Counts and continuity state are changed only below after
    // the timing checks pass.
    m_hasSequence = true;
    m_lastSequence = frame.sequence;
    m_lastSampleTimestampUs = sampleTimestampUs;

    const bool wheelTimestampsValid = leftTimestampUs > 0 && rightTimestampUs > 0;
    const int64_t timestampSkewUs = wheelTimestampsValid
        ? std::llabs(leftTimestampUs - rightTimestampUs) : 0;
    const bool skewValid = wheelTimestampsValid &&
        timestampSkewUs <= m_config.maxWheelTimestampSkewUs;
    const bool ageValid = wheelTimestampsValid &&
        (nowUs <= 0 || (nowUs >= leftTimestampUs && nowUs >= rightTimestampUs &&
            nowUs - leftTimestampUs <= m_config.maxSampleAgeUs &&
            nowUs - rightTimestampUs <= m_config.maxSampleAgeUs));
    const bool timingValid = m_scalingValid && wheelTimestampsValid && skewValid &&
        sampleTimestampUs >= std::max(leftTimestampUs, rightTimestampUs) && ageValid;

    if (!timingValid) {
        auto result = statusResult(frame, LongitudinalOdometryUpdateStatus::INVALID_FRAME,
                                   sampleTimestampUs);
        result.sampleTimingValid = false;
        result.leftFeedbackValid = false;
        result.rightFeedbackValid = false;
        result.velocityValid = false;
        result.continuityValid = false;
        result.positionValid = false;
        result.odometryValid = false;
        m_latest = result;
        return result;
    }

    const bool epochChanged = m_hasEpoch &&
        (frame.left.continuityEpoch != m_leftEpoch ||
         frame.right.continuityEpoch != m_rightEpoch);
    const bool continuityIssue = epochChanged ||
        frame.left.rebased || frame.right.rebased ||
        frame.left.error != ESP_OK || frame.right.error != ESP_OK;

    if (!m_hasEpoch) {
        m_leftEpoch = frame.left.continuityEpoch;
        m_rightEpoch = frame.right.continuityEpoch;
        m_hasEpoch = true;
    }
    if (continuityIssue) {
        markContinuityBroken(frame.left.continuityEpoch,
                             frame.right.continuityEpoch,
                             epochChanged);
        auto result = statusResult(frame, LongitudinalOdometryUpdateStatus::ACCEPTED,
                                   sampleTimestampUs);
        result.sampleTimingValid = true;
        result.leftFeedbackValid = frame.left.valid && frame.left.error == ESP_OK;
        result.rightFeedbackValid = frame.right.valid && frame.right.error == ESP_OK;
        result.velocityValid = false;
        result.continuityValid = false;
        result.positionValid = false;
        result.odometryValid = false;
        m_latest = result;
        return result;
    }

    if (!m_hasBase) {
        m_baseLeftCount = frame.left.logicalCount;
        m_baseRightCount = frame.right.logicalCount;
        m_hasBase = true;
        m_continuityBroken = false;
    }

    const int64_t leftDeltaCount = frame.left.logicalCount - m_baseLeftCount;
    const int64_t rightDeltaCount = frame.right.logicalCount - m_baseRightCount;
    const double leftPositionM = static_cast<double>(leftDeltaCount) *
        m_config.metersPerCountLeft * m_config.leftForwardSign;
    const double rightPositionM = static_cast<double>(rightDeltaCount) *
        m_config.metersPerCountRight * m_config.rightForwardSign;

    const bool leftSpeedFinite = frame.left.valid && std::isfinite(frame.left.speedDps);
    const bool rightSpeedFinite = frame.right.valid && std::isfinite(frame.right.speedDps);
    const float leftVelocityMps = leftSpeedFinite
        ? static_cast<float>(static_cast<double>(frame.left.speedDps) * DEG_TO_RAD *
            m_config.leftWheelRadiusM * m_config.leftForwardSign) : 0.0f;
    const float rightVelocityMps = rightSpeedFinite
        ? static_cast<float>(static_cast<double>(frame.right.speedDps) * DEG_TO_RAD *
            m_config.rightWheelRadiusM * m_config.rightForwardSign) : 0.0f;
    const bool velocityFinite = std::isfinite(leftVelocityMps) &&
        std::isfinite(rightVelocityMps);

    LongitudinalOdometryResult result = {};
    result.status = LongitudinalOdometryUpdateStatus::ACCEPTED;
    result.frameAccepted = true;
    result.sampleTimingValid = true;
    result.leftFeedbackValid = leftSpeedFinite;
    result.rightFeedbackValid = rightSpeedFinite;
    result.velocityValid = leftSpeedFinite && rightSpeedFinite && velocityFinite;
    result.continuityValid = true;
    result.positionValid = true;
    result.odometryValid = result.velocityValid;
    result.sequence = frame.sequence;
    result.odometrySequence = frame.sequence;
    result.generation = m_generation;
    result.leftContinuityEpoch = frame.left.continuityEpoch;
    result.rightContinuityEpoch = frame.right.continuityEpoch;
    result.sampleTimestampUs = sampleTimestampUs;
    result.leftPositionM = leftPositionM;
    result.rightPositionM = rightPositionM;
    result.positionM = (leftPositionM + rightPositionM) * 0.5;
    result.distanceDifferenceM = leftPositionM - rightPositionM;
    result.leftVelocityMps = leftVelocityMps;
    result.rightVelocityMps = rightVelocityMps;
    result.velocityMps = (leftVelocityMps + rightVelocityMps) * 0.5f;
    result.velocityDifferenceMps = leftVelocityMps - rightVelocityMps;
    m_latest = result;
    return result;
}
