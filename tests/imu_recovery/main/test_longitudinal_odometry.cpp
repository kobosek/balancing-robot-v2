#include "unity.h"
#include "EncoderService.hpp"
#include "LongitudinalOdometry.hpp"
#include <cmath>

namespace {
EncoderFrame makeFrame(uint64_t sequence,
                       int64_t timestampUs,
                       int64_t leftCount,
                       int64_t rightCount,
                       uint32_t leftEpoch = 1,
                       uint32_t rightEpoch = 1,
                       float leftSpeedDps = 0.0f,
                       float rightSpeedDps = 0.0f,
                       bool leftValid = true,
                       bool rightValid = true)
{
    EncoderFrame frame = {};
    frame.sequence = sequence;
    frame.sampleTimestampUs = timestampUs;
    frame.left.logicalCount = leftCount;
    frame.right.logicalCount = rightCount;
    frame.left.sampleTimestampUs = timestampUs;
    frame.right.sampleTimestampUs = timestampUs;
    frame.left.continuityEpoch = leftEpoch;
    frame.right.continuityEpoch = rightEpoch;
    frame.left.speedDps = leftSpeedDps;
    frame.right.speedDps = rightSpeedDps;
    frame.left.valid = leftValid;
    frame.right.valid = rightValid;
    return frame;
}

LongitudinalOdometryConfig testConfig()
{
    EncoderConfig encoder;
    return longitudinalOdometryConfigFromEncoder(encoder, 20000, 1000);
}
}

TEST_CASE("longitudinal position uses signed counts and ignores speed filter alpha", "[odometry]") {
    auto config = testConfig();
    LongitudinalOdometry odometry(config);
    const auto first = makeFrame(1, 1000000, 0, 0);
    TEST_ASSERT_TRUE(odometry.update(first, 1000000).positionValid);

    const auto second = makeFrame(2, 1005000, 280, -280, 1, 1, 10.0f, -80.0f);
    const auto result = odometry.update(second, 1005000);
    const double expectedDistance = 280.0 * config.metersPerCountLeft;
    TEST_ASSERT_TRUE(result.positionValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, expectedDistance, result.positionM);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, expectedDistance, result.leftPositionM);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, expectedDistance, result.rightPositionM);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.0, result.distanceDifferenceM);
    TEST_ASSERT_TRUE(result.velocityValid);
    TEST_ASSERT_TRUE(std::fabs(result.leftVelocityMps - result.rightVelocityMps) > 0.001f);
}

TEST_CASE("odometry consumes each sequence once and rejects time regressions", "[odometry][ordering]") {
    LongitudinalOdometry odometry(testConfig());
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::ACCEPTED,
                      odometry.update(makeFrame(1, 1000000, 0, 0), 1000000).status);
    const auto accepted = odometry.update(makeFrame(2, 1005000, 10, -10), 1005000);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::ACCEPTED, accepted.status);
    const auto duplicate = odometry.update(makeFrame(2, 1005000, 200, -200), 1005000);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::DUPLICATE, duplicate.status);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, accepted.positionM, duplicate.positionM);

    const auto oldSequence = odometry.update(makeFrame(1, 1006000, 300, -300), 1006000);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::OUT_OF_ORDER, oldSequence.status);
    const auto oldTimestamp = odometry.update(makeFrame(3, 1004000, 300, -300), 1006000);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::OUT_OF_ORDER, oldTimestamp.status);
}

TEST_CASE("odometry rejects stale or skewed samples without moving the position", "[odometry][timing]") {
    auto config = testConfig();
    config.maxSampleAgeUs = 1000;
    config.maxWheelTimestampSkewUs = 100;
    LongitudinalOdometry odometry(config);
    TEST_ASSERT_TRUE(odometry.update(makeFrame(1, 1000000, 0, 0), 1000000).positionValid);
    const auto before = odometry.latest();

    auto skewed = makeFrame(2, 1005000, 100, -100);
    skewed.right.sampleTimestampUs += 101;
    const auto skewedResult = odometry.update(skewed, 1005000);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::INVALID_FRAME, skewedResult.status);
    TEST_ASSERT_FALSE(skewedResult.positionValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, before.positionM, skewedResult.positionM);

    const auto stale = odometry.update(makeFrame(3, 1006000, 200, -200), 1008001);
    TEST_ASSERT_EQUAL(LongitudinalOdometryUpdateStatus::INVALID_FRAME, stale.status);
    TEST_ASSERT_FALSE(stale.sampleTimingValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, before.positionM, stale.positionM);
}

TEST_CASE("continuity epoch invalidates the old base and recaptures without a position jump", "[odometry][continuity]") {
    LongitudinalOdometry odometry(testConfig());
    const auto first = odometry.update(makeFrame(1, 1000000, 100, -100), 1000000);
    TEST_ASSERT_TRUE(first.positionValid);
    const uint32_t initialGeneration = first.generation;

    const auto lost = odometry.update(makeFrame(2, 1005000, 120, -120, 2, 1, 0, 0, false, true), 1005000);
    TEST_ASSERT_FALSE(lost.positionValid);
    TEST_ASSERT_FALSE(lost.continuityValid);
    TEST_ASSERT_TRUE(lost.generation > initialGeneration);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, first.positionM, lost.positionM);

    const auto recaptured = odometry.update(makeFrame(3, 1010000, 130, -130, 2, 1), 1010000);
    TEST_ASSERT_TRUE(recaptured.positionValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.0, recaptured.positionM);

    const auto after = odometry.update(makeFrame(5, 1015000, 230, -230, 2, 1), 1015000);
    TEST_ASSERT_TRUE(after.positionValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, 100.0 * testConfig().metersPerCountLeft, after.positionM);
    TEST_ASSERT_EQUAL_UINT64(5, after.odometrySequence);
}
