#include "unity.h"
#include "EncoderService.hpp"
#include "pcnt_fakes.hpp"
#include "sensor_fakes.hpp"
#include <algorithm>
#include <cmath>

TEST_CASE("PCNT accumulated counts cross both limits on both wheels without speed spikes", "[encoder]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderConfig config; config.speed_filter_alpha = 1;
    EncoderService encoders(config);
    TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    int64_t total = 0, maxDelta = 0;
    const float expectedSpeed = 7 * 360.0f / (28 * 100) / 0.005f;
    // Cross +30000, reverse through zero, cross -30000, then reverse again.
    for (const int steps : {5000, -10000, 10000}) {
        const int delta = steps > 0 ? 7 : -7;
        for (int i = 0; i < std::abs(steps); ++i) {
            pcnt_fake::advance(0, delta); pcnt_fake::advance(1, -delta);
            sensor_fake::clockUs += 5000; encoders.update(); total += delta;
            const auto f = encoders.getFrame();
            TEST_ASSERT_TRUE(f.left.valid && f.right.valid);
            TEST_ASSERT_EQUAL_INT64(total, f.left.rawCount);
            TEST_ASSERT_EQUAL_INT64(-total, f.right.rawCount);
            TEST_ASSERT_EQUAL_INT64(delta, f.left.deltaCount);
            TEST_ASSERT_EQUAL_INT64(-delta, f.right.deltaCount);
            TEST_ASSERT_FLOAT_WITHIN(0.001f, delta > 0 ? expectedSpeed : -expectedSpeed, f.left.speedDps);
            TEST_ASSERT_FLOAT_WITHIN(0.001f, delta > 0 ? -expectedSpeed : expectedSpeed, f.right.speedDps);
            maxDelta = std::max(maxDelta, std::abs(f.left.deltaCount));
        }
    }
    TEST_ASSERT_EQUAL_INT64(7, maxDelta);
    TEST_ASSERT_EQUAL_UINT(0, pcnt_fake::orderingErrors);
    sensor_fake::clockUs = -1;
}
TEST_CASE("PCNT setup propagates watchpoint clear enable and start errors without leaking units", "[encoder]") {
    for (auto op : {pcnt_fake::Operation::LOW_WATCH, pcnt_fake::Operation::HIGH_WATCH,
         pcnt_fake::Operation::CLEAR, pcnt_fake::Operation::ENABLE, pcnt_fake::Operation::START}) {
        for (int wheel : {0, 1}) {
            pcnt_fake::reset(); pcnt_fake::failNext = op; pcnt_fake::failWheel = wheel;
            { EncoderService encoders(EncoderConfig{}); TEST_ASSERT_EQUAL(ESP_FAIL, encoders.init()); }
            TEST_ASSERT_EQUAL_UINT(0, pcnt_fake::liveUnits);
        }
    }
}
TEST_CASE("encoder speed uses measured time and failed reads cannot retain valid speed", "[encoder]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderConfig config; config.speed_filter_alpha = 1;
    EncoderService encoders(config); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    pcnt_fake::advance(0, 42); pcnt_fake::advance(1, -42);
    sensor_fake::clockUs += 30000; encoders.update();
    auto f = encoders.getFrame();
    TEST_ASSERT_EQUAL_INT64(30000, f.left.measurementPeriodUs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180, f.left.speedDps);
    pcnt_fake::failNext = pcnt_fake::Operation::READ; pcnt_fake::failWheel = 0;
    sensor_fake::clockUs += 5000; encoders.update(); f = encoders.getFrame();
    TEST_ASSERT_FALSE(f.left.valid); TEST_ASSERT_TRUE(f.right.valid);
    TEST_ASSERT_EQUAL_FLOAT(0, f.left.speedDps);
    sensor_fake::clockUs += 5000; encoders.update();
    TEST_ASSERT_FALSE(encoders.getFrame().left.valid); // Re-establish baseline.
    sensor_fake::clockUs += 5000; pcnt_fake::advance(0, 7); encoders.update();
    TEST_ASSERT_TRUE(encoders.getFrame().left.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180, encoders.getFrame().left.speedDps);
    sensor_fake::clockUs = -1;
}
TEST_CASE("encoder rebasing prevents signed accumulator overflow and reports interrupted continuity", "[encoder]") {
    for (int sign : {-1, 1}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderService encoders(EncoderConfig{}); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        pcnt_fake::setCount(0, sign * ((1 << 28) - 7)); encoders.reset();
        pcnt_fake::advance(0, sign * 7); sensor_fake::clockUs += 5000; encoders.update();
        auto f = encoders.getFrame();
        TEST_ASSERT_FALSE(f.left.valid); TEST_ASSERT_TRUE(f.left.rebased && f.left.continuityLost);
        TEST_ASSERT_EQUAL_INT64(sign * 7, f.left.logicalCount);
        pcnt_fake::failNext = pcnt_fake::Operation::CLEAR; pcnt_fake::failWheel = 0;
        sensor_fake::clockUs += 5000; encoders.update();
        TEST_ASSERT_FALSE(encoders.getFrame().left.valid);
        sensor_fake::clockUs += 5000; encoders.update(); // Clear/start and seed.
        pcnt_fake::advance(0, sign * 7); sensor_fake::clockUs += 5000; encoders.update();
        f = encoders.getFrame();
        TEST_ASSERT_TRUE(f.left.valid && f.left.continuityLost);
        TEST_ASSERT_EQUAL_INT64(sign * 14, f.left.logicalCount);
        TEST_ASSERT_EQUAL_INT64(sign * 7, f.left.deltaCount);
        TEST_ASSERT_EQUAL_UINT(0, pcnt_fake::orderingErrors);
    }
    sensor_fake::clockUs = -1;
}
TEST_CASE("half-limit encoder discontinuity is rejected instead of wrap-corrected", "[encoder]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderService encoders(EncoderConfig{}); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    pcnt_fake::setCount(0, 30000); sensor_fake::clockUs += 5000; encoders.update();
    const auto f = encoders.getFrame();
    TEST_ASSERT_FALSE(f.left.valid);
    TEST_ASSERT_EQUAL_FLOAT(0, f.left.speedDps);
    TEST_ASSERT_TRUE(f.left.continuityLost && f.right.valid);
    sensor_fake::clockUs = -1;
}

TEST_CASE("encoder catch-up iterations retain the previous frame until a measurable interval", "[encoder]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderConfig config; config.speed_filter_alpha = 1;
    EncoderService encoders(config); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    sensor_fake::clockUs += 5000; pcnt_fake::advance(0, 7); encoders.update();
    const auto before = encoders.getFrame();
    sensor_fake::clockUs += 1; pcnt_fake::advance(0, 1); encoders.update();
    TEST_ASSERT_EQUAL_UINT64(before.sequence, encoders.getFrame().sequence);
    TEST_ASSERT_EQUAL_FLOAT(before.left.speedDps, encoders.getFrame().left.speedDps);
    sensor_fake::clockUs += 4999; pcnt_fake::advance(0, 6); encoders.update();
    TEST_ASSERT_EQUAL_INT64(7, encoders.getFrame().left.deltaCount);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180, encoders.getFrame().left.speedDps);
    sensor_fake::clockUs = -1;
}

TEST_CASE("PCNT rebase stop and restart failures remain visible as invalid frames", "[encoder]") {
    for (auto op : {pcnt_fake::Operation::STOP, pcnt_fake::Operation::START}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderService encoders(EncoderConfig{}); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        pcnt_fake::setCount(0, (1 << 28) - 7); encoders.reset();
        pcnt_fake::failNext = op; pcnt_fake::failWheel = 0;
        pcnt_fake::advance(0, 7); sensor_fake::clockUs += 5000; encoders.update();
        TEST_ASSERT_FALSE(encoders.getFrame().left.valid);
        if (op == pcnt_fake::Operation::STOP) {
            TEST_ASSERT_EQUAL(ESP_FAIL, encoders.getFrame().left.error);
            sensor_fake::clockUs += 5000; encoders.update(); // Retry stopping.
        } else {
            sensor_fake::clockUs += 5000; encoders.update(); // Clear succeeded, restart failed.
            TEST_ASSERT_EQUAL(ESP_FAIL, encoders.getFrame().left.error);
        }
        TEST_ASSERT_FALSE(encoders.getFrame().left.valid);
        sensor_fake::clockUs += 5000; encoders.update(); // Restart and seed only.
        TEST_ASSERT_FALSE(encoders.getFrame().left.valid);
        sensor_fake::clockUs += 5000; encoders.update();
        TEST_ASSERT_TRUE(encoders.getFrame().left.valid);
    }
    sensor_fake::clockUs = -1;
}

TEST_CASE("encoder filter retains nominal response and equal decay over jittered intervals", "[encoder][timing]") {
    float regular = 0, jittered = 0;
    for (int variant : {0, 1}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderService encoders(EncoderConfig{}, 5000); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        const int intervals[8] = {1, 9, 2, 8, 1, 9, 2, 8};
        float expected = 0;
        const float speed = 360.0f / 2.8f; // One pulse/ms, both schedules cover 40 ms.
        for (int i = 0; i < 8; ++i) {
            const int ms = variant ? intervals[i] : 5;
            pcnt_fake::advance(0, ms); pcnt_fake::advance(1, -ms);
            sensor_fake::clockUs += ms * 1000; encoders.update();
            const auto f = encoders.getFrame();
            TEST_ASSERT_TRUE(f.left.valid && f.right.valid);
            TEST_ASSERT_FLOAT_WITHIN(0.001f, -f.left.speedDps, f.right.speedDps);
            if (!variant) {
                expected = 0.1f * speed + 0.9f * expected;
                TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, f.left.speedDps);
            }
        }
        if (variant) jittered = encoders.getFrame().left.speedDps;
        else regular = encoders.getFrame().left.speedDps;
    }
    sensor_fake::clockUs = -1;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, regular, jittered);
}
TEST_CASE("short encoder interval no longer amplifies a single pulse by five", "[encoder][timing]") {
    float nominal = 0, shortInterval = 0;
    for (int periodUs : {5000, 1000}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderService encoders(EncoderConfig{}, 5000); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        sensor_fake::clockUs += periodUs; pcnt_fake::advance(0, 1); encoders.update();
        const auto f = encoders.getFrame();
        TEST_ASSERT_TRUE(f.left.valid);
        if (periodUs == 5000) nominal = f.left.speedDps;
        else shortInterval = f.left.speedDps;
    }
    sensor_fake::clockUs = -1;
    TEST_ASSERT_TRUE(shortInterval > nominal && shortInterval < nominal * 1.05f);
}
TEST_CASE("encoder filter uses the configured nominal loop period and alpha endpoints", "[encoder][timing]") {
    for (float alpha : {0.0f, 0.1f, 1.0f}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderConfig config; config.speed_filter_alpha = alpha;
        EncoderService encoders(config, 10000); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        sensor_fake::clockUs += 10000; pcnt_fake::advance(0, 10); encoders.update();
        const float expected = alpha * 360.0f / 2.8f;
        TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, encoders.getFrame().left.speedDps);
        sensor_fake::clockUs += 1000; pcnt_fake::advance(0, 1); encoders.update();
        TEST_ASSERT_TRUE(std::isfinite(encoders.getFrame().left.speedDps));
    }
    sensor_fake::clockUs = -1;
}

TEST_CASE("PCNT reset observed before accumulator ISR gets one bounded re-read", "[encoder][timing]") {
    for (int sign : {-1, 1}) {
        pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
        EncoderConfig config; config.speed_filter_alpha = 1;
        EncoderService encoders(config); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
        pcnt_fake::setCount(0, sign * 29993); pcnt_fake::setCount(1, -sign * 29993);
        encoders.reset();
        pcnt_fake::advance(0, sign * 7); pcnt_fake::advance(1, -sign * 7);
        pcnt_fake::reportBeforeOverflowISR(0, sign * 30000);
        pcnt_fake::reportBeforeOverflowISR(1, -sign * 30000);
        sensor_fake::clockUs += 5000; encoders.update();
        const auto f = encoders.getFrame();
        TEST_ASSERT_TRUE(f.left.valid && f.right.valid);
        TEST_ASSERT_EQUAL_INT64(sign * 7, f.left.deltaCount);
        TEST_ASSERT_EQUAL_INT64(-sign * 7, f.right.deltaCount);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, sign * 180, f.left.speedDps);
        TEST_ASSERT_FALSE(f.left.continuityLost || f.right.continuityLost);
    }
    sensor_fake::clockUs = -1;
}
