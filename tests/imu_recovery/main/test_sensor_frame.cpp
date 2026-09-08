#include "unity.h"
#include "OrientationEstimator.hpp"
#include "EncoderService.hpp"
#include "sensor_fakes.hpp"
#include "pcnt_fakes.hpp"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cmath>

namespace {
struct ImuWriter {
    OrientationEstimator* estimator;
    SemaphoreHandle_t done;
};
void publishImu(void* arg) {
    auto& writer = *static_cast<ImuWriter*>(arg);
    const auto generation = writer.estimator->getOrientation().generation;
    for (unsigned i = 1; i <= 10000; ++i) {
        IMUSampleMetadata metadata;
        metadata.saturationMask = i % 64;
        metadata.fifoRemainingPackets = i % 80;
        writer.estimator->processSample(i, 0, 1, 0, 0, i, i * 4000LL, generation, metadata);
        if (i % 100 == 0) vTaskDelay(1);
    }
    xSemaphoreGive(writer.done);
    vTaskDelete(nullptr);
}
struct EncoderWriter {
    EncoderService* encoders;
    SemaphoreHandle_t done;
};
void publishEncoders(void* arg) {
    auto& writer = *static_cast<EncoderWriter*>(arg);
    for (unsigned i = 1; i <= 10000; ++i) {
        pcnt_fake::advance(0, 7); pcnt_fake::advance(1, -7);
        sensor_fake::clockUs += 5000;
        writer.encoders->update();
        if (i % 100 == 0) vTaskDelay(1);
    }
    xSemaphoreGive(writer.done);
    vTaskDelete(nullptr);
}
}
TEST_CASE("SensorFrame cross-core readers never mix accepted sample fields", "[imu][concurrency]") {
    OrientationEstimator estimator; estimator.init();
    ImuWriter writer{&estimator, xSemaphoreCreateBinary()};
    TEST_ASSERT_NOT_NULL(writer.done);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(publishImu, "frame-writer", 4096, &writer,
        5, nullptr, 1 - xPortGetCoreID()));
    unsigned reads = 0, mismatches = 0;
    uint64_t previous = 0;
    while (xSemaphoreTake(writer.done, 0) != pdTRUE) {
        const auto f = estimator.getOrientation();
        if (f.sample_sequence) {
            ++reads;
            const auto n = f.sample_sequence;
            if (n < previous || f.sample_timestamp_us != static_cast<int64_t>(n * 4000) ||
                f.ax_g != static_cast<float>(n) || f.ay_g != 0 || f.az_g != 1 ||
                f.yaw_rate_dps != static_cast<float>(n) || f.saturationMask != n % 64 ||
                f.fifoRemainingPackets != n % 80 || !std::isfinite(f.pitch_deg)) ++mismatches;
            previous = n;
        }
        taskYIELD();
    }
    vSemaphoreDelete(writer.done);
    TEST_ASSERT_GREATER_THAN_UINT(0, reads);
    TEST_ASSERT_EQUAL_UINT(0, mismatches);
    TEST_ASSERT_EQUAL_UINT64(10000, estimator.getOrientation().sample_sequence);
}
TEST_CASE("EncoderFrame cross-core readers see matching counts deltas and times", "[encoder][concurrency]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderService encoders(EncoderConfig{}); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    EncoderWriter writer{&encoders, xSemaphoreCreateBinary()};
    TEST_ASSERT_NOT_NULL(writer.done);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreatePinnedToCore(publishEncoders, "encoder-writer", 4096, &writer,
        5, nullptr, 1 - xPortGetCoreID()));
    unsigned reads = 0, mismatches = 0;
    while (xSemaphoreTake(writer.done, 0) != pdTRUE) {
        const auto f = encoders.getFrame();
        if (f.left.valid) {
            ++reads;
            if (!f.right.valid || f.left.logicalCount != -f.right.logicalCount ||
                f.left.rawCount != -f.right.rawCount || f.left.deltaCount != 7 || f.right.deltaCount != -7 ||
                f.left.measurementPeriodUs != 5000 || f.right.measurementPeriodUs != 5000 ||
                f.left.sampleTimestampUs != f.right.sampleTimestampUs ||
                f.sampleTimestampUs != f.left.sampleTimestampUs) ++mismatches;
        }
        taskYIELD();
    }
    vSemaphoreDelete(writer.done); sensor_fake::clockUs = -1;
    TEST_ASSERT_GREATER_THAN_UINT(0, reads);
    TEST_ASSERT_EQUAL_UINT(0, mismatches);
}
TEST_CASE("SensorFrame loss and saturation metadata survive invalidation without advancing sequence", "[imu][estimate]") {
    OrientationEstimator estimator; estimator.init();
    IMUSampleMetadata metadata{3, 0x21};
    const auto generation = estimator.getOrientation().generation;
    TEST_ASSERT_TRUE(estimator.processSample(0.5, 0.25, 1, 0, 2, 3, 100000, generation, metadata));
    estimator.setValidated();
    const auto f = estimator.getOrientation();
    TEST_ASSERT_EQUAL_FLOAT(0.5, f.ax_g);
    TEST_ASSERT_EQUAL_FLOAT(0.25, f.ay_g);
    TEST_ASSERT_EQUAL_UINT8(0x21, f.saturationMask);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(SensorFIFOState::BACKLOG), static_cast<int>(f.fifoState));
    estimator.recordFifoLoss(7, true);
    auto lost = estimator.getOrientation();
    TEST_ASSERT_FALSE(lost.valid);
    TEST_ASSERT_EQUAL_UINT64(f.sample_sequence, lost.sample_sequence);
    TEST_ASSERT_EQUAL_UINT64(7, lost.lostSamples);
    TEST_ASSERT_TRUE(lost.lossCountUncertain);
    estimator.reset(); lost = estimator.getOrientation();
    TEST_ASSERT_EQUAL_UINT64(f.sample_sequence, lost.sample_sequence);
    TEST_ASSERT_EQUAL_UINT64(7, lost.lostSamples);
    TEST_ASSERT_TRUE(lost.lossCountUncertain);
}
