#include "unity.h"
#include "OrientationEstimator.hpp"
#include "MPU6050Profile.hpp"
#include "freertos/task.h"
#include <cmath>

namespace {
struct Stream {
    OrientationEstimator estimator;
    uint32_t generation;
    int64_t timestamp = 1000000;
    float dt;
    explicit Stream(float period = 0.004f) : dt(period) {
        estimator.init(0.98f, period);
        generation = estimator.getOrientation().generation;
    }
    void step(float ax = 0, float ay = 0, float az = 1, float gx = 0, float gy = 0, float gz = 0, uint8_t mask = 0) {
        timestamp += std::llround(dt * 1000000);
        TEST_ASSERT_TRUE(estimator.processSample(ax, ay, az, gx, gy, gz, timestamp, generation, {0, mask}));
    }
    void upright(unsigned samples = 5) { for (unsigned i = 0; i < samples; ++i) step(); }
};
}

TEST_CASE("orientation distinguishes upright side and inverted gravity", "[imu][orientation]") {
    for (unsigned pose = 0; pose < 3; ++pose) {
        Stream stream;
        for (unsigned i = 0; i < 5; ++i) stream.step(0, pose == 1 ? 1 : 0, pose == 0 ? 1 : pose == 2 ? -1 : 0);
        TEST_ASSERT_TRUE(stream.estimator.setValidated());
        const auto sample = stream.estimator.getOrientation();
        TEST_ASSERT_FLOAT_WITHIN(0.001f, pose * 90.0f, sample.tilt_deg);
        TEST_ASSERT_EQUAL(pose == 0, sample.withinTilt(5));
        TEST_ASSERT_EQUAL(pose == 0, sample.withinTilt(80));
    }
}

TEST_CASE("initialization needs consecutive gravity samples and accepts extreme pose as data", "[imu][orientation]") {
    Stream stream;
    for (unsigned i = 0; i < 10; ++i) stream.step(0.5f, 0, 0.1f);
    TEST_ASSERT_FALSE(stream.estimator.setValidated());
    stream.upright(4);
    TEST_ASSERT_FALSE(stream.estimator.setValidated());
    stream.step(1, 0, 2);
    stream.upright(4);
    TEST_ASSERT_FALSE(stream.estimator.setValidated());
    stream.upright(1);
    TEST_ASSERT_TRUE(stream.estimator.setValidated());
    Stream vertical;
    vertical.step(1, 0, 0);
    TEST_ASSERT_EQUAL_UINT64(1, vertical.estimator.getOrientation().sample_sequence);
    TEST_ASSERT_FALSE(vertical.estimator.setValidated()); // One sample is insufficient, but the packet is valid.
}

TEST_CASE("gyro clipping stays invalid through clean samples until generation reset", "[imu][orientation]") {
    Stream stream;
    stream.upright();
    TEST_ASSERT_TRUE(stream.estimator.setValidated());
    stream.step(0, 0, 1, 0, 500, 0, 0x10);
    stream.upright(10);
    TEST_ASSERT_FALSE(stream.estimator.setValidated());
    TEST_ASSERT_FALSE(stream.estimator.getOrientation().fresh(stream.timestamp, 20000));
    TEST_ASSERT_TRUE(stream.estimator.getOrientation().gyroContinuityLost);
    stream.estimator.reset();
    stream.generation = stream.estimator.getOrientation().generation;
    stream.upright();
    TEST_ASSERT_TRUE(stream.estimator.setValidated());
}

TEST_CASE("pitch correction recovers an error larger than five degrees", "[imu][regression]") {
    for (float target : {-15.0f, 15.0f}) {
        Stream stream;
        stream.upright(1250);
        const float theta = target * OrientationEstimator::DEG_TO_RAD;
        // A disagreement after a disturbance must not latch out gravity forever.
        for (unsigned i = 0; i < 5000; ++i)
            stream.step(-std::sin(theta), 0, std::cos(theta));
        TEST_ASSERT_FLOAT_WITHIN(0.5f, target, stream.estimator.getPitchDeg());
        stream.upright(5000);
        TEST_ASSERT_FLOAT_WITHIN(0.5f, 0, stream.estimator.getPitchDeg());
    }
}

TEST_CASE("normal accelerometer scale error does not block initialization", "[imu][regression]") {
    for (float magnitude : {0.9f, 1.1f}) {
        Stream stream;
        for (unsigned i = 0; i < 5; ++i) stream.step(0, 0, magnitude);
        TEST_ASSERT_TRUE(stream.estimator.setValidated());
        TEST_ASSERT_TRUE(stream.estimator.getOrientation().fresh(stream.timestamp, 20000));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, stream.estimator.getPitchDeg());
    }
}

TEST_CASE("legacy sensor rate signs do not depend on accelerometer Z sign", "[imu][regression]") {
    for (float z : {-1.0f, 1.0f}) {
        Stream stream;
        for (unsigned i = 0; i < 5; ++i) stream.step(0, 0, z);
        TEST_ASSERT_TRUE(stream.estimator.setValidated());
        // Reject acceleration correction to isolate the established gyro convention.
        for (unsigned i = 0; i < 250; ++i) stream.step(0, 0, 2 * z, 0, 10, 30);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 10, stream.estimator.getPitchDeg());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 10, stream.estimator.getPitchRateDPS());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 30, stream.estimator.getYawDeg());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 30, stream.estimator.getYawRateDPS());
    }
}

TEST_CASE("yaw accumulation retains subdegree accuracy over an hour", "[imu][orientation]") {
    Stream stream;
    for (unsigned i = 0; i < 900000; ++i) {
        stream.step(0, 0, 1, 0, 0, 60);
        if (i % 10000 == 0) vTaskDelay(1);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 216000, stream.estimator.getYawDeg());
}

TEST_CASE("gyro integration uses the configured sample period", "[imu][orientation]") {
    for (float period : {0.004f, 0.001f, 0.005f}) {
        Stream stream(period);
        stream.upright();
        for (unsigned i = 0; i < static_cast<unsigned>(std::lround(1.0f / period)); ++i)
            stream.step(0, 0, 2, 0, 10, 60);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 10, stream.estimator.getPitchDeg());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 60, stream.estimator.getYawDeg());
    }
}

TEST_CASE("timing includes all transactions and rejects excessive filter delay", "[imu][orientation]") {
    TEST_ASSERT_EQUAL_INT64(3240, MPU6050Profile::transferTimeUs(2, 100000));
    MPU6050Config config;
    TEST_ASSERT_EQUAL(3, MPU6050Profile::fromConfig(config).maxReadPackets);
    TEST_ASSERT_EQUAL_INT64(4320, MPU6050Profile::transferTimeUs(3, 100000));
    TEST_ASSERT_TRUE(MPU6050Profile::timingValid(config, 20, 5));
    config.dlpf_config = 6;
    TEST_ASSERT_FALSE(MPU6050Profile::timingValid(config, 20, 5));
}
