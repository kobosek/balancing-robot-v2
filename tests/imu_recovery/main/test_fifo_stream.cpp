#include "unity.h"
#include "FIFOProcessor.hpp"
#include "OrientationEstimator.hpp"
#include "I2CDevice.hpp"
#include "sensor_fakes.hpp"
#include "esp_timer.h"
#include <limits>

TEST_CASE("FIFO partial destructive failures never publish or retry", "[imu][fifo]") {
    for (int consumed : {0, 7, 24}) {
        sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 24;
        I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
        MPU6050Driver driver(device); OrientationEstimator estimator;
        estimator.init();
        FIFOProcessor fifo(driver, estimator); fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
        sensor_fake::fifoFailureConsume = consumed;
        const auto before = estimator.getOrientation();
        const auto result = fifo.processFIFO(before.generation, 1020000);
        TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::LOST_ALIGNMENT), static_cast<int>(result.outcome));
        TEST_ASSERT_EQUAL_UINT(1, sensor_fake::fifoReads);
        TEST_ASSERT_TRUE(before.sample_sequence == estimator.getOrientation().sample_sequence);
        TEST_ASSERT_FALSE(estimator.getOrientation().valid);
        TEST_ASSERT_EQUAL_UINT64(2, estimator.getOrientation().lostSamples);
        TEST_ASSERT_TRUE(estimator.getOrientation().lossCountUncertain);
    }
    sensor_fake::clockUs = -1;
}
TEST_CASE("count retry and partial packet preserve accepted stream", "[imu][fifo]") {
    sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 29;
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
    MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
    FIFOProcessor fifo(driver, estimator); fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
    sensor_fake::failReadRegister = 0x72; sensor_fake::readFailures = 1;
    const auto generation = estimator.getOrientation().generation;
    const auto result = fifo.processFIFO(generation, 1020000);
    TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::ACCEPTED), static_cast<int>(result.outcome));
    TEST_ASSERT_TRUE(result.retried);
    TEST_ASSERT_EQUAL_UINT(2, result.accepted);
    TEST_ASSERT_EQUAL_UINT(2, sensor_fake::countReads);
    TEST_ASSERT_EQUAL_UINT(1, sensor_fake::fifoReads);
    TEST_ASSERT_EQUAL_UINT(generation, estimator.getOrientation().generation);
    TEST_ASSERT_FALSE(estimator.getOrientation().valid);
    sensor_fake::clockUs = -1;
}
TEST_CASE("FIFO timestamp lower bound holds while sensor keeps filling", "[imu][fifo]") {
    const int64_t period = 4000;
    for (int phase = 0; phase < period; phase += 137) {
        for (unsigned backlog = 1; backlog <= 4; ++backlog) {
            const int64_t start = 1000000;
            const int64_t newest = start - phase;
            for (unsigned index = 0; index < backlog; ++index) {
                const int64_t acquisition = newest - (backlog - index - 1) * period;
                const auto stamp = FIFOProcessor::sampleTimestamp(start, backlog, index, period);
                TEST_ASSERT_TRUE(stamp <= acquisition);
                // New samples arriving during a 5 ms transfer do not change old packet ages.
                TEST_ASSERT_TRUE(start + 5000 - stamp >= start + 5000 - acquisition);
            }
        }
    }
}
TEST_CASE("estimator boot reset finite checks and coherent metadata", "[imu][estimate]") {
    OrientationEstimator estimator;
    TEST_ASSERT_FALSE(estimator.getOrientation().valid);
    estimator.init();
    const auto generation = estimator.getOrientation().generation;
    for (int i = 0; i < 5; ++i)
        TEST_ASSERT_TRUE(estimator.processSample(0, 0, 1, 0, 0, 0, 84000 + i * 4000, generation));
    TEST_ASSERT_FALSE(estimator.getOrientation().valid);
    estimator.setValidated();
    const auto first = estimator.getOrientation();
    TEST_ASSERT_TRUE(first.fresh(119999, 20000));
    TEST_ASSERT_FALSE(first.fresh(120001, 20000));
    TEST_ASSERT_FALSE(first.fresh(99999, 20000));
    TEST_ASSERT_FALSE(estimator.processSample(0, 0, 1, 0, std::numeric_limits<float>::quiet_NaN(), 0, 104000, generation));
    TEST_ASSERT_TRUE(first.sample_sequence == estimator.getOrientation().sample_sequence);
    estimator.reset();
    TEST_ASSERT_FALSE(estimator.getOrientation().valid);
    TEST_ASSERT_FALSE(estimator.processSample(0, 0, 1, 0, 0, 0, 104000, generation));
}
TEST_CASE("every resync operation error reaches owner", "[imu][fifo]") {
    for (int reg : {0x23, 0x6a, 0x3a}) {
        sensor_fake::reset();
        I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
        MPU6050Driver driver(device); OrientationEstimator estimator; FIFOProcessor fifo(driver, estimator);
        if (reg == 0x3a) { sensor_fake::failReadRegister = reg; sensor_fake::readFailures = 1; }
        else { sensor_fake::failWriteRegister = reg; sensor_fake::writeFailures = 1; }
        TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, fifo.resync());
    }
}

TEST_CASE("complete FIFO packets do not receive a second artificial age period", "[imu][regression]") {
    TEST_ASSERT_TRUE(FIFOProcessor::sampleTimestamp(1000000, 2, 1, 4000) == 996000);
    TEST_ASSERT_TRUE(FIFOProcessor::sampleTimestamp(1000000, 2, 0, 4000) == 992000);
}
TEST_CASE("FIFO threshold yields to approaching sample freshness deadline", "[imu][regression]") {
    sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 12;
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
    MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
    FIFOProcessor fifo(driver, estimator);
    fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
    const auto result = fifo.processFIFO(estimator.getOrientation().generation, 1008000);
    const auto sample = estimator.getOrientation();
    sensor_fake::clockUs = -1;
    TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::ACCEPTED), static_cast<int>(result.outcome));
    TEST_ASSERT_EQUAL_UINT(1, result.accepted);
    TEST_ASSERT_TRUE(sample.sample_timestamp_us == 996000);
}
TEST_CASE("bounded FIFO catchup preserves three packet bursts at 100 kHz", "[imu][regression]") {
    sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 72;
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
    MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
    FIFOProcessor fifo(driver, estimator);
    fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
    const auto result = fifo.processFIFO(estimator.getOrientation().generation, 999000);
    const auto sample = estimator.getOrientation();
    sensor_fake::clockUs = -1;
    TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::ACCEPTED), static_cast<int>(result.outcome));
    TEST_ASSERT_EQUAL_UINT(3, result.accepted);
    TEST_ASSERT_TRUE(result.moreData);
    TEST_ASSERT_TRUE(sample.sample_timestamp_us == 984000);
}

TEST_CASE("corrupt zero or ones FIFO packets cannot refresh a valid estimate", "[imu][regression]") {
    for (int byte : {0, 255}) {
        sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 24;
        sensor_fake::fifoFillByte = byte;
        I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
        MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
        const auto generation = estimator.getOrientation().generation;
        for (int i = 0; i < 5; ++i) estimator.processSample(0, 0, 1, 0, 0, 0, 964000 + i * 4000, generation);
        estimator.setValidated();
        const auto before = estimator.getOrientation();
        FIFOProcessor fifo(driver, estimator);
        fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
        const auto result = fifo.processFIFO(generation, 1008000);
        const auto after = estimator.getOrientation();
        sensor_fake::clockUs = -1;
        TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::LOST_ALIGNMENT), static_cast<int>(result.outcome));
        TEST_ASSERT_TRUE(before.sample_sequence == after.sample_sequence);
        TEST_ASSERT_TRUE(before.sample_timestamp_us == after.sample_timestamp_us);
    }
}

TEST_CASE("ordered stale FIFO batch is drained without requesting repair", "[imu][regression]") {
    sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 120;
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
    MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
    const auto generation = estimator.getOrientation().generation;
    for (int i = 0; i < 5; ++i) estimator.processSample(0, 0, 1, 0, 0, 0, 934000 + i * 4000, generation);
    estimator.setValidated();
    FIFOProcessor fifo(driver, estimator);
    fifo.configure(MPU6050Profile::fromConfig(MPU6050Config{}), 24);
    const auto result = fifo.processFIFO(generation, 999000);
    const bool stale = !estimator.getOrientation().fresh(1000000, 20000);
    sensor_fake::fixedCount = 24; sensor_fake::clockUs = 1004000;
    const auto tail = fifo.processFIFO(generation, 999000);
    const bool fresh = estimator.getOrientation().fresh(1004000, 20000);
    sensor_fake::clockUs = -1;
    TEST_ASSERT_EQUAL(static_cast<int>(FIFOOutcome::ACCEPTED), static_cast<int>(result.outcome));
    TEST_ASSERT_TRUE(result.moreData && stale);
    TEST_ASSERT_TRUE(tail.accepted > 0 && fresh);
    TEST_ASSERT_EQUAL_UINT32(generation, estimator.getOrientation().generation);
}

TEST_CASE("FIFO ADC rails publish per-axis saturation in the same accepted sample", "[imu][fifo]") {
    sensor_fake::reset(); sensor_fake::clockUs = 1000000; sensor_fake::fixedCount = 24;
    sensor_fake::fifoSaturateAxes = 0x3f;
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_8, 0x68, 100000);
    MPU6050Driver driver(device); OrientationEstimator estimator; estimator.init();
    const auto profile = MPU6050Profile::fromConfig(MPU6050Config{});
    FIFOProcessor fifo(driver, estimator); fifo.configure(profile, 24);
    const auto result = fifo.processFIFO(estimator.getOrientation().generation, 1020000);
    const auto f = estimator.getOrientation();
    sensor_fake::clockUs = -1;
    TEST_ASSERT_EQUAL_UINT(2, result.accepted);
    TEST_ASSERT_EQUAL_UINT8(0x3f, f.saturationMask);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 32767.0f / profile.accelLsbPerG, f.ax_g);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -32768.0f / profile.accelLsbPerG, f.ay_g);
    TEST_ASSERT_EQUAL_UINT64(2, f.sample_sequence);
}
