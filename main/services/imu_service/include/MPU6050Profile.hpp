#pragma once

#include "ConfigData.hpp"
#include "driver/gpio.h"
#include "mpu6050.hpp"
#include <cstdint>

struct MPU6050Profile {
    static constexpr float ACCEL_LSB_PER_G_2G = 16384.0f;
    static constexpr float ACCEL_LSB_PER_G_4G = 8192.0f;
    static constexpr float ACCEL_LSB_PER_G_8G = 4096.0f;
    static constexpr float ACCEL_LSB_PER_G_16G = 2048.0f;
    static constexpr float GYRO_LSB_PER_DPS_250 = 131.0f;
    static constexpr float GYRO_LSB_PER_DPS_500 = 65.5f;
    static constexpr float GYRO_LSB_PER_DPS_1000 = 32.8f;
    static constexpr float GYRO_LSB_PER_DPS_2000 = 16.4f;
    static constexpr float DEFAULT_ACCEL_LSB_PER_G = ACCEL_LSB_PER_G_4G;
    static constexpr float DEFAULT_GYRO_LSB_PER_DPS = GYRO_LSB_PER_DPS_500;
    static constexpr float DEFAULT_SAMPLE_PERIOD_S = 0.001f;

    MPU6050AccelConfig accelRangeReg = MPU6050AccelConfig::RANGE_4G;
    MPU6050GyroConfig gyroRangeReg = MPU6050GyroConfig::RANGE_500_DEG;
    MPU6050DLPFConfig dlpfReg = MPU6050DLPFConfig::DLPF_BW_44HZ_ACC_42HZ_GYRO;
    MPU6050SampleRateDiv sampleRateDivReg = MPU6050SampleRateDiv::RATE_1KHZ;
    MPU6050InterruptPinConfig interruptPinConfig = MPU6050InterruptPinConfig::ACTIVE_HIGH;
    bool interruptEnabled = true;
    bool sampleRateLimitedByTransport = false;
    float accelLsbPerG = DEFAULT_ACCEL_LSB_PER_G;
    float gyroLsbPerDps = DEFAULT_GYRO_LSB_PER_DPS;
    float samplePeriodS = DEFAULT_SAMPLE_PERIOD_S;

    static MPU6050Profile fromConfig(const MPU6050Config& config) {
        MPU6050Profile profile;

        switch (config.accel_range) {
            case 0:
                profile.accelRangeReg = MPU6050AccelConfig::RANGE_2G;
                profile.accelLsbPerG = ACCEL_LSB_PER_G_2G;
                break;
            case 2:
                profile.accelRangeReg = MPU6050AccelConfig::RANGE_8G;
                profile.accelLsbPerG = ACCEL_LSB_PER_G_8G;
                break;
            case 3:
                profile.accelRangeReg = MPU6050AccelConfig::RANGE_16G;
                profile.accelLsbPerG = ACCEL_LSB_PER_G_16G;
                break;
            case 1:
            default:
                profile.accelRangeReg = MPU6050AccelConfig::RANGE_4G;
                profile.accelLsbPerG = ACCEL_LSB_PER_G_4G;
                break;
        }

        switch (config.gyro_range) {
            case 0:
                profile.gyroRangeReg = MPU6050GyroConfig::RANGE_250_DEG;
                profile.gyroLsbPerDps = GYRO_LSB_PER_DPS_250;
                break;
            case 2:
                profile.gyroRangeReg = MPU6050GyroConfig::RANGE_1000_DEG;
                profile.gyroLsbPerDps = GYRO_LSB_PER_DPS_1000;
                break;
            case 3:
                profile.gyroRangeReg = MPU6050GyroConfig::RANGE_2000_DEG;
                profile.gyroLsbPerDps = GYRO_LSB_PER_DPS_2000;
                break;
            case 1:
            default:
                profile.gyroRangeReg = MPU6050GyroConfig::RANGE_500_DEG;
                profile.gyroLsbPerDps = GYRO_LSB_PER_DPS_500;
                break;
        }

        uint8_t dlpfValue = static_cast<uint8_t>(config.dlpf_config);
        if (dlpfValue > static_cast<uint8_t>(MPU6050DLPFConfig::DLPF_BW_5HZ_ACC_5HZ_GYRO)) {
            dlpfValue = static_cast<uint8_t>(MPU6050DLPFConfig::DLPF_BW_260HZ_ACC_256HZ_GYRO);
        }
        profile.dlpfReg = static_cast<MPU6050DLPFConfig>(dlpfValue);

        uint8_t sampleRateValue = static_cast<uint8_t>(config.sample_rate_divisor & 0xFF);
        const uint8_t minimumSafeDivider = minimumSampleRateDividerForBus(config.i2c_freq_hz);
        if (sampleRateValue < minimumSafeDivider) {
            sampleRateValue = minimumSafeDivider;
            profile.sampleRateLimitedByTransport = true;
        }
        profile.sampleRateDivReg = static_cast<MPU6050SampleRateDiv>(sampleRateValue);

        profile.interruptPinConfig = config.interrupt_active_high ?
            MPU6050InterruptPinConfig::ACTIVE_HIGH :
            MPU6050InterruptPinConfig::ACTIVE_LOW;
        profile.interruptEnabled = (config.int_pin >= 0 && config.int_pin < GPIO_NUM_MAX);

        const bool dlpfEnabled = dlpfValue >= 1 && dlpfValue <= 6;
        const float gyroOutputRateHz = dlpfEnabled ? 1000.0f : 8000.0f;
        profile.samplePeriodS = 1.0f / (gyroOutputRateHz / (1.0f + static_cast<float>(sampleRateValue)));

        return profile;
    }

private:
    static uint8_t minimumSampleRateDividerForBus(uint32_t i2cFrequencyHz) {
        if (i2cFrequencyHz <= 100000) {
            return static_cast<uint8_t>(MPU6050SampleRateDiv::RATE_250HZ);
        }
        if (i2cFrequencyHz <= 200000) {
            return static_cast<uint8_t>(MPU6050SampleRateDiv::RATE_500HZ);
        }
        return static_cast<uint8_t>(MPU6050SampleRateDiv::RATE_1KHZ);
    }
};
