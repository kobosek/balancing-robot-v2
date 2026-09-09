#pragma once

#include "driver/gpio.h"
#include "mpu6050.hpp"
#include "config/MPU6050Config.hpp"
#include <cstdint>
#include <algorithm>

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
    static constexpr float DEFAULT_SAMPLE_PERIOD_S = 0.004f;

    MPU6050AccelConfig accelRangeReg = MPU6050AccelConfig::RANGE_4G;
    MPU6050GyroConfig gyroRangeReg = MPU6050GyroConfig::RANGE_500_DEG;
    MPU6050DLPFConfig dlpfReg = MPU6050DLPFConfig::DLPF_BW_44HZ_ACC_42HZ_GYRO;
    MPU6050SampleRateDiv sampleRateDivReg = MPU6050SampleRateDiv::RATE_250HZ;
    MPU6050InterruptPinConfig interruptPinConfig = MPU6050InterruptPinConfig::ACTIVE_HIGH;
    bool interruptEnabled = true;
    uint32_t samplePeriodUs = 4000;
    unsigned maxReadPackets = 4;
    float accelLsbPerG = DEFAULT_ACCEL_LSB_PER_G;
    float gyroLsbPerDps = DEFAULT_GYRO_LSB_PER_DPS;
    float samplePeriodS = DEFAULT_SAMPLE_PERIOD_S;
    uint32_t filterDelayUs = 4900;

    static int64_t transferTimeUs(unsigned packets, uint32_t frequency) {
        if (!frequency) return INT64_MAX;
        // Three transmit/receive transactions: count (5), data (12N+3), status (4).
        return ((12LL * packets + 12) * 9 * 1000000 + frequency - 1) / frequency;
    }

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
        constexpr uint32_t filterDelaysUs[] = {980, 2000, 3000, 4900, 8500, 13800, 19000};
        profile.filterDelayUs = filterDelaysUs[dlpfValue];

        uint8_t sampleRateValue = static_cast<uint8_t>(config.sample_rate_divisor & 0xFF);
        profile.sampleRateDivReg = static_cast<MPU6050SampleRateDiv>(sampleRateValue);

        profile.interruptPinConfig = config.interrupt_active_high ?
            MPU6050InterruptPinConfig::ACTIVE_HIGH :
            MPU6050InterruptPinConfig::ACTIVE_LOW;
        profile.interruptEnabled = (config.int_pin >= 0 && config.int_pin < GPIO_NUM_MAX);

        const bool dlpfEnabled = dlpfValue >= 1 && dlpfValue <= 6;
        const float gyroOutputRateHz = dlpfEnabled ? 1000.0f : 8000.0f;
        profile.samplePeriodS = 1.0f / (gyroOutputRateHz / (1.0f + static_cast<float>(sampleRateValue)));

        profile.samplePeriodUs = (1000000U * (sampleRateValue + 1)) / static_cast<unsigned>(gyroOutputRateHz);
        // Bound the destructive FIFO burst to 4 ms. Count/status are separate
        // transactions, included in transferTimeUs for full-cycle validation.
        // Subtracting their overhead here reduced 100 kHz catch-up from 3 to 2.
        const uint32_t bytes = config.i2c_freq_hz * 4ULL / 9000;
        profile.maxReadPackets = std::min<unsigned>(4, bytes > 3 ? (bytes - 3) / 12 : 0);
        return profile;
    }

    static bool timingValid(const MPU6050Config& config, int ageMs, int controlMs) {
        const auto p = fromConfig(config);
        const unsigned threshold = (config.fifo_read_threshold + 11) / 12;
        if (!p.maxReadPackets || !p.samplePeriodUs || threshold > p.maxReadPackets) return false;
        const int64_t transferUs = transferTimeUs(threshold, config.i2c_freq_hz);
        // Age of the newest packet (not the oldest packet in the batch), filter
        // group delay, full transfer, one scheduler tick and a control period.
        // Runtime coalescing must still yield to the cached sample deadline.
        return p.samplePeriodUs + p.filterDelayUs + transferUs + 1000 + controlMs * 1000LL < ageMs * 1000LL &&
            (12 * 9LL * 1000000 / config.i2c_freq_hz) * 2 < p.samplePeriodUs &&
            5LL * p.samplePeriodUs < 250000;
    }
};
