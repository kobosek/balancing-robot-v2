#include "IMUCalibration.hpp"

#include "mpu6050.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

namespace {
struct RunningStats {
    int count = 0;
    double mean = 0.0;
    double m2 = 0.0;

    void add(double sample) {
        count++;
        const double delta = sample - mean;
        mean += delta / count;
        const double delta2 = sample - mean;
        m2 += delta * delta2;
    }

    double stddev() const {
        if (count == 0) {
            return 0.0;
        }
        const double variance = std::max(0.0, m2 / count);
        return std::sqrt(variance);
    }
};

void paceCalibrationSampling(int64_t samplePeriodUs, int64_t& nextSampleDeadlineUs) {
    if (samplePeriodUs <= 0) {
        return;
    }

    const int64_t nowUs = esp_timer_get_time();
    if (nextSampleDeadlineUs == 0) {
        nextSampleDeadlineUs = nowUs + samplePeriodUs;
    }

    const int64_t remainingUs = nextSampleDeadlineUs - nowUs;
    if (remainingUs > 0) {
        const TickType_t delayTicks = pdMS_TO_TICKS(static_cast<uint32_t>((remainingUs + 999) / 1000));
        if (delayTicks > 0) {
            vTaskDelay(delayTicks);
        }
    }

    const int64_t afterDelayUs = esp_timer_get_time();
    do {
        nextSampleDeadlineUs += samplePeriodUs;
    } while (nextSampleDeadlineUs <= afterDelayUs);
}
} // namespace

IMUCalibration::IMUCalibration(MPU6050Driver& driver) :
    m_driver(driver) {
}

void IMUCalibration::setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps) {
    m_gyro_offset_dps[0] = x_offset_dps;
    m_gyro_offset_dps[1] = y_offset_dps;
    m_gyro_offset_dps[2] = z_offset_dps;
}

esp_err_t IMUCalibration::calibrate(const MPU6050Profile& profile,
                                    int calibrationSamples,
                                    std::function<void(int, int)> progressCallback) {
    if (profile.gyroLsbPerDps <= 0.0f || calibrationSamples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    RunningStats gxStats;
    RunningStats gyStats;
    RunningStats gzStats;
    int successfulSamples = 0;
    int attempts = 0;
    const int maxAttempts = calibrationSamples + 200;
    esp_err_t lastReadError = ESP_OK;
    const int64_t samplePeriodUs = profile.samplePeriodS > 0.0f ?
        std::max<int64_t>(1, static_cast<int64_t>(std::llround(static_cast<double>(profile.samplePeriodS) * 1000000.0))) :
        0;
    int64_t nextSampleDeadlineUs = 0;

    while (successfulSamples < calibrationSamples && attempts < maxAttempts) {
        attempts++;
        int16_t rawGx = 0;
        int16_t rawGy = 0;
        int16_t rawGz = 0;

        esp_err_t readRet = m_driver.readRawGyroXYZ(rawGx, rawGy, rawGz);
        if (readRet != ESP_OK) {
            lastReadError = readRet;
            continue;
        }

        const float gxDps = static_cast<float>(rawGx) / profile.gyroLsbPerDps;
        const float gyDps = static_cast<float>(rawGy) / profile.gyroLsbPerDps;
        const float gzDps = static_cast<float>(rawGz) / profile.gyroLsbPerDps;

        gxStats.add(gxDps);
        gyStats.add(gyDps);
        gzStats.add(gzDps);
        successfulSamples++;

        if (progressCallback != nullptr &&
            calibrationSamples >= 5 &&
            successfulSamples > 0 &&
            (successfulSamples % (calibrationSamples / 5) == 0)) {
            progressCallback(successfulSamples, calibrationSamples);
        }

        paceCalibrationSampling(samplePeriodUs, nextSampleDeadlineUs);
    }

    const int minSuccessfulSamples = std::max(1, (calibrationSamples + 1) / 2);
    if (successfulSamples < minSuccessfulSamples) {
        return lastReadError != ESP_OK ? lastReadError : ESP_FAIL;
    }

    const float offsetGxDps = static_cast<float>(gxStats.mean);
    const float offsetGyDps = static_cast<float>(gyStats.mean);
    const float offsetGzDps = static_cast<float>(gzStats.mean);

    m_gyro_offset_dps[0] = offsetGxDps;
    m_gyro_offset_dps[1] = offsetGyDps;
    m_gyro_offset_dps[2] = offsetGzDps;

    ESP_LOGI(TAG,
             "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f",
             successfulSamples,
             offsetGxDps,
             offsetGyDps,
             offsetGzDps,
             gxStats.stddev(),
             gyStats.stddev(),
             gzStats.stddev());

    return ESP_OK;
}
