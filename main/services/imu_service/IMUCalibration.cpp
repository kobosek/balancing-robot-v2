#include "IMUCalibration.hpp"

#include "mpu6050.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

IMUCalibration::IMUCalibration(MPU6050Driver& driver) :
    m_driver(driver) {
    m_calib_gx_samples.reserve(100);
    m_calib_gy_samples.reserve(100);
    m_calib_gz_samples.reserve(100);
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

    double gxSumDps = 0.0;
    double gySumDps = 0.0;
    double gzSumDps = 0.0;
    int successfulSamples = 0;
    int attempts = 0;
    const int maxAttempts = calibrationSamples + 200;
    esp_err_t lastReadError = ESP_OK;

    m_calib_gx_samples.clear();
    m_calib_gy_samples.clear();
    m_calib_gz_samples.clear();

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

        gxSumDps += gxDps;
        gySumDps += gyDps;
        gzSumDps += gzDps;
        m_calib_gx_samples.push_back(gxDps);
        m_calib_gy_samples.push_back(gyDps);
        m_calib_gz_samples.push_back(gzDps);
        successfulSamples++;

        if (progressCallback != nullptr &&
            calibrationSamples >= 5 &&
            successfulSamples > 0 &&
            (successfulSamples % (calibrationSamples / 5) == 0)) {
            progressCallback(successfulSamples, calibrationSamples);
        }
    }

    if (successfulSamples < calibrationSamples / 2) {
        return lastReadError != ESP_OK ? lastReadError : ESP_FAIL;
    }

    const float offsetGxDps = static_cast<float>(gxSumDps / successfulSamples);
    const float offsetGyDps = static_cast<float>(gySumDps / successfulSamples);
    const float offsetGzDps = static_cast<float>(gzSumDps / successfulSamples);

    m_gyro_offset_dps[0] = offsetGxDps;
    m_gyro_offset_dps[1] = offsetGyDps;
    m_gyro_offset_dps[2] = offsetGzDps;

    const auto calculateStdDev = [](const std::vector<float>& samples, double mean) {
        if (samples.empty()) {
            return 0.0;
        }
        const double squareSum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
        const double variance = std::max(0.0, squareSum / samples.size() - mean * mean);
        return std::sqrt(variance);
    };

    ESP_LOGI(TAG,
             "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f",
             successfulSamples,
             offsetGxDps,
             offsetGyDps,
             offsetGzDps,
             calculateStdDev(m_calib_gx_samples, offsetGxDps),
             calculateStdDev(m_calib_gy_samples, offsetGyDps),
             calculateStdDev(m_calib_gz_samples, offsetGzDps));

    return ESP_OK;
}
