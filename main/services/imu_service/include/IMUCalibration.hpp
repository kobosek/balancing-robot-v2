#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include <vector>
#include <functional>
#include "ConfigData.hpp"

// Forward Declarations
class MPU6050Driver;

class IMUCalibration {
public:
    IMUCalibration(MPU6050Driver& driver);
    ~IMUCalibration() = default;

    esp_err_t calibrate(const MPU6050Config& config, 
                      std::function<void(int, int)> progressCallback = nullptr);

    float getGyroOffsetXDPS() const { return m_gyro_offset_dps[0]; }

    float getGyroOffsetYDPS() const { return m_gyro_offset_dps[1]; }


    float getGyroOffsetZDPS() const { return m_gyro_offset_dps[2]; }

    void setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps);

private:
    static constexpr const char* TAG = "IMUCalib";

    MPU6050Driver& m_driver;
    float m_gyro_offset_dps[3] = {0.0f, 0.0f, 0.0f}; 

    std::vector<float> m_calib_gx_samples;
    std::vector<float> m_calib_gy_samples;
    std::vector<float> m_calib_gz_samples;

    static float calculateGyroScaleFactor(const MPU6050Config& config);
};
