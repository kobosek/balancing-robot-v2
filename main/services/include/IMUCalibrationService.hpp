#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <mutex> 
#include "esp_log.h"
#include "ConfigData.hpp" // For MPU6050Config definition

// Forward Declarations
class MPU6050Driver;
class EventBus;
class BaseEvent;

class IMUCalibrationService {
public:
    IMUCalibrationService(MPU6050Driver& driver, const MPU6050Config& imuConfig, EventBus& bus);
    ~IMUCalibrationService();

    esp_err_t init();
    esp_err_t calibrate();

    float getGyroOffsetXDPS() const;
    float getGyroOffsetYDPS() const;
    float getGyroOffsetZDPS() const;
    bool isCalibrating() const;

    // Set gyro offsets from config (software offsets, not hardware registers)
    void setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps);

    // <<< ADDED: Method for event subscriptions >>>
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "IMUCalibService";

    MPU6050Driver& m_driver;
    MPU6050Config m_config; // Copy of config struct for params
    EventBus& m_eventBus;

    SemaphoreHandle_t m_mutex;
    volatile bool m_is_calibrating_flag;

    // Store offsets directly in DPS (Degrees Per Second)
    float m_gyro_offset_dps[3]; // Index 0:X, 1:Y, 2:Z

    // Temporary storage during calibration
    std::vector<float> m_calib_gx_samples;
    std::vector<float> m_calib_gy_samples;
    std::vector<float> m_calib_gz_samples;

    // Calculated scale factor based on config
    float m_gyro_lsb_per_dps;

    void handleStartCalibrationRequest(const BaseEvent& event); // Keep private if only called via lambda
    float calculateGyroScaleFactor() const;
};