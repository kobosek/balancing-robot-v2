#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <mutex> // <<< Already Included
#include "esp_log.h"

// Forward Declarations
class MPU6050Driver;
class ConfigService;
class EventBus;
class BaseEvent;
struct MPU6050Config; // For accessing config details

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

    // <<< ADDED: Method for event subscriptions >>>
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "IMUCalibService";

    MPU6050Driver& m_driver;
    const MPU6050Config& m_config; // Reference to config struct for params
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

    void handleStartCalibrationRequest(const BaseEvent& event);
    float calculateGyroScaleFactor() const;
};