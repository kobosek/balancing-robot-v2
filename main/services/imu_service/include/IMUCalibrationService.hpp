#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <mutex> 
#include "esp_log.h"
#include "ConfigData.hpp" 
#include <atomic>

// Forward Declarations
class MPU6050Driver;
class EventBus;
class BaseEvent;
enum class IMUState; 

class IMUCalibrationService {
public:
    IMUCalibrationService(MPU6050Driver& driver, const MPU6050Config& imuConfig, EventBus& bus);
    ~IMUCalibrationService();

    esp_err_t init();
    esp_err_t calibrate(); // Called by IMUCalibrationTask

    float getGyroOffsetXDPS() const;
    float getGyroOffsetYDPS() const;
    float getGyroOffsetZDPS() const;
    bool isCalibrating() const; // Checks internal flag

    void setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps);

    void subscribeToEvents(EventBus& bus); // Subscribes to IMU_StateChanged
    void notifyIMUStateChange(IMUState newState); // Called by IMUService or via IMU_StateChanged event
    
    // requestStateTransition removed as it was unused and calibration flow is event-driven.

private:
    static constexpr const char* TAG = "IMUCalibService";

    MPU6050Driver& m_driver;
    MPU6050Config m_config; 
    EventBus& m_eventBus;

    SemaphoreHandle_t m_mutex; // Protects access to calibration process and offsets
    std::atomic<bool> m_is_calibrating_flag; // True while calibrate() method is actively running
    
    std::atomic<IMUState> m_current_imu_state; // Aware of IMUService's state

    float m_gyro_offset_dps[3]; 

    std::vector<float> m_calib_gx_samples;
    std::vector<float> m_calib_gy_samples;
    std::vector<float> m_calib_gz_samples;

    float m_gyro_lsb_per_dps;

    float calculateGyroScaleFactor() const;
};