#pragma once

#include "IMUDataReadyInterrupt.hpp"
#include "OrientationEstimator.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstdint>
#include <memory>

class IIMUFaultSink;
class IMUHealthMonitor;
class MPU6050Driver;

class FIFOProcessor {
public:
    FIFOProcessor(MPU6050Driver& driver,
                  std::shared_ptr<OrientationEstimator> estimator,
                  IMUHealthMonitor& healthMonitor,
                  IIMUFaultSink& faultSink);
    ~FIFOProcessor();

    bool processFIFO();
    void configureReadout(float samplePeriodS, int fifoReadThresholdBytes);
    void setScalingFactors(float accelLsbPerG, float gyroLsbPerDps);

    esp_err_t enableFIFO();
    esp_err_t enableFIFO(int interruptPin, bool activeHigh);
    esp_err_t disableFIFO();
    esp_err_t resetFIFO();
    esp_err_t resetAndReEnableFIFO();

    void notifyDataReady();
    esp_err_t registerInterrupt(int interruptPin, bool activeHigh);
    esp_err_t unregisterInterrupt(int interruptPin);

    SemaphoreHandle_t getDataReadySemaphore() { return m_dataReadySemaphore; }
    static void IRAM_ATTR isrHandler(void* arg);

private:
    static constexpr const char* TAG = "FIFOProcessor";
    static constexpr size_t FIFO_PACKET_SIZE = 12;
    static constexpr size_t MAX_FIFO_BUFFER_SIZE = 1024;
    static constexpr uint16_t MAX_SAMPLES_PER_PIPELINE_CALL = 20;

    MPU6050Driver& m_driver;
    std::shared_ptr<OrientationEstimator> m_estimator;
    IMUHealthMonitor& m_healthMonitor;
    IIMUFaultSink& m_faultSink;

    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_dps;
    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];

    std::atomic<bool> m_isr_active;
    SemaphoreHandle_t m_dataReadySemaphore;
    bool m_isr_handler_installed;
    int m_interrupt_pin;
    bool m_interrupt_active_high;
    IMUDataReadyInterrupt m_irqHelper;
    uint16_t m_fifo_read_threshold_bytes;
    uint8_t m_misalignment_strikes = 0;

    bool validateFIFOData(const uint8_t* data, size_t length);
    esp_err_t recoverFifoHard(const char* reason);
};
