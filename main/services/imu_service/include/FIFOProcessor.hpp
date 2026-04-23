#pragma once

#include "IIMUDataSink.hpp"
#include "IMUDataReadyInterrupt.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstdint>

class IIMUFaultSink;
class IMUHealthMonitor;
class MPU6050Driver;

class FIFOProcessor {
public:
    FIFOProcessor(MPU6050Driver& driver,
                  IIMUDataSink& dataSink,
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

    SemaphoreHandle_t getDataReadySemaphore() const { return m_dataReadySemaphore; }
    static void IRAM_ATTR isrHandler(void* arg);

private:
    static constexpr const char* TAG = "FIFOProcessor";
    static constexpr size_t FIFO_PACKET_SIZE = 12;
    static constexpr size_t MAX_FIFO_BUFFER_SIZE = 1024;
    static constexpr uint16_t MAX_SAMPLES_PER_PIPELINE_CALL = 20;
    static constexpr uint32_t OUTLIER_LOG_INTERVAL = 25;

    MPU6050Driver& m_driver;
    IIMUDataSink& m_dataSink;
    IMUHealthMonitor& m_healthMonitor;
    IIMUFaultSink& m_faultSink;

    std::atomic<float> m_accel_scale_g_per_lsb;
    std::atomic<float> m_gyro_scale_dps_per_lsb;
    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];

    std::atomic<bool> m_isr_active;
    SemaphoreHandle_t m_dataReadySemaphore;
    bool m_isr_handler_installed;
    int m_interrupt_pin;
    bool m_interrupt_active_high;
    IMUDataReadyInterrupt m_irqHelper;
    std::atomic<uint16_t> m_fifo_read_threshold_bytes;
    uint32_t m_physics_outlier_count;

    void notifyDataReady();
    void clearPendingDataReadySignal();
    esp_err_t registerInterrupt(int interruptPin, bool activeHigh);
    esp_err_t unregisterInterrupt();
    esp_err_t recoverFifoHard(const char* reason);
};
