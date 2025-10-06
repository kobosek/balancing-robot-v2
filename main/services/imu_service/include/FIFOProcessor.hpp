#pragma once

#include "esp_err.h"
#include "OrientationEstimator.hpp"
#include "EventBus.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstdint>

// Forward declarations
class MPU6050Driver;
class IMUHealthMonitor;
class IMU_CommunicationError;

class FIFOProcessor {
public:

    FIFOProcessor(MPU6050Driver& driver,
                  std::shared_ptr<OrientationEstimator> estimator,
                  IMUHealthMonitor& healthMonitor,
                  EventBus& eventBus);

    ~FIFOProcessor();

    bool processFIFO();

    void setScalingFactors(float accelLsbPerG, float gyroLsbPerDps);

    esp_err_t enableFIFO();
    esp_err_t enableFIFO(int interruptPin, bool activeHigh); // Overload that also registers ISR

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
    static constexpr uint16_t MAX_SAMPLES_PER_PIPELINE_CALL = 20; // Max MPU samples to process per task cycle

    MPU6050Driver& m_driver;
    std::shared_ptr<OrientationEstimator> m_estimator;
    IMUHealthMonitor& m_healthMonitor;
    EventBus& m_eventBus;

    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_dps;

    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];

    // ISR validation and safety
    std::atomic<bool> m_isr_active;  // Tracks if ISR is currently active to prevent reentrancy
    
    // ISR related members
    SemaphoreHandle_t m_dataReadySemaphore; // Semaphore for synchronization with FIFOTask
    bool m_isr_handler_installed;           // Whether the ISR handler is installed
    int m_interrupt_pin;                    // GPIO pin for the interrupt
    bool m_interrupt_active_high;           // Whether the interrupt is active high

    bool validateFIFOData(const uint8_t* data, size_t length);

    // Hard recovery sequence for FIFO/INT anomalies
    esp_err_t recoverFifoHard(const char* reason);

    // Soft misalignment handling
    uint8_t m_misalignment_strikes = 0;
    static constexpr uint8_t MISALIGNMENT_STRIKES_BEFORE_HARD_RECOVERY = 3;
};
