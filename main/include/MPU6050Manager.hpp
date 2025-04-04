#pragma once

#include "interfaces/IMPU6050Manager.hpp"
#include "esp_log.h"
#include "include/mpu6050.hpp"
#include "freertos/FreeRTOS.h"

class MPU6050Manager : public IMPU6050Manager {
    public:
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        float calculatePitch(float&, float) const override; 
        float calculateFifoPitch(float&) const override;
    private:
        static constexpr const char* TAG = "MPU6050Manager";

        static void IRAM_ATTR isrHandler(void* arg); 
        void IRAM_ATTR handleInterrupt();
        static void fifo_task_wrapper(void *arg);
        void fifo_task();
        void processFifoData(uint8_t* data, uint16_t length);

        float _pitch_fifo = 0.0f;
        SemaphoreHandle_t _pitch_mutex;
        static constexpr size_t FIFO_BUFFER_SIZE = 1024;
        uint8_t _fifo_buffer[FIFO_BUFFER_SIZE];
        TaskHandle_t fifo_task_handle = nullptr;
        uint8_t data_ready_counter = 0;

        esp_err_t calibrateGyro();    
        MPU6050 _sensor;
        static constexpr float ALPHA = 0.98f;
        static constexpr int CALIBRATION_SAMPLES = 100;
        static constexpr uint32_t I2C_MASTER_FREQ_HZ = 400000;
        float _gyro_error = 0.0f;

};
