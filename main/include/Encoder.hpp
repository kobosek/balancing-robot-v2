#pragma once

#include "interfaces/IEncoder.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <array>
#include "driver/pulse_cnt.h"

class Encoder : public IEncoder {
    public:
        Encoder(gpio_num_t pinA, gpio_num_t pinB) : m_pinA(pinA), m_pinB(pinB) {};
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        float getSpeed(float dt) override;
    private:
        static constexpr const char* TAG = "Encoder";

        static void IRAM_ATTR isrHandler(void* arg); 
        void IRAM_ATTR handleInterrupt();

        static void task_wrapper(void* arg);
        void calculate_speed_task(void* arg);    

        gpio_num_t m_pinA;
        gpio_num_t m_pinB;

        volatile uint64_t lastTime = 0;
        volatile int32_t position = 0;
        volatile int32_t lastPosition = 0;
        volatile uint8_t last_state = 0;
        volatile float last_speed = 0.0f;

        QueueHandle_t dtQueue;
};

class PCNTEncoder : public IEncoder {
    public:
        PCNTEncoder(gpio_num_t pinA, gpio_num_t pinB) : m_pinA(pinA), m_pinB(pinB), m_unit(NULL) {};
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        float getSpeed(float dt) override;
    private:
        static constexpr const char* TAG = "PCNTEncoder";
        
        gpio_num_t m_pinA;
        gpio_num_t m_pinB;
        pcnt_unit_handle_t m_unit;

        volatile int lastPulseCount = 0;
        volatile float last_speed = 0.0f;
        volatile float last_filtered_speed = 0.0f;
};