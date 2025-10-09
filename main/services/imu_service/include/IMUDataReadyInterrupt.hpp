#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include <atomic>

class IMUDataReadyInterrupt {
public:
    IMUDataReadyInterrupt();
    ~IMUDataReadyInterrupt();

    // Configure GPIO and register ISR handler. Safe to call multiple times; reconfigures the same pin.
    esp_err_t init(gpio_num_t pin, bool activeHigh, gpio_isr_t cb, void* arg);

    // Remove ISR handler for the configured pin.
    esp_err_t deinit();

    bool isInstalled() const { return m_handlerInstalled; }
    gpio_num_t pin() const { return m_pin; }
    bool activeHigh() const { return m_activeHigh; }

private:
    static esp_err_t ensureServiceInstalled();

    static std::atomic<bool> s_serviceInstalled;

    gpio_num_t m_pin;
    bool m_activeHigh;
    bool m_handlerInstalled;
};
