#include "IMUDataReadyInterrupt.hpp"
#include "esp_log.h"

std::atomic<bool> IMUDataReadyInterrupt::s_serviceInstalled{false};

static const char* TAG_IRQ = "IMUDataReadyIRQ";

IMUDataReadyInterrupt::IMUDataReadyInterrupt()
    : m_pin(GPIO_NUM_MAX), m_activeHigh(true), m_handlerInstalled(false) {}

IMUDataReadyInterrupt::~IMUDataReadyInterrupt() {
    // Ensure handler is removed
    (void)deinit();
}

esp_err_t IMUDataReadyInterrupt::ensureServiceInstalled() {
    if (s_serviceInstalled.load(std::memory_order_acquire)) {
        return ESP_OK;
    }
    // Try to install once
    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    if (ret == ESP_OK) {
        s_serviceInstalled.store(true, std::memory_order_release);
        ESP_LOGD(TAG_IRQ, "ISR service installed");
        return ESP_OK;
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        // Already installed elsewhere
        s_serviceInstalled.store(true, std::memory_order_release);
        ESP_LOGV(TAG_IRQ, "ISR service already installed");
        return ESP_OK;
    }
    ESP_LOGE(TAG_IRQ, "Failed to install ISR service: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUDataReadyInterrupt::init(gpio_num_t pin, bool activeHigh, gpio_isr_t cb, void* arg) {
    if (pin < 0 || pin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG_IRQ, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }
    if (cb == nullptr) {
        ESP_LOGE(TAG_IRQ, "Callback cannot be null");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure GPIO for interrupt
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = activeHigh ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = activeHigh ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = activeHigh ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IRQ, "Failed to configure GPIO %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    ret = ensureServiceInstalled();
    if (ret != ESP_OK) {
        return ret;
    }

    // Remove any previous handler on this pin then add new one
    gpio_isr_handler_remove(pin);
    ret = gpio_isr_handler_add(pin, cb, arg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IRQ, "Failed to add ISR handler for pin %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    m_pin = pin;
    m_activeHigh = activeHigh;
    m_handlerInstalled = true;
    ESP_LOGI(TAG_IRQ, "ISR handler installed on pin %d (active %s)", pin, activeHigh ? "high" : "low");
    return ESP_OK;
}

esp_err_t IMUDataReadyInterrupt::deinit() {
    if (!m_handlerInstalled) {
        return ESP_OK;
    }
    esp_err_t ret = gpio_isr_handler_remove(m_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IRQ, "Error removing ISR handler for pin %d: %s", m_pin, esp_err_to_name(ret));
        return ret;
    }
    m_handlerInstalled = false;
    ESP_LOGI(TAG_IRQ, "ISR handler removed from pin %d", m_pin);
    return ESP_OK;
}
