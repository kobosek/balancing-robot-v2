#include "include/Encoder.hpp"
#include "esp_timer.h"
#include "freertos/queue.h"

void IRAM_ATTR Encoder::isrHandler(void *arg) {
    Encoder *self = static_cast<Encoder *>(arg);
    self->handleInterrupt();
}

void IRAM_ATTR Encoder::handleInterrupt() {
    // Read the current state of A and B
    uint64_t currentTime = esp_timer_get_time();
    uint64_t dt = currentTime - lastTime;
    lastTime = currentTime;

    uint8_t current_state = (gpio_get_level(m_pinB) << 1) | gpio_get_level(m_pinA);

    // Determine direction based on state transitions
    if ((last_state == 0b00 && current_state == 0b01) ||
        (last_state == 0b01 && current_state == 0b11) ||
        (last_state == 0b11 && current_state == 0b10) ||
        (last_state == 0b10 && current_state == 0b00)) {
        position++; // Forward
    } else if ((last_state == 0b00 && current_state == 0b10) ||
               (last_state == 0b10 && current_state == 0b11) ||
               (last_state == 0b11 && current_state == 0b01) ||
               (last_state == 0b01 && current_state == 0b00)) {
        position--; // Reverse
    }
    last_state = current_state; // Update last state


    xQueueSendFromISR(dtQueue, &dt, NULL);
};

void Encoder::task_wrapper(void *arg) {
    Encoder *self = static_cast<Encoder *>(arg);
    self->calculate_speed_task(arg);
}

void Encoder::calculate_speed_task(void *pvParameters) {
    uint64_t dt = 0;
    while(1) {
        if(xQueueReceive(dtQueue, &dt, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Time between interrupts: %.5f", dt * 0.000001f);
        }
    }
}

esp_err_t Encoder::init(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Initializing Encoder");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << m_pinA) | (1ULL << m_pinB),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins");
        return ret;
    }

    dtQueue = xQueueCreate(10, sizeof(uint64_t));

    xTaskCreate(task_wrapper, "encoder_task", 4096, this, 5, NULL);

    gpio_install_isr_service(0);
    
    ret = gpio_isr_handler_add(m_pinA, isrHandler, this);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup encoder A handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(m_pinB, isrHandler, this);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup encoder B handler");
        return ret;
    }

    return ESP_OK;
}

float Encoder::getSpeed(float dt)  {
        // Calculate raw speed
        int32_t deltaPos = position - lastPosition;
        lastPosition = position;
        // Calculate current speed (ticks per second)
        float currentSpeed = (float)deltaPos / dt;
        
        float alpha = 0.2f;
        currentSpeed = (alpha * currentSpeed) + ((1 - alpha) * last_speed);

        last_speed = currentSpeed;

        return currentSpeed / 28 * 360 / 100; //ticksPerSec 
};

esp_err_t Encoder::onConfigUpdate(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Updating Encoder configuration");
    // Add any configuration update logic here
    return ESP_OK;
}