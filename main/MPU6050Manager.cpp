#include "include/MPU6050Manager.hpp"
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define MPU6050_ADDR 0x68

const gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_7;
const gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_8;
const gpio_num_t INTERRUPT_PIN = GPIO_NUM_9;

void IRAM_ATTR MPU6050Manager::isrHandler(void *arg) {
    MPU6050Manager *self = static_cast<MPU6050Manager *>(arg);
    self->handleInterrupt();
}

void IRAM_ATTR MPU6050Manager::handleInterrupt() {
    data_ready_counter++;
    if(data_ready_counter >= 10) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(fifo_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void MPU6050Manager::fifo_task_wrapper(void *arg) {
    MPU6050Manager *self = static_cast<MPU6050Manager *>(arg);
    self->fifo_task();
}

void MPU6050Manager::fifo_task() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (_sensor.isDataReady() == ESP_OK) {
            data_ready_counter = 0;

            uint8_t count_buf[2];
            esp_err_t ret = _sensor.readFromFifo(count_buf, _fifo_buffer);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "FIFO read failed: %s", esp_err_to_name(ret));
                continue;
            }

            uint16_t fifo_count = (count_buf[0] << 8) | count_buf[1];
            ESP_LOGD(TAG, "FIFO count: %d", fifo_count);
            if (fifo_count > 0 && fifo_count <= FIFO_BUFFER_SIZE) {
                processFifoData(_fifo_buffer, fifo_count);
            }
        }

        if(_sensor.isFIFOOverflow() == ESP_OK) {
            ESP_LOGW(TAG, "FIFO overflow, resetting");
            _sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
        }
    }
}

void MPU6050Manager::processFifoData(uint8_t* data, uint16_t length) {
    uint16_t num_samples = length / 12;
    if (length % 12 != 0) {
        ESP_LOGW(TAG, "Invalid FIFO data length: %d", length);
        return;
    }

    if (xSemaphoreTake(_pitch_mutex, portMAX_DELAY) != pdTRUE) return;

    for (int i = 0; i < num_samples; ++i) {
        int offset = i * 12;

        // Parse accelerometer data
        int16_t ax_raw = (data[offset] << 8) | data[offset + 1];
        int16_t ay_raw = (data[offset + 2] << 8) | data[offset + 3];
        int16_t az_raw = (data[offset + 4] << 8) | data[offset + 5];

        // Parse gyroscope Y-axis data
        int16_t gy_raw = (data[offset + 8] << 8) | data[offset + 9];

        // Convert to floats
        float ax = ax_raw / _sensor.getAccelScale();
        float ay = ay_raw / _sensor.getAccelScale();
        float az = az_raw / _sensor.getAccelScale();
        float gy = (gy_raw / _sensor.getGyroScale()) - _gyro_error;

        // Calculate accelerometer pitch
        float angleY_accel = std::atan2(-ax, std::sqrt(ay*ay + az*az)) * 180.0f / M_PI;

        // Time delta (1ms for 1kHz sample rate)
        const float delta_time = 0.001f;

        // Integrate gyro and apply filter
        _pitch_fifo = ALPHA * (_pitch_fifo + gy * delta_time) + (1 - ALPHA) * angleY_accel;
    }

    xSemaphoreGive(_pitch_mutex);
}

esp_err_t MPU6050Manager::init(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Initializing MPU6050Manager");

    esp_err_t ret = _sensor.init(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, MPU6050_ADDR, I2C_MASTER_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _sensor.setAccelRange(MPU6050AccelConfig::RANGE_4G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _sensor.setGyroRange(MPU6050GyroConfig::RANGE_500_DEG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _sensor.setDLPFConfig(MPU6050DLPFConfig::DLPF_BW_44HZ_ACC_42HZ_GYRO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DLPF config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _sensor.setSampleRate(MPU6050SampleRateDiv::RATE_1KHZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _sensor.setupInterrupt(MPU6050InterruptPinConfig::ACTIVE_HIGH, MPU6050Interrupt::DATA_READY_FIFO_OVERFLOW); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup interrupt: %s", esp_err_to_name(ret));
        return ret;
    }

    _pitch_mutex = xSemaphoreCreateMutex();
    if (!_pitch_mutex) {
        ESP_LOGE(TAG, "Failed to create pitch mutex");
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(
        fifo_task_wrapper,           // Task function
        "fifo_task",         // Task name
        4096,                   // Stack size
        this,              // Task parameters
        5, // Priority
        &fifo_task_handle,                   // Task handle
        1                      // Core ID (1)
    );

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // Trigger on rising edge
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, isrHandler, this);

    ESP_LOGI(TAG, "GPIO configured for MPU6050 INT pin");

    ret = _sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup FIFO: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = calibrateGyro();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050Manager initialized successfully");
    return ESP_OK;
}

float MPU6050Manager::calculatePitch(float& pitch, float dt) const {
    float acceleration_x, acceleration_y, acceleration_z;
    float omega_x, omega_y, omega_z;

    if (_sensor.getAcceleration(acceleration_x, acceleration_y, acceleration_z) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration");
        return pitch; 
    }

    if (_sensor.getRotation(omega_x, omega_y, omega_z) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read rotation");
        return pitch; 
    }

    float angleY_accel = std::atan2(-acceleration_x, std::sqrt(acceleration_y*acceleration_y + acceleration_z*acceleration_z)) * 180.0f / M_PI;
    omega_y -= _gyro_error;

    pitch = ALPHA * (pitch + omega_y *dt) + (1 - ALPHA) * angleY_accel;

    ESP_LOGV(TAG, "Calculated pitch: %.2f", pitch);
    return pitch;
}

float MPU6050Manager::calculateFifoPitch(float& pitch) const {
    if (xSemaphoreTake(_pitch_mutex, portMAX_DELAY) == pdTRUE) {
        pitch = _pitch_fifo;
        xSemaphoreGive(_pitch_mutex);
    }
    return pitch;
}

esp_err_t MPU6050Manager::calibrateGyro() {
    ESP_LOGI(TAG, "Calibrating gyroscope...");
    float omega_x, omega_y, omega_z;
    _gyro_error = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (_sensor.getRotation(omega_x, omega_y, omega_z) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope during calibration");
            return ESP_FAIL;
        }
        _gyro_error += omega_y;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    _gyro_error /= CALIBRATION_SAMPLES;
    ESP_LOGI(TAG, "Calibration complete. Gyro error: %.2f", _gyro_error);
    return ESP_OK;
}

esp_err_t MPU6050Manager::onConfigUpdate(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Updating MPU6050Manager configuration");
    
    // Here you can add any configuration updates specific to the MPU6050
    // For example, if you want to change the accelerometer or gyroscope range based on config:
    
    // esp_err_t ret = _sensor.setAccelRange(static_cast<MPU6050AccelConfig>(config.getMpu6050AccelRange()));
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to update accelerometer range: %s", esp_err_to_name(ret));
    //     return ret;
    // }
    
    // ret = _sensor.setGyroRange(static_cast<MPU6050GyroConfig>(config.getMpu6050GyroRange()));
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to update gyroscope range: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    // You might also want to recalibrate the gyroscope after a configuration change
    // return calibrateGyro();

    return ESP_OK;
}