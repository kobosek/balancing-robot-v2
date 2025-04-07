// main/services/IMUService.cpp
#include "IMUService.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <algorithm> // Include for std::min
#include "freertos/FreeRTOS.h" // Include necessary FreeRTOS headers
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mpu6050.hpp" // Include for MPU6050 class and enums
#include "OrientationEstimator.hpp" // Include for m_estimator usage
#include "OrientationDataEvent.hpp" // <<< ADDED

// Include necessary event headers
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "StartCalibrationRequestEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "EventTypes.hpp"

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float G_ACCEL = 9.80665f;

// Constructor with corrected initializer list order
IMUService::IMUService(const MPU6050Config& config, EventBus& bus, OrientationEstimator& estimator) :
    m_config(config),                   // 1st in header
    m_eventBus(bus),                    // 2nd
    m_estimator(estimator),             // 3rd
    m_sensor(),                         // 4th (Default constructed)
    m_fifo_task_handle(nullptr),        // 5th
    m_calibration_mutex(nullptr),       // 6th
    m_is_calibrating(false),            // 7th
    m_gyro_offset_radps{0.0f, 0.0f, 0.0f},// 8th (Initialize array)
    m_isr_data_counter(0)               // 9th
{
    // Constructor body remains empty
}


IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_fifo_task_handle) { vTaskDelete(m_fifo_task_handle); m_fifo_task_handle = nullptr; }
    if (m_calibration_mutex) { vSemaphoreDelete(m_calibration_mutex); m_calibration_mutex = nullptr; }
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
         gpio_isr_handler_remove(m_config.int_pin);
    }
    ESP_LOGI(TAG, "IMUService deconstructed.");
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    m_calibration_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(m_calibration_mutex != NULL, ESP_FAIL, TAG, "Failed create calibration mutex");

    esp_err_t ret = m_sensor.init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init MPU6050 driver");

    // Sensor configuration...
    MPU6050AccelConfig accelRange = static_cast<MPU6050AccelConfig>(m_config.accel_range);
    MPU6050GyroConfig gyroRange = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);
    ESP_RETURN_ON_ERROR(m_sensor.setAccelRange(accelRange), TAG, "Failed set Accel Range");
    ESP_RETURN_ON_ERROR(m_sensor.setGyroRange(gyroRange), TAG, "Failed set Gyro Range");
    ESP_RETURN_ON_ERROR(m_sensor.setDLPFConfig(dlpf), TAG, "Failed set DLPF");
    ESP_RETURN_ON_ERROR(m_sensor.setSampleRate(rateDiv), TAG, "Failed set Sample Rate");

    // Interrupt setup...
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << m_config.int_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    ret = gpio_config(&io_conf); ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
         ret = gpio_install_isr_service(0);
         if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
             ESP_RETURN_ON_ERROR(ret, TAG, "Failed install ISR service");
         }
         isr_service_installed = (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE);
    }
    ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR handler already added for pin %d.", m_config.int_pin);
        ret = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed add ISR handler"); ESP_LOGI(TAG, "GPIO interrupt configured for INT pin %d", m_config.int_pin);
    MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
    ESP_RETURN_ON_ERROR(m_sensor.setupInterrupt(intPinConfig, MPU6050Interrupt::DATA_READY), TAG, "Failed setup MPU Interrupt");
    ESP_RETURN_ON_ERROR(m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL), TAG, "Failed setup MPU FIFO");

    // Create FIFO Task...
    BaseType_t taskCreated = xTaskCreatePinnedToCore( fifo_task_wrapper, "imu_fifo_task", 4096, this, configMAX_PRIORITIES - 2, &m_fifo_task_handle, 1);
    ESP_RETURN_ON_FALSE(taskCreated == pdPASS, ESP_FAIL, TAG, "Failed create IMU FIFO task");
    ESP_LOGI(TAG, "IMU FIFO Task created.");

    // Subscribe to Calibration Request
    m_eventBus.subscribe(EventType::START_CALIBRATION_REQUEST, [this](const BaseEvent& ev) {
        ESP_LOGI(TAG, "Received StartCalibrationRequestEvent via lambda.");
        this->triggerCalibration();
    });
    ESP_LOGI(TAG, "Subscribed to START_CALIBRATION_REQUEST events.");

    // Perform initial calibration
    ret = performCalibration(); // Perform initial calibration
    CalibrationCompleteEvent initial_calib_event(ret);
    m_eventBus.publish(initial_calib_event);
    ESP_RETURN_ON_ERROR(ret, TAG, "Initial Gyro calibration failed");

    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

esp_err_t IMUService::triggerCalibration() {
    ESP_LOGI(TAG, "Calibration triggered.");
    if (xSemaphoreTake(m_calibration_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Calibration busy, request ignored.");
        CalibrationCompleteEvent complete_event(ESP_ERR_TIMEOUT);
        m_eventBus.publish(complete_event);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = performCalibration();

    CalibrationCompleteEvent complete_event(ret);
    m_eventBus.publish(complete_event);
    ESP_LOGI(TAG,"Published CalibrationCompleteEvent (Status: %s)", esp_err_to_name(ret));

    xSemaphoreGive(m_calibration_mutex);
    return ret;
}

void IMUService::handleStartCalibrationRequest(const BaseEvent& event) {
     // This function remains but is no longer called directly by the event subscription lambda
    ESP_LOGW(TAG, "Handling StartCalibrationRequestEvent via method (Should be handled by lambda).");
}

esp_err_t IMUService::performCalibration() {
    if (m_is_calibrating) {
         ESP_LOGW(TAG, "Calibration already in progress.");
         return ESP_ERR_INVALID_STATE;
    }
    m_is_calibrating = true; // Set flag early
    ESP_LOGI(TAG, "Starting Gyro calibration (%d samples)...", m_config.calibration_samples);

    // Drain FIFO
    uint16_t fifo_count = 0;
    uint8_t dummy_buf[128];
    uint8_t count_buf[2];
    int drain_attempts = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(20));
        if(m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2) == ESP_OK) {
            fifo_count = (count_buf[0] << 8) | count_buf[1];
            if (fifo_count >= 1024) {
                ESP_LOGW(TAG, "FIFO overflow during pre-calibration drain. Resetting FIFO.");
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                fifo_count = 0;
            } else if (fifo_count > 0) {
                uint16_t bytes_to_read = std::min(fifo_count, (uint16_t)sizeof(dummy_buf));
                ESP_LOGD(TAG, "Draining %d bytes from FIFO before calibration...", bytes_to_read);
                m_sensor.readRegisters(MPU6050Register::FIFO_R_W, dummy_buf, bytes_to_read);
            }
        } else {
             ESP_LOGE(TAG, "Failed to read FIFO count during pre-calibration drain.");
             m_is_calibrating = false; // Clear flag on error
             return ESP_FAIL;
        }
        drain_attempts++;
    } while (fifo_count > 0 && drain_attempts < 10); // Limit drain attempts
    if (fifo_count > 0) {
        ESP_LOGW(TAG, "FIFO not fully drained after %d attempts.", drain_attempts);
        // Continue anyway, but log warning
    } else {
        ESP_LOGI(TAG, "FIFO drained before calibration.");
    }

    // --- Store current offsets before calculating new ones ---
    float old_offset_x = m_gyro_offset_radps[0];
    float old_offset_y = m_gyro_offset_radps[1];
    float old_offset_z = m_gyro_offset_radps[2];
    // --- End store ---

    // Temporarily zero out for calculation (or use local vars for sums)
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int successful_samples = 0;
    int attempts = 0;
    const int max_attempts = m_config.calibration_samples + 100;
    std::vector<float> gy_samples;
    gy_samples.reserve(m_config.calibration_samples);

    ESP_LOGI(TAG, "Collecting calibration samples...");
    while (successful_samples < m_config.calibration_samples && attempts < max_attempts) {
        // ... (Sample collection loop remains the same) ...
        attempts++;
        float ax, ay, az, gx_dps, gy_dps, gz_dps;
        esp_err_t ret_a = m_sensor.getAcceleration(ax, ay, az);
        esp_err_t ret_g = m_sensor.getRotation(gx_dps, gy_dps, gz_dps);

        if (ret_a != ESP_OK || ret_g != ESP_OK) {
            ESP_LOGE(TAG, "Read fail during calib (attempt %d)", attempts);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGV(TAG, "Calib Sample %d: gy=%.4f", successful_samples + 1, gy_dps);
        gx_sum += gx_dps;
        gy_sum += gy_dps;
        gz_sum += gz_dps;
        gy_samples.push_back(gy_dps);
        successful_samples++;

        if ((successful_samples) % (m_config.calibration_samples / 10) == 0 && successful_samples > 0) {
             ESP_LOGI(TAG, "Calib progress: %d/%d", successful_samples, m_config.calibration_samples);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    esp_err_t final_status = ESP_FAIL;

    if (successful_samples < m_config.calibration_samples / 2) {
        ESP_LOGE(TAG, "Insufficient calibration samples obtained (%d/%d). Calibration failed, restoring previous offsets.", successful_samples, m_config.calibration_samples);
        // --- Restore old offsets on failure ---
        m_gyro_offset_radps[0] = old_offset_x;
        m_gyro_offset_radps[1] = old_offset_y;
        m_gyro_offset_radps[2] = old_offset_z;
        // --- End restore ---
    } else {
        float offset_gx_dps = gx_sum / successful_samples;
        float offset_gy_dps = gy_sum / successful_samples;
        float offset_gz_dps = gz_sum / successful_samples;

        // --- Update member offsets ONLY on success ---
        m_gyro_offset_radps[0] = offset_gx_dps * (M_PI / 180.0f);
        m_gyro_offset_radps[1] = offset_gy_dps * (M_PI / 180.0f);
        m_gyro_offset_radps[2] = offset_gz_dps * (M_PI / 180.0f);
        // --- End update ---

        double sq_sum = std::inner_product(gy_samples.begin(), gy_samples.end(), gy_samples.begin(), 0.0);
        double stdev = std::sqrt(fabs(sq_sum / successful_samples - offset_gy_dps * offset_gy_dps));

        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). New Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Y Stdev: %.4f",
                 successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev);

        if (std::abs(offset_gy_dps) > 1.0 || stdev > 0.5) {
             ESP_LOGW(TAG, "Warning: Gyro Y offset or standard deviation seems high. Verify stillness during calibration.");
        }
        final_status = ESP_OK; // Mark as successful
    }

    m_is_calibrating = false; // Clear flag after success or failure
    ESP_LOGI(TAG, "Calibration process finished.");
    return final_status;
}


// Static ISR handler (Definition requires IRAM_ATTR)
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    static_cast<IMUService *>(arg)->handleInterrupt();
}

// Member ISR handler (Definition requires IRAM_ATTR)
void IRAM_ATTR IMUService::handleInterrupt() {
     if(m_is_calibrating) return; // Ignore interrupts during calibration

    uint8_t previous_count = m_isr_data_counter.fetch_add(1);
    if ((previous_count + 1) == m_config.fifo_read_threshold) {
        uint8_t expected = m_config.fifo_read_threshold;
        m_isr_data_counter.compare_exchange_strong(expected, 0);
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(m_fifo_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    } else if ((previous_count + 1) > m_config.fifo_read_threshold) {
         m_isr_data_counter.store(0);
         // Avoid logging from ISR if possible
         // ESP_EARLY_LOGW(TAG, "ISR counter exceeded threshold (%d), resetting.", previous_count + 1);
         BaseType_t woken = pdFALSE;
         vTaskNotifyGiveFromISR(m_fifo_task_handle, &woken);
         portYIELD_FROM_ISR(woken);
    }
}


void IMUService::fifo_task_wrapper(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if(s) s->fifo_task();
    else {
        ESP_LOGE(TAG, "fifo_task_wrapper received null argument!");
        vTaskDelete(NULL);
    }
}

void IMUService::fifo_task() {
    ESP_LOGI(TAG, "IMU FIFO Task started (Batch Processing) on Core %d.", xPortGetCoreID());
    uint8_t fifo_buffer[1024];
    const int bytes_per_sample = 12;

    float accel_scale = m_sensor.getAccelScale();
    float gyro_scale = m_sensor.getGyroScale();

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
             bool currently_calibrating = m_is_calibrating;
             float current_offset_y_radps = m_gyro_offset_radps[1];
             ESP_LOGD(TAG, "FIFO task notified. Calibrating flag: %s, Current Y Offset (rad/s): %.4f", currently_calibrating ? "true" : "false", current_offset_y_radps);

             if (currently_calibrating) {
                 // Drain FIFO if calibrating (safer than just continuing)
                 uint8_t count_buf[2]; uint16_t fifo_count_drain = 0;
                 if(m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2) == ESP_OK) {
                     fifo_count_drain = (count_buf[0] << 8) | count_buf[1];
                     if (fifo_count_drain >= 1024) { // Overflow during drain? Reset.
                          ESP_LOGW(TAG, "FIFO overflow during calibration drain check. Resetting.");
                          m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                     } else if (fifo_count_drain > 0) {
                          uint16_t disc = std::min((uint16_t)sizeof(fifo_buffer), fifo_count_drain);
                          ESP_LOGD(TAG, "Draining %d bytes during calibration state.", disc);
                          m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, disc); // Discard data
                     }
                 }
                 continue; // Skip processing
             }

            // --- Read FIFO Count ---
            uint8_t count_buf[2]; uint16_t fifo_count = 0;
            esp_err_t ret = m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed read FIFO count: %s. Resetting FIFO.", esp_err_to_name(ret));
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); // <<< RESET FIFO
                continue;
            }
            fifo_count = (count_buf[0] << 8) | count_buf[1];

            // --- Handle Overflow ---
            if (fifo_count >= 1024) {
                ESP_LOGW(TAG, "FIFO overflow! Count=%d. Resetting.", fifo_count);
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                continue;
            }

            // --- Read Data if Available ---
            if (fifo_count >= bytes_per_sample) {
                uint16_t bytes_to_read = fifo_count - (fifo_count % bytes_per_sample);
                if(bytes_to_read == 0) continue;

                if (bytes_to_read > sizeof(fifo_buffer)) {
                    ESP_LOGW(TAG,"FIFO count %d > buffer %zu, reading max buffer size.", fifo_count, sizeof(fifo_buffer));
                    bytes_to_read = sizeof(fifo_buffer) - (sizeof(fifo_buffer) % bytes_per_sample);
                     if(bytes_to_read == 0) { continue; }
                }

                // --- Read FIFO Data ---
                ret = m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_read);
                if (ret != ESP_OK) {
                     ESP_LOGE(TAG, "FIFO read failed: %s. Resetting FIFO.", esp_err_to_name(ret));
                     m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); // <<< RESET FIFO
                     continue;
                }

                // --- Process Data ---
                ESP_LOGV(TAG, "Read %d bytes (%d samples) from FIFO, passing to estimator.", bytes_to_read, bytes_to_read / bytes_per_sample);
                float offset_y_dps = current_offset_y_radps * (180.0f / M_PI);
                m_estimator.processFifoBatch(fifo_buffer, bytes_to_read, accel_scale, gyro_scale, offset_y_dps);

                // --- Publish Orientation Event ---
                float pitch_deg = m_estimator.getPitchDeg();
                float roll_deg = m_estimator.getRollDeg();
                OrientationDataEvent orientation_event(
                    pitch_deg * OrientationEstimator::DEG_TO_RAD,
                    roll_deg * OrientationEstimator::DEG_TO_RAD,
                    0.0f, 0.0f // Rates currently zero
                );
                m_eventBus.publish(orientation_event);
            }
        }
    }
}