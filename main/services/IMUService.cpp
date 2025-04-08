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
#include "OrientationDataEvent.hpp" // Include event to publish

// Include necessary event headers
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "StartCalibrationRequestEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "EventTypes.hpp"
#include "IMU_CommunicationErrorEvent.hpp" // <<< ADDED

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

        // Perform initial calibration
        ret = performCalibration(); // Perform initial calibration
        CalibrationCompleteEvent initial_calib_event(ret);
        m_eventBus.publish(initial_calib_event); // Publish result of initial calibration
        ESP_RETURN_ON_ERROR(ret, TAG, "Initial Gyro calibration failed"); // Halt init if initial calib fails
        
    // Interrupt setup...
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << m_config.int_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Use pull-up on INT pin
    io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    ret = gpio_config(&io_conf); ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);

    // Install ISR service only once globally if needed, or manage contextually
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
         ret = gpio_install_isr_service(0); // ESP_INTR_FLAG_IRAM or similar flags might be needed depending on exact timing requirements
         if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // Allow already installed state
             ESP_RETURN_ON_ERROR(ret, TAG, "Failed install ISR service");
         }
         isr_service_installed = (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE);
    }

    // Add ISR Handler for the specific pin
    ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR handler already added for pin %d.", m_config.int_pin);
        // If already added, maybe it's okay? Or should we remove first? Depends on application structure.
        // For now, let's treat it as non-fatal if it was already there.
        ret = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed add ISR handler for pin %d", m_config.int_pin);
    ESP_LOGI(TAG, "GPIO interrupt configured for INT pin %d", m_config.int_pin);

    // Configure MPU6050 Interrupts (DATA_READY) and FIFO (ACCEL+GYRO, Reset then Enable)
    MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
    ESP_RETURN_ON_ERROR(m_sensor.setupInterrupt(intPinConfig, MPU6050Interrupt::DATA_READY), TAG, "Failed setup MPU Interrupt");
    ESP_RETURN_ON_ERROR(m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL), TAG, "Failed setup MPU FIFO");

    // Create FIFO Task...
    BaseType_t taskCreated = xTaskCreatePinnedToCore( fifo_task_wrapper, "imu_fifo_task", 4096, this, configMAX_PRIORITIES - 2, &m_fifo_task_handle, 0); // Pinned to Core 0
    ESP_RETURN_ON_FALSE(taskCreated == pdPASS, ESP_FAIL, TAG, "Failed create IMU FIFO task");
    ESP_LOGI(TAG, "IMU FIFO Task created on Core 0.");

    // Subscribe to Calibration Request
    m_eventBus.subscribe(EventType::START_CALIBRATION_REQUEST, [this](const BaseEvent& ev) {
        ESP_LOGI(TAG, "Received StartCalibrationRequestEvent via lambda.");
        this->triggerCalibration(); // Call trigger method from lambda
    });
    ESP_LOGI(TAG, "Subscribed to START_CALIBRATION_REQUEST events.");

    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

// triggerCalibration remains the same
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

// handleStartCalibrationRequest remains the same (but unused by subscription lambda)
void IMUService::handleStartCalibrationRequest(const BaseEvent& event) {
     ESP_LOGW(TAG, "Handling StartCalibrationRequestEvent via method (Should be handled by lambda).");
     // Could optionally call triggerCalibration() here if lambda wasn't used
}


// performCalibration remains the same
esp_err_t IMUService::performCalibration() {
    if (m_is_calibrating) {
         ESP_LOGW(TAG, "Calibration already in progress.");
         return ESP_ERR_INVALID_STATE;
    }
    m_is_calibrating = true; // Set flag early
    ESP_LOGI(TAG, "Starting Gyro calibration (%d samples)...", m_config.calibration_samples);

    float old_offset_x = m_gyro_offset_radps[0];
    float old_offset_y = m_gyro_offset_radps[1];
    float old_offset_z = m_gyro_offset_radps[2];

    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int successful_samples = 0;
    int attempts = 0;
    const int max_attempts = m_config.calibration_samples + 200; // Allow more attempts
    std::vector<float> gy_samples; // Used for standard deviation calculation
    gy_samples.reserve(m_config.calibration_samples);

    ESP_LOGI(TAG, "Collecting calibration samples...");
    while (successful_samples < m_config.calibration_samples && attempts < max_attempts) {
        attempts++;
        float ax, ay, az, gx_dps, gy_dps, gz_dps;
        // Use direct reads instead of FIFO during calibration for simplicity
        esp_err_t ret_a = m_sensor.getAcceleration(ax, ay, az);
        esp_err_t ret_g = m_sensor.getRotation(gx_dps, gy_dps, gz_dps);

        if (ret_a != ESP_OK || ret_g != ESP_OK) {
            ESP_LOGE(TAG, "Read fail during calib (attempt %d)", attempts);
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait longer on error
            continue;
        }

        ESP_LOGV(TAG, "Calib Sample %d: gy=%.4f", successful_samples + 1, gy_dps);
        gx_sum += gx_dps;
        gy_sum += gy_dps;
        gz_sum += gz_dps;
        gy_samples.push_back(gy_dps); // Store Y samples for stdev
        successful_samples++;

        // Log progress less frequently
        if ((successful_samples % (m_config.calibration_samples / 5) == 0) && successful_samples > 0) {
             ESP_LOGI(TAG, "Calib progress: %d/%d", successful_samples, m_config.calibration_samples);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay between samples
    }

    esp_err_t final_status = ESP_OK;

    if (successful_samples < m_config.calibration_samples / 2) {
        ESP_LOGE(TAG, "Insufficient calibration samples obtained (%d/%d). Calibration failed, restoring previous offsets.", successful_samples, m_config.calibration_samples);
        m_gyro_offset_radps[0] = old_offset_x;
        m_gyro_offset_radps[1] = old_offset_y;
        m_gyro_offset_radps[2] = old_offset_z;
        final_status = ESP_FAIL;
    } else {
        float offset_gx_dps = gx_sum / successful_samples;
        float offset_gy_dps = gy_sum / successful_samples;
        float offset_gz_dps = gz_sum / successful_samples;

        // Update member offsets (in rad/s) ONLY on success
        m_gyro_offset_radps[0] = offset_gx_dps * (M_PI / 180.0f);
        m_gyro_offset_radps[1] = offset_gy_dps * (M_PI / 180.0f);
        m_gyro_offset_radps[2] = offset_gz_dps * (M_PI / 180.0f);

        // Calculate standard deviation for Y axis
        double sq_sum = std::inner_product(gy_samples.begin(), gy_samples.end(), gy_samples.begin(), 0.0);
        double stdev = std::sqrt(fabs(sq_sum / successful_samples - offset_gy_dps * offset_gy_dps));

        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). New Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Y Stdev: %.4f",
                 successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev);

        // Check if offsets or standard deviation are excessively large
        if (std::abs(offset_gy_dps) > 2.0 || stdev > 1.0) { // Adjusted thresholds
             ESP_LOGW(TAG, "Warning: Gyro Y offset or standard deviation seems high. Verify stillness during calibration.");
             // Decide if this should be treated as a failure or just a warning
             // final_status = ESP_FAIL; // Uncomment to make high offset/stdev a failure
        }
        // If not overridden by warning checks, mark as success
        if (final_status != ESP_FAIL) {
            final_status = ESP_OK;
        }
    }

    m_is_calibrating = false; // Clear flag after success or failure
    ESP_LOGI(TAG, "Calibration process finished with status: %s", esp_err_to_name(final_status));
    return final_status;
}

// Static ISR handler (Definition requires IRAM_ATTR)
// Ensure this function is placed in IRAM using the linker script or appropriate attributes if needed
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    static_cast<IMUService *>(arg)->handleInterrupt();
}

// Member ISR handler (Definition requires IRAM_ATTR)
// Ensure this function is placed in IRAM
void IRAM_ATTR IMUService::handleInterrupt() {
     // Very minimal logic in ISR: increment counter and notify task if threshold reached
     if(m_is_calibrating) return; // Ignore interrupts during calibration

    uint8_t current_count = m_isr_data_counter.fetch_add(1) + 1; // Get count *after* increment

    if (current_count == m_config.fifo_read_threshold) {
        // Attempt to reset counter back to 0 atomically
        uint8_t expected = m_config.fifo_read_threshold;
        m_isr_data_counter.compare_exchange_strong(expected, 0); // Reset if it's still the threshold value

        // Notify the task
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(m_fifo_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    } else if (current_count > m_config.fifo_read_threshold) {
        // Safety net: If counter somehow went past threshold, reset it and notify anyway
         m_isr_data_counter.store(0);
         // Log this rare event outside ISR if needed for debugging
         BaseType_t woken = pdFALSE;
         vTaskNotifyGiveFromISR(m_fifo_task_handle, &woken);
         portYIELD_FROM_ISR(woken);
    }
     // Do nothing if counter < threshold
}


void IMUService::fifo_task_wrapper(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if(s) s->fifo_task();
    else {
        ESP_LOGE(TAG, "fifo_task_wrapper received null argument!");
        vTaskDelete(NULL); // Clean up the task itself
    }
}

// *** MODIFIED FIFO TASK ***
void IMUService::fifo_task() {
    ESP_LOGI(TAG, "IMU FIFO Task started (Robust FIFO) on Core %d.", xPortGetCoreID());
    uint8_t fifo_buffer[1024]; // Buffer to hold raw FIFO data
    const int bytes_per_sample = 12; // Accel (6) + Gyro (6)

    // Get sensor scales once (assuming they don't change)
    float accel_scale = m_sensor.getAccelScale();
    float gyro_scale = m_sensor.getGyroScale();

    while (1) {
        // Wait indefinitely for notification from ISR
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
             // Check calibration status (read volatile bool)
             bool currently_calibrating = m_is_calibrating;
             // Get current offset atomically (read volatile float array - mutex might be better)
             // For simplicity, assume direct read is okay for now if updates are infrequent
             float current_offset_y_radps = m_gyro_offset_radps[1];

             ESP_LOGD(TAG, "FIFO task notified. Calibrating flag: %s, Current Y Offset (rad/s): %.4f", currently_calibrating ? "true" : "false", current_offset_y_radps);

             if (currently_calibrating) {
                 // If calibrating, drain the FIFO to prevent overflow but don't process
                 uint8_t count_buf_drain[2]; uint16_t fifo_count_drain = 0;
                 if(m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf_drain, 2) == ESP_OK) {
                     fifo_count_drain = (count_buf_drain[0] << 8) | count_buf_drain[1];
                     if (fifo_count_drain >= 1024) { // Overflow during drain? Reset.
                          ESP_LOGW(TAG, "FIFO overflow during calibration drain check. Resetting.");
                          m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                          m_isr_data_counter.store(0); // Reset counter after reset
                     } else if (fifo_count_drain > 0) {
                          uint16_t bytes_to_drain = std::min((uint16_t)sizeof(fifo_buffer), fifo_count_drain);
                          ESP_LOGD(TAG, "Draining %d bytes during calibration state.", bytes_to_drain);
                          m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_drain); // Discard data
                     }
                 }
                 continue; // Skip normal processing
             }

            // --- Read FIFO Count ---
            uint8_t count_buf[2];
            uint16_t fifo_count = 0;
            esp_err_t ret = m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed read FIFO count: %s. Skipping cycle.", esp_err_to_name(ret));
                m_consecutive_i2c_failures++; // <<< ADDED: Increment failure count
                if (m_consecutive_i2c_failures >= I2C_FAILURE_THRESHOLD) {
                    ESP_LOGE(TAG, "IMU communication failure threshold reached (%d consecutive errors). Publishing event.", m_consecutive_i2c_failures);
                    IMU_CommunicationErrorEvent error_event;
                    m_eventBus.publish(error_event);
                    m_consecutive_i2c_failures = 0; // Reset counter after publishing
                }
                // ---> MODIFICATION: Do NOT reset FIFO here immediately <---
                // m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                continue; // Try again next notification
            } else {
                 m_consecutive_i2c_failures = 0; // <<< ADDED: Reset failure count on success
            }
            fifo_count = (count_buf[0] << 8) | count_buf[1];

            // --- Handle Hardware Overflow (Count >= 1024) ---
            if (fifo_count >= 1024) {
                ESP_LOGW(TAG, "FIFO overflow detected by count! Count=%d. Resetting.", fifo_count);
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                m_isr_data_counter.store(0); // Reset ISR counter after FIFO reset
                continue;
            }

            // --- Check Overflow Flag (Secondary Check) ---
            if (m_sensor.isFIFOOverflow() == ESP_OK) {
                 ESP_LOGW(TAG, "FIFO overflow flag detected by status register! Resetting.");
                 m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                 m_isr_data_counter.store(0); // Reset ISR counter after FIFO reset
                 continue;
            }


            // --- Read Data if Available ---
            if (fifo_count > 0) {
                // Determine actual bytes to read (limited by buffer size)
                uint16_t bytes_to_read = std::min((uint16_t)sizeof(fifo_buffer), fifo_count);

                // Log if we have to truncate due to buffer size
                if (bytes_to_read < fifo_count) {
                     ESP_LOGW(TAG,"FIFO count %d > buffer %zu, reading only %d bytes.", fifo_count, sizeof(fifo_buffer), bytes_to_read);
                }

                // --- Read FIFO Data ---
                ret = m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_read);
                if (ret != ESP_OK) {
                     ESP_LOGE(TAG, "FIFO read data failed: %s. Skipping processing for this batch.", esp_err_to_name(ret));
                     m_consecutive_i2c_failures++; // <<< ADDED: Increment failure count
                     if (m_consecutive_i2c_failures >= I2C_FAILURE_THRESHOLD) {
                         ESP_LOGE(TAG, "IMU communication failure threshold reached (%d consecutive errors). Publishing event.", m_consecutive_i2c_failures);
                         IMU_CommunicationErrorEvent error_event;
                         m_eventBus.publish(error_event);
                         m_consecutive_i2c_failures = 0; // Reset counter after publishing
                     }
                     // ---> MODIFICATION: Do NOT reset FIFO here immediately <---
                     // m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                     continue; // Skip processing, hope it recovers next time
                } else {
                     m_consecutive_i2c_failures = 0; // <<< ADDED: Reset failure count on success
                }

                // ---> ADDED: Reset ISR counter only AFTER successful read and before processing <---
                m_isr_data_counter.store(0);

                // --- Process Data ---
                // Pass the actual bytes read to the estimator
                ESP_LOGV(TAG, "Read %d bytes from FIFO (reported count: %d), passing to estimator.", bytes_to_read, fifo_count);
                float offset_y_dps = current_offset_y_radps * (180.0f / M_PI);
                m_estimator.processFifoBatch(fifo_buffer, bytes_to_read, accel_scale, gyro_scale, offset_y_dps); // Pass bytes_to_read

                // --- Publish Orientation Event ---
                float pitch_deg = m_estimator.getPitchDeg();
                float pitch_rate_radps = 0.0f; // m_estimator.getPitchRateRadPerSec();

                OrientationDataEvent orientation_event(
                    pitch_deg * (M_PI / 180.0f), // Convert to Rad
                    pitch_rate_radps
                );
                m_eventBus.publish(orientation_event);

            } // end if (fifo_count > 0)
            else {
                 ESP_LOGV(TAG, "FIFO count is 0. Waiting for more data.");
            }
        } // end if (ulTaskNotifyTake)
    } // end while(1)
}
// *** END MODIFIED FIFO TASK ***


// --- Recovery Methods ---

esp_err_t IMUService::resetSensor() {
    ESP_LOGI(TAG, "Attempting IMU sensor reset...");
    // TODO: Implement actual reset logic using m_sensor driver
    // Option 1: Software Reset (if available)
    // esp_err_t ret = m_sensor.reset();
    // Option 2: Power Management Reset (MPU6050 specific)
    esp_err_t ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x80); // Set RESET bit
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete
        // Clear sleep bit potentially set by reset
        ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x00);
    }
    ESP_LOGI(TAG, "Sensor reset attempt finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::reinitializeSensor() {
    ESP_LOGI(TAG, "Reinitializing IMU sensor configuration...");
    esp_err_t ret = ESP_OK;

    // Re-apply essential configurations (similar to init, but without full I2C setup)
    // Consider error handling for each step - maybe return on first error?
    MPU6050AccelConfig accelRange = static_cast<MPU6050AccelConfig>(m_config.accel_range);
    MPU6050GyroConfig gyroRange = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);

    ret = m_sensor.setAccelRange(accelRange);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit Accel Range: %s", esp_err_to_name(ret)); return ret; }

    ret = m_sensor.setGyroRange(gyroRange);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit Gyro Range: %s", esp_err_to_name(ret)); return ret; }

    ret = m_sensor.setDLPFConfig(dlpf);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit DLPF: %s", esp_err_to_name(ret)); return ret; }

    ret = m_sensor.setSampleRate(rateDiv);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit Sample Rate: %s", esp_err_to_name(ret)); return ret; }

    // Re-setup Interrupts and FIFO
    MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
    ret = m_sensor.setupInterrupt(intPinConfig, MPU6050Interrupt::DATA_READY);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit MPU Interrupt: %s", esp_err_to_name(ret)); return ret; }

    ret = m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed reinit MPU FIFO: %s", esp_err_to_name(ret)); return ret; }

    // Reset ISR counter state
    m_isr_data_counter.store(0);
    m_consecutive_i2c_failures = 0; // Also reset failure count here

    ESP_LOGI(TAG, "Sensor reinitialization finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::verifyCommunication() {
    ESP_LOGI(TAG, "Verifying IMU communication...");
    // Attempt a simple read, e.g., WHO_AM_I register
    uint8_t who_am_i = 0;
    esp_err_t ret = m_sensor.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i, 8);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Communication verified successfully (WHO_AM_I=0x%02X).", who_am_i);
        // Optionally check if who_am_i matches expected value (e.g., 0x68 for MPU6050)
        if (who_am_i != 0x68) { // Example value, adjust if needed
             ESP_LOGW(TAG, "WHO_AM_I value (0x%02X) does not match expected (0x68).", who_am_i);
             // Decide if this is a failure or just a warning
             // ret = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Communication verification failed: %s", esp_err_to_name(ret));
    }
    return ret;
}