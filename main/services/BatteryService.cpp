// main/BatteryService.cpp
#include "BatteryService.hpp"           // Relative path within module's include dir
#include "EventTypes.hpp"               // Found via INCLUDE_DIRS
#include "BatteryStatusUpdatedEvent.hpp"// Found via INCLUDE_DIRS (needed for publish)
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS (needed for publish)
#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for BatteryConfig)
#include <numeric>
#include <vector>
#include <algorithm>
#include "esp_check.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"               // For ADC calibration scheme checks
#include "esp_adc/adc_oneshot.h"        // Keep specific ADC includes
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"          // Moved from header
#include "freertos/task.h"              // Moved from header
#include <mutex>                        // Moved from header


BatteryService::BatteryService(const BatteryConfig& config, EventBus& bus) :
    m_config(config),
    m_eventBus(bus),
    m_adc_channel(ADC_CHANNEL_0), // Initialize channel
    m_adc_cali_handle(nullptr),
    m_adc_cali_enable(false),
    m_monitor_task_handle(nullptr),
    m_oneshot_adc_handle(nullptr) // Initialize new handle
{
    m_latest_status.percentage = -1;
    m_latest_status.voltage = -1.0f; // Indicate invalid initial voltage
    m_latest_status.isLow = false;
}

BatteryService::~BatteryService() {
    stop(); // Ensure task/timer is stopped first

    // --- Correct ADC Calibration Deinitialization ---
    if (m_adc_cali_enable && m_adc_cali_handle) {
        ESP_LOGI(TAG, "Deleting ADC calibration scheme handle.");
        // Choose the correct delete function based on the scheme created
        #if SOC_ADC_CALIBRATION_V1_SUPPORTED // Check if V1 (Curve Fitting) is supported
             ESP_LOGD(TAG,"Using V1 Calibration Scheme (Curve Fitting) for deinit.");
             ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_curve_fitting(m_adc_cali_handle));
        #elif SOC_ADC_CALIBRATION_V2_SUPPORTED // Check if V2 (Line Fitting) is supported
             ESP_LOGD(TAG,"Using V2 Calibration Scheme (Line Fitting) for deinit.");
             ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_line_fitting(m_adc_cali_handle));
        #endif
        m_adc_cali_handle = nullptr;
        m_adc_cali_enable = false;
    } else {
         ESP_LOGI(TAG, "ADC calibration handle already null or disabled.");
    }

    // --- Deinitialize ADC Unit ---
    if (m_oneshot_adc_handle) {
         ESP_LOGI(TAG, "Deleting ADC oneshot handle.");
         ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_del_unit(m_oneshot_adc_handle));
         m_oneshot_adc_handle = nullptr;
    }
     ESP_LOGI(TAG, "BatteryService Deinitialized.");
}

// Helper to map GPIO and check if it's valid for ADC1
esp_err_t BatteryService::map_gpio_to_channel() {
     // adc_oneshot_io_to_channel returns the channel via output param
     adc_unit_t unit_id = ADC_UNIT_1;
     esp_err_t ret = adc_oneshot_io_to_channel(m_config.adc_pin, &unit_id, &m_adc_channel);
     if (ret != ESP_OK) {
          ESP_LOGE(TAG, "GPIO %d is not a valid ADC1 pin (%s)", (int)m_config.adc_pin, esp_err_to_name(ret));
          m_adc_channel = ADC_CHANNEL_0; // Ensure channel is marked invalid
     } else {
          ESP_LOGI(TAG, "Mapped GPIO %d to ADC1 Channel %d", (int)m_config.adc_pin, (int)m_adc_channel);
     }
     return ret;
}


esp_err_t BatteryService::init() {
    ESP_LOGI(TAG, "Initializing BatteryService...");
    esp_err_t ret;

    // 1. Map GPIO to Channel
    ret = map_gpio_to_channel();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to map GPIO to ADC channel");

    // 2. Init ADC Unit
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0) // Check if clk_src exists
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, // Explicitly set default clock source if available
#endif
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_config1, &m_oneshot_adc_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to init ADC1 oneshot unit");


    // 3. Configure ADC Channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN,    // Use static const member (adc_atten_t)
        .bitwidth = ADC_WIDTH, // Use static const member (adc_bitwidth_t)
    };
    ret = adc_oneshot_config_channel(m_oneshot_adc_handle, m_adc_channel, &channel_config); // Use mapped channel
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to config ADC channel %d for GPIO %d", (int)m_adc_channel, (int)m_config.adc_pin);


    // 4. Initialize ADC Calibration
    m_adc_cali_enable = adc_calibration_init(m_adc_channel); // Pass mapped channel
    if (!m_adc_cali_enable) {
        ESP_LOGW(TAG, "ADC calibration skipped or failed. Voltage readings may be inaccurate.");
    }

    ESP_LOGI(TAG, "BatteryService Initialized (ADC GPIO: %d -> Channel: %d).", (int)m_config.adc_pin, (int)m_adc_channel);
    return ESP_OK;
}

// Updated Calibration Init for Oneshot Driver - Takes channel
bool BatteryService::adc_calibration_init(adc_channel_t channel) {
    esp_err_t ret = ESP_FAIL;
    m_adc_cali_enable = false;

    if (channel == ADC_CHANNEL_0) {
        ESP_LOGE(TAG, "Cannot initialize calibration, invalid ADC channel.");
        return false;
    }

    ESP_LOGI(TAG, "Attempting ADC calibration for channel %d...", (int)channel);

    // Check which calibration scheme is supported and attempt creation
    // Note: ESP32-S3 generally supports Curve Fitting (V1)
#if SOC_ADC_CALIBRATION_V1_SUPPORTED // Preferred for ESP32-S3, ESP32-C3 etc.
    ESP_LOGI(TAG, "Using ADC_CALI_SCHEME_CURVE_FITTING");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel, // Pass mapped channel
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &m_adc_cali_handle);
    if (ret == ESP_OK) {
        m_adc_cali_enable = true;
    }
#else
    // Fallback or specific check for other chips if needed
    ESP_LOGW(TAG,"Curve Fitting V1 not supported by SOC_ADC_CALIBRATION. Check target.");
    ret = ESP_ERR_NOT_SUPPORTED;
    // #elif SOC_ADC_CALIBRATION_V2_SUPPORTED // Example for Line Fitting on older chips
    // ESP_LOGI(TAG, "Using ADC_CALI_SCHEME_LINE_FITTING");
    //  adc_cali_line_fitting_config_t cali_config = {
    //     .unit_id = ADC_UNIT_1,
    //     .atten = ADC_ATTEN,
    //     .bitwidth = ADC_WIDTH,
    // };
    // ret = adc_cali_create_scheme_line_fitting(&cali_config, &m_adc_cali_handle);
    //  if (ret == ESP_OK) {
    //     m_adc_cali_enable = true;
    // }
#endif

    // Handle results
    if (ret == ESP_OK && m_adc_cali_enable) {
        ESP_LOGI(TAG, "ADC Calibration scheme created successfully.");
    } else {
         if (ret == ESP_ERR_NOT_SUPPORTED) {
             ESP_LOGW(TAG,"ADC calibration scheme not supported for this configuration.");
         } else {
             ESP_LOGE(TAG,"Failed to create ADC calibration scheme: %s (%d)", esp_err_to_name(ret), ret);
         }
         m_adc_cali_handle = nullptr; // Ensure handle is null if creation failed
         m_adc_cali_enable = false;
    }
    return m_adc_cali_enable;
}


void BatteryService::start() {
     if (m_monitor_task_handle != nullptr) { ESP_LOGW(TAG, "Monitor task already started."); return; }
     ESP_LOGI(TAG, "Starting battery monitoring task...");
     // Create task with sufficient stack size
     xTaskCreate(monitorTaskWrapper, "BattMonTask", 3072, this, 5, &m_monitor_task_handle);
     if (m_monitor_task_handle == nullptr) { ESP_LOGE(TAG, "Failed to create battery monitor task!"); }
}

void BatteryService::stop() {
     if (m_monitor_task_handle == nullptr) { ESP_LOGD(TAG, "Monitor task not running."); return; }
     ESP_LOGI(TAG, "Stopping battery monitoring task...");
     vTaskDelete(m_monitor_task_handle);
     m_monitor_task_handle = nullptr;
}

BatteryStatus BatteryService::getLatestStatus() const {
     std::lock_guard<std::mutex> lock(m_status_mutex);
     return m_latest_status;
}


void BatteryService::monitorTaskWrapper(void* arg) {
    BatteryService* instance = static_cast<BatteryService*>(arg);
    if (instance) instance->monitorTask(); else { ESP_LOGE("BattServWrap", "Task wrapper null instance!"); vTaskDelete(NULL); }
}

void BatteryService::monitorTask() {
    ESP_LOGI(TAG, "Battery Monitor Task Started.");
    while(1) {
        updateBatteryStatus();
        // Use the configured interval
        TickType_t delay_ticks = pdMS_TO_TICKS(READ_INTERVAL_MS);
        if (delay_ticks == 0) delay_ticks = 1; // Minimum 1 tick delay
        vTaskDelay(delay_ticks);
    }
}

void BatteryService::updateBatteryStatus() {
    if (!m_oneshot_adc_handle || m_adc_channel == ADC_CHANNEL_0) {
        ESP_LOGE(TAG, "ADC handle (%p) or channel (%d) not initialized!", m_oneshot_adc_handle, (int)m_adc_channel);
        return;
    }

    int adc_raw_sum = 0;
    int valid_samples = 0;
    int read_raw = 0;

    for (int i = 0; i < OVERSAMPLING; i++) {
        esp_err_t read_ret = adc_oneshot_read(m_oneshot_adc_handle, m_adc_channel, &read_raw);
        if (read_ret == ESP_OK) {
            adc_raw_sum += read_raw;
            valid_samples++;
        } else {
             ESP_LOGE(TAG, "ADC oneshot read failed on chan %d (err %d)", (int)m_adc_channel, read_ret);
             vTaskDelay(pdMS_TO_TICKS(1)); // Small delay before next sample attempt
        }
    }

    if (valid_samples < OVERSAMPLING / 2) { // Require at least half the samples to be valid
        ESP_LOGE(TAG, "Insufficient valid ADC samples obtained (%d/%d).", valid_samples, OVERSAMPLING);
        // Optionally update status to indicate error?
        // {
        //     std::lock_guard<std::mutex> lock(m_status_mutex);
        //     m_latest_status.voltage = -1.0f; // Error indicator
        //     m_latest_status.percentage = -1;
        // }
        return;
    }

    int adc_reading_avg = adc_raw_sum / valid_samples;

    // Convert ADC reading to voltage in mV
    int voltage_mv = 0;
    if (m_adc_cali_enable && m_adc_cali_handle != nullptr) {
        esp_err_t cali_ret = adc_cali_raw_to_voltage(m_adc_cali_handle, adc_reading_avg, &voltage_mv);
         if (cali_ret != ESP_OK) {
             ESP_LOGW(TAG, "ADC calibration failed (%s), voltage inaccurate.", esp_err_to_name(cali_ret));
             voltage_mv = -1; // Indicate failure
         }
    } else {
         ESP_LOGW(TAG, "ADC calibration not enabled/valid, voltage reading inaccurate.");
         voltage_mv = -1; // Indicate calibration unavailable
    }

    float actual_voltage = 0.0f;
    int percentage = 0;
    bool is_low = false;

    if (voltage_mv >= 0) { // Proceed only if voltage calculation was possible
        float measured_voltage = static_cast<float>(voltage_mv) / 1000.0f;
        actual_voltage = measured_voltage * m_config.voltage_divider_ratio;

        float voltage_range = m_config.voltage_max - m_config.voltage_min;
        if (voltage_range < 0.1f) { // Avoid division by zero/small range
            ESP_LOGW(TAG, "Battery voltage range (max-min) is too small or invalid (%.2f). Percentage calculation unreliable.", voltage_range);
            percentage = (actual_voltage >= m_config.voltage_max) ? 100 : 0;
        } else {
            percentage = static_cast<int>(((actual_voltage - m_config.voltage_min) / voltage_range) * 100.0f);
        }
        percentage = std::max(0, std::min(100, percentage));
        is_low = actual_voltage <= (m_config.voltage_min + 0.1f); // Threshold slightly above min
    } else {
         ESP_LOGW(TAG, "Could not determine calibrated voltage.");
         actual_voltage = -1.0f; // Error indicator
         percentage = -1;      // Error indicator
         is_low = false;       // Assume not low if unknown
    }

    ESP_LOGI(TAG, "Battery Update: ADC Avg Raw=%d (%d samples), Calibrated mV=%d, Actual V=%.2f, Percent=%d%%",
             adc_reading_avg, valid_samples, voltage_mv, actual_voltage, percentage);

    BatteryStatus newStatus = {.voltage = actual_voltage, .percentage = percentage, .isLow = is_low};
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        // Only publish if status changed significantly? Optional.
        // if (std::fabs(newStatus.voltage - m_latest_status.voltage) > 0.05 || newStatus.percentage != m_latest_status.percentage) {
           m_latest_status = newStatus;
        // } else { return; } // Skip publish if no change
    }

    // Publish the updated status
    BatteryStatusUpdatedEvent event(newStatus);
    m_eventBus.publish(event);
}