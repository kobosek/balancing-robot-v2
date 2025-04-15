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
#include <mutex>                        // Moved from header
#include "esp_rom_sys.h"                // For esp_rom_delay_us

BatteryService::BatteryService(const BatteryConfig& config, EventBus& bus) :
    m_config(config),
    m_eventBus(bus),
    m_adc_channel(ADC_CHANNEL_0), // Initialize channel
    m_adc_cali_handle(nullptr),
    m_adc_cali_enable(false),
    m_oneshot_adc_handle(nullptr) // Initialize new handle
{
    m_latest_status.percentage = -1;
    m_latest_status.voltage = -1.0f; // Indicate invalid initial voltage
    m_latest_status.isLow = false;
}

BatteryService::~BatteryService() {
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

BatteryStatus BatteryService::getLatestStatus() const {
     std::lock_guard<std::mutex> lock(m_status_mutex);
     return m_latest_status;
}

void BatteryService::updateBatteryStatus() {
    if (!m_oneshot_adc_handle || m_adc_channel == ADC_CHANNEL_0) {
        ESP_LOGW(TAG, "ADC not initialized properly, cannot read battery voltage.");
        return;
    }

    // Read raw ADC value with oversampling
    int raw_sum = 0;
    for (int i = 0; i < OVERSAMPLING; i++) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(m_oneshot_adc_handle, m_adc_channel, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed (%s)! Skipping sample %d.", esp_err_to_name(ret), i);
            continue;
        }
        raw_sum += raw_value;
        // Small delay between samples
        if (i < OVERSAMPLING - 1) { // Don't delay after the last sample
            esp_rom_delay_us(10); // very short microsecond delay
        }
    }

    // Calculate average
    int adc_raw = raw_sum / OVERSAMPLING;
    ESP_LOGD(TAG, "Raw ADC value: %d (average of %d samples)", adc_raw, OVERSAMPLING);

    // Convert to voltage
    float voltage = 0.0f;
    if (m_adc_cali_enable && m_adc_cali_handle) {
        int volt_mv;
        esp_err_t ret = adc_cali_raw_to_voltage(m_adc_cali_handle, adc_raw, &volt_mv);
        if (ret == ESP_OK) {
            voltage = volt_mv / 1000.0f; // Convert mV to V
        } else {
            ESP_LOGW(TAG, "ADC calibration conversion failed (%s)!", esp_err_to_name(ret));
            // Fall back to manual conversion if calibration fails
            voltage = (adc_raw * 0.0008f); // Default multiplier if no calibration
        }
    } else {
        // Manual conversion using approximate multiplier
        voltage = (adc_raw * 0.0008f); // Default multiplier if no calibration
    }

    // Apply resistor divider correction from config
    if (m_config.voltage_divider_ratio > 1.0f) {
        voltage *= m_config.voltage_divider_ratio;
    }

    // No offset correction in the current BatteryConfig

    ESP_LOGD(TAG, "Battery voltage: %.2fV (unclamped)", voltage);

    // Sanity check - clamp voltage to reasonable range
    if (voltage < 0.1f) {
        ESP_LOGW(TAG, "Invalid voltage reading (%.2fV), clamping to min", voltage);
        voltage = 0.1f;
    } else if (voltage > 20.0f) { // Assume no battery > 20V
        ESP_LOGW(TAG, "Unreasonable voltage reading (%.2fV), clamping to max", voltage);
        voltage = 20.0f;
    }

    // Calculate percentage based on min/max from config
    float percentage = 0.0f;
    if (voltage <= m_config.voltage_min) {
        percentage = 0.0f;
    } else if (voltage >= m_config.voltage_max) {
        percentage = 100.0f;
    } else {
        percentage = ((voltage - m_config.voltage_min) / 
                     (m_config.voltage_max - m_config.voltage_min)) * 100.0f;
    }
    
    // Round to integer
    int rounded_percentage = static_cast<int>(percentage + 0.5f);
    
    // Check if battery is critically low (10% safety margin above min)
    bool is_low = voltage <= (m_config.voltage_min + (m_config.voltage_max - m_config.voltage_min) * 0.1f);
    
    // Create battery status
    BatteryStatus new_status;
    new_status.voltage = voltage;
    new_status.percentage = rounded_percentage;
    new_status.isLow = is_low;
    
    // Check if status has changed significantly
    bool status_changed = false;
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        
        if (m_latest_status.percentage != rounded_percentage || 
            std::abs(m_latest_status.voltage - voltage) > 0.05f ||
            m_latest_status.isLow != is_low) {
            
            status_changed = true;
            m_latest_status = new_status;
        }
    }
    
    // Publish event if status changed
    if (status_changed) {
        ESP_LOGI(TAG, "Battery status updated: %.2fV, %d%%, %s", 
                voltage, rounded_percentage, is_low ? "LOW" : "OK");
        
        BatteryStatusUpdatedEvent event(new_status);
        m_eventBus.publish(event);
    } else {
        ESP_LOGD(TAG, "Battery status unchanged: %.2fV, %d%%, %s", 
                voltage, rounded_percentage, is_low ? "LOW" : "OK");
    }
}