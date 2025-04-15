#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include <cmath>
#include <string>
#include <memory>

#include "Application.hpp"

// Logging Tag
static const char* TAG = "AppMain";

// --- Main Application Entry Point ---
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "====== Balancing Robot Starting Up ======");

    // Create the Application instance
    Application app;

    // Initialize the application components
    if (app.init() != ESP_OK) {
        ESP_LOGE(TAG, "Application initialization failed! Halting.");
        return;
    }
    
    // Run the application (starts tasks, sets initial state)
    app.run();
    
    // Main task can now sleep
    ESP_LOGI(TAG, "======= App running, main task yielding =======");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Main task sleeps for 1 minute intervals
    }
}