// ================================================
// File: main/core/Application.cpp
// ================================================
#include "Application.hpp"

#include "ApplicationContext.hpp"
#include "ApplicationEventWiring.hpp"
#include "ApplicationRuntime.hpp"
#include "ConfigurationService.hpp"
#include "EventBus.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMUService.hpp"
#include "StateManager.hpp"
#include "SystemState.hpp"
#include "esp_check.h"
#include "esp_log.h"

Application::Application()
{
    ESP_LOGI(TAG, "Creating Application instance");
}

Application::~Application()
{
    ESP_LOGI(TAG, "Application instance destroyed");
}

esp_err_t Application::init()
{
    ESP_LOGI(TAG, "Initializing Application");

    m_context = std::make_unique<ApplicationContext>();
    ESP_RETURN_ON_FALSE(m_context != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate application context");

    esp_err_t ret = m_context->initialize();
    ESP_RETURN_ON_ERROR(ret, TAG, "Application context initialization failed");

    m_eventWiring = std::make_unique<ApplicationEventWiring>();
    ESP_RETURN_ON_FALSE(m_eventWiring != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate event wiring");

    ret = m_eventWiring->wire(*m_context);
    ESP_RETURN_ON_ERROR(ret, TAG, "Application event wiring failed");

    m_context->eventBus().publish(IMU_AvailabilityChanged(m_context->imuService().isAvailable()));

    ESP_LOGI(TAG, "Application initialization complete");
    return ESP_OK;
}

void Application::run()
{
    ESP_LOGI(TAG, "Running Application");

    ESP_RETURN_VOID_ON_FALSE(m_context != nullptr, ESP_ERR_INVALID_STATE, TAG, "Application context is not initialized");

    const int intervalMs = m_context->configService().getMainLoopConfig().interval_ms;
    const int batteryIntervalMs = m_context->configService().getSystemBehaviorConfig().battery_read_interval_ms;

    m_runtime = std::make_unique<ApplicationRuntime>();
    if (m_runtime == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate application runtime");
        m_context->stateManager().setState(SystemState::FATAL_ERROR);
        return;
    }

    const esp_err_t ret = m_runtime->start(*m_context, intervalMs, batteryIntervalMs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start required tasks. Entering FATAL_ERROR state.");
        m_context->stateManager().setState(SystemState::FATAL_ERROR);
        return;
    }

    m_context->stateManager().setState(SystemState::IDLE);
    ESP_LOGI(TAG, "System State set to IDLE");
    ESP_LOGI(TAG, "Application running");
}
