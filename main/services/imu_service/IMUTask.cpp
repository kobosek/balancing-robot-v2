#include "IMUTask.hpp"
#include "IMUService.hpp"
#include <algorithm>
IMUTask::IMUTask(IMUService& service) : m_service(service), m_exited(xSemaphoreCreateBinary()) {}
IMUTask::~IMUTask() { stop(); if (m_exited) vSemaphoreDelete(m_exited); }
bool IMUTask::start() {
    std::lock_guard<std::mutex> lock(m_lifecycle);
    if (m_handle) return true;
    if (!m_exited) return false;
    m_stop = false;
    return xTaskCreatePinnedToCore(entry, "IMU", 6144, this, configMAX_PRIORITIES - 2, &m_handle, 0) == pdPASS;
}
void IMUTask::stop() {
    std::lock_guard<std::mutex> lock(m_lifecycle);
    if (!m_handle) return;
    m_stop = true;
    xTaskNotifyGive(m_handle);
    xSemaphoreTake(m_exited, portMAX_DELAY);
    m_handle = nullptr;
}
void IMUTask::wake() {
    // Event handlers must never wait behind cooperative shutdown.
    std::unique_lock<std::mutex> lock(m_lifecycle, std::try_to_lock);
    if (lock.owns_lock() && m_handle) xTaskNotifyGive(m_handle);
}
bool IMUTask::wait(uint32_t milliseconds) {
    return ulTaskNotifyTake(pdTRUE, std::max<TickType_t>(1, pdMS_TO_TICKS(milliseconds))) != 0;
}
void IRAM_ATTR IMUTask::interrupt(void* argument) {
    auto* self = static_cast<IMUTask*>(argument);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(self->m_handle, &woken);
    if (woken) portYIELD_FROM_ISR();
}
void IMUTask::entry(void* argument) {
    auto* self = static_cast<IMUTask*>(argument);
    self->m_service.runWorker(*self);
    const auto exited = self->m_exited;
    xSemaphoreGive(exited);
    // No access to the task/service object after acknowledging exit.
    vTaskDelete(nullptr);
}
