#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <atomic>
#include <mutex>
class IMUService;
class IMUTask {
public:
    explicit IMUTask(IMUService& service);
    ~IMUTask();
    bool start();
    void stop();
    void wake();
    bool stopping() const { return m_stop.load(); }
    bool wait(uint32_t milliseconds);
    static void IRAM_ATTR interrupt(void* argument);
private:
    static void entry(void* argument);
    IMUService& m_service;
    std::mutex m_lifecycle;
    TaskHandle_t m_handle = nullptr;
    SemaphoreHandle_t m_exited = nullptr;
    std::atomic<bool> m_stop{false};
};
