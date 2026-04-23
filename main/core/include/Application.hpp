#pragma once

#include <memory>
#include "esp_err.h"

class ApplicationContext;
class ApplicationEventWiring;
class ApplicationRuntime;

class Application {
public:
    Application();
    ~Application();

    esp_err_t init();
    void run();

private:
    static constexpr const char* TAG = "Application";

    std::unique_ptr<ApplicationContext> m_context;
    std::unique_ptr<ApplicationEventWiring> m_eventWiring;
    std::unique_ptr<ApplicationRuntime> m_runtime;
};
