#pragma once

#include "Task.hpp" // Your base task class header
#include "freertos/semphr.h"

class FIFOProcessor;

// Task to process FIFO data via FIFOProcessor
class FIFOTask : public Task {
public:
    // Constructor without taskCore parameter
    explicit FIFOTask(FIFOProcessor& fifoProcessor);
    ~FIFOTask() override = default;

protected:
   void run() override;

private:
    static constexpr const char* TAG = "IMUFIFOTask";

    FIFOProcessor& m_fifoProcessor;
};