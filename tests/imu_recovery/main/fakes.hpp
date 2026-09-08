#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace motor_fake {
struct Write {
    uint32_t duty1;
    uint32_t duty2;
};
extern std::array<Write, 32> writes;
extern std::atomic<unsigned> count;
extern std::atomic<bool> pauseNextWrite;
extern SemaphoreHandle_t writeEntered;
extern SemaphoreHandle_t releaseWrite;
void reset();
}
