#include "fakes.hpp"
#include "MX1616H_HWDriver.hpp"

namespace motor_fake {
std::array<Write, 32> writes{};
std::atomic<unsigned> count{0};
std::atomic<bool> pauseNextWrite{false};
SemaphoreHandle_t writeEntered = nullptr;
SemaphoreHandle_t releaseWrite = nullptr;

void reset() {
    writes = {};
    count = 0;
    pauseNextWrite = false;
}
}

MX1616H_HWDriver::MX1616H_HWDriver(gpio_num_t pin1, gpio_num_t pin2,
    ledc_channel_t channel1, ledc_channel_t channel2, ledc_timer_t timer,
    ledc_mode_t mode, ledc_timer_bit_t resolution, uint32_t frequency)
    : m_pin_in1(pin1), m_pin_in2(pin2), m_channel1(channel1), m_channel2(channel2),
      m_timer_num(timer), m_speed_mode(mode), m_duty_resolution(resolution),
      m_pwm_frequency(frequency), m_is_initialized(true) {}

esp_err_t MX1616H_HWDriver::init() { return ESP_OK; }

esp_err_t MX1616H_HWDriver::setRawDuty(uint32_t duty1, uint32_t duty2) {
    if (motor_fake::pauseNextWrite.exchange(false)) {
        xSemaphoreGive(motor_fake::writeEntered);
        xSemaphoreTake(motor_fake::releaseWrite, portMAX_DELAY);
    }
    const unsigned index = motor_fake::count.load();
    if (index < motor_fake::writes.size()) {
        motor_fake::writes[index] = {duty1, duty2};
    }
    motor_fake::count.store(index + 1);
    return ESP_OK;
}
