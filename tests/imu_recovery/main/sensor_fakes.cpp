#include "sensor_fakes.hpp"
#include "I2CDevice.hpp"
#include "IMUDataReadyInterrupt.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <array>
#include <algorithm>
#include <mutex>
namespace sensor_fake {
std::atomic<bool> absent{false}, noSamples{false}, overflow{false};
std::atomic<int> failReadRegister{-1}, readFailures{0}, failWriteRegister{-1}, writeFailures{0};
std::atomic<int> fifoFailureConsume{-1}, fixedCount{-1}, fifoFillByte{-1};
std::atomic<int64_t> clockUs{-1};
std::atomic<unsigned> irqSetupDelayMs{0}, fifoSaturateAxes{0};
std::atomic<unsigned> fifoReads{0}, countReads{0}, resets{0}, opens{0}, closes{0}, wrongOwner{0};
std::mutex mutex;
std::array<uint8_t, 256> registers{};
int64_t enabledAt = 0;
unsigned consumed = 0;
bool enabled = false;
TaskHandle_t owner = nullptr;
void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    absent = noSamples = overflow = false;
    failReadRegister = failWriteRegister = fifoFailureConsume = fixedCount = fifoFillByte = -1;
    readFailures = writeFailures = 0;
    fifoReads = countReads = resets = opens = closes = wrongOwner = 0;
    irqSetupDelayMs = fifoSaturateAxes = 0; clockUs = -1; registers = {}; registers[0x75] = 0x68; registers[0x1c] = 0x08;
    enabledAt = 0; consumed = 0; enabled = false; owner = nullptr;
}
void recordOwner() {
    const auto current = xTaskGetCurrentTaskHandle();
    if (!owner) owner = current;
    if (owner != current) ++wrongOwner;
}
}
extern "C" int64_t __real_esp_timer_get_time();
extern "C" int64_t __wrap_esp_timer_get_time() {
    const auto now = sensor_fake::clockUs.load();
    return now >= 0 ? now : __real_esp_timer_get_time();
}
I2CDevice::I2CDevice() : m_busHandle(nullptr), m_deviceHandle(nullptr) {}
I2CDevice::~I2CDevice() { close(); }
esp_err_t I2CDevice::open(i2c_port_t, gpio_num_t, gpio_num_t, uint16_t, uint32_t) {
    std::lock_guard<std::mutex> lock(sensor_fake::mutex);
    sensor_fake::recordOwner(); ++sensor_fake::opens;
    m_deviceHandle = reinterpret_cast<i2c_master_dev_handle_t>(this);
    return ESP_OK;
}
esp_err_t I2CDevice::close() {
    std::lock_guard<std::mutex> lock(sensor_fake::mutex);
    if (m_deviceHandle) { sensor_fake::recordOwner(); ++sensor_fake::closes; }
    m_deviceHandle = nullptr;
    return ESP_OK;
}
bool I2CDevice::isOpen() const { return m_deviceHandle != nullptr; }
void I2CDevice::setConfig(const I2CConfig& config) { m_config = config; }
I2CConfig I2CDevice::getConfig() const { return m_config; }
esp_err_t I2CDevice::writeRegister(uint8_t reg, uint8_t data) {
    using namespace sensor_fake;
    std::lock_guard<std::mutex> lock(mutex); recordOwner();
    if (absent || !m_deviceHandle) return ESP_ERR_TIMEOUT;
    if (reg == failWriteRegister && writeFailures > 0) { --writeFailures; return ESP_ERR_TIMEOUT; }
    registers[reg] = data;
    if (reg == 0x6b && data == 0x80) ++resets;
    if (reg == 0x6a) {
        if (data & 4) { consumed = 0; enabledAt = esp_timer_get_time(); }
        enabled = (data & 0x40) != 0;
        if (enabled) { consumed = 0; enabledAt = esp_timer_get_time(); }
    }
    return ESP_OK;
}
esp_err_t I2CDevice::readRegisters(uint8_t reg, uint8_t* data, size_t len, uint32_t) const {
    using namespace sensor_fake;
    std::lock_guard<std::mutex> lock(mutex); recordOwner();
    if (reg == 0x72) ++countReads;
    if (reg == 0x74) ++fifoReads;
    if (absent || !m_deviceHandle) return ESP_ERR_TIMEOUT;
    if (reg == failReadRegister && readFailures > 0) { --readFailures; return ESP_ERR_TIMEOUT; }
    std::fill(data, data + len, 0);
    if (reg == 0x72) {
        const unsigned period = (registers[0x19] + 1) * (registers[0x1a] == 0 ? 125 : 1000);
        const unsigned produced = enabled && !noSamples ? (esp_timer_get_time() - enabledAt) / period : 0;
        unsigned count = produced * 12 > consumed ? produced * 12 - consumed : 0;
        if (fixedCount >= 0) count = fixedCount;
        data[0] = count >> 8; data[1] = count;
    } else if (reg == 0x74) {
        const int prefix = fifoFailureConsume.exchange(-1);
        if (prefix >= 0) { consumed += std::min<unsigned>(prefix, len); return ESP_ERR_TIMEOUT; }
        const uint16_t oneG = 16384U >> ((registers[0x1c] >> 3) & 3);
        for (unsigned i = 0; i + 11 < len; i += 12) { data[i + 4] = oneG >> 8; data[i + 5] = oneG; }
        for (unsigned i = 0; i + 11 < len; i += 12) {
            for (unsigned axis = 0; axis < 6; ++axis) if (fifoSaturateAxes & (1U << axis)) {
                data[i + axis * 2] = axis % 2 ? 0x80 : 0x7f;
                data[i + axis * 2 + 1] = axis % 2 ? 0 : 0xff;
            }
        }
        if (fifoFillByte >= 0) std::fill(data, data + len, fifoFillByte.load());
        consumed += len;
    } else if (reg == 0x3a) {
        data[0] = overflow.exchange(false) ? 0x10 : 0;
    } else {
        for (unsigned i = 0; i < len; ++i) data[i] = registers[reg + i];
    }
    return ESP_OK;
}
std::atomic<bool> IMUDataReadyInterrupt::s_serviceInstalled{false};
IMUDataReadyInterrupt::IMUDataReadyInterrupt() : m_pin(GPIO_NUM_MAX), m_activeHigh(true), m_handlerInstalled(false) {}
IMUDataReadyInterrupt::~IMUDataReadyInterrupt() { deinit(); }
esp_err_t IMUDataReadyInterrupt::init(gpio_num_t pin, bool high, gpio_isr_t, void*) {
    if (sensor_fake::irqSetupDelayMs) vTaskDelay(pdMS_TO_TICKS(sensor_fake::irqSetupDelayMs.load()));
    m_pin = pin; m_activeHigh = high; m_handlerInstalled = true; return ESP_OK;
}
esp_err_t IMUDataReadyInterrupt::deinit() { m_handlerInstalled = false; return ESP_OK; }
// IRQ notifications are deliberately suppressed: worker integration uses fallback polling.
