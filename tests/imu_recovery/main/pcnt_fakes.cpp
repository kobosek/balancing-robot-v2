#include "pcnt_fakes.hpp"
#include <array>
struct pcnt_unit_t {
    int low = 0, high = 0, hardware = 0, accumulator = 0;
    bool allocated = false, lowWatch = false, highWatch = false;
    bool cleared = false, enabled = false, running = false, accumulate = false;
};
struct pcnt_chan_t {};
namespace pcnt_fake {
std::array<pcnt_unit_t, 2> units{};
std::array<pcnt_chan_t, 4> channels{};
std::array<int, 2> nextReadOffset{};
unsigned channelIndex = 0, orderingErrors = 0, liveUnits = 0;
Operation failNext = Operation::NONE;
int failWheel = -1;
bool fail(Operation operation, pcnt_unit_handle_t unit) {
    if (operation == failNext && (failWheel < 0 || unit == &units[failWheel])) {
        failNext = Operation::NONE;
        return true;
    }
    return false;
}
void reset() { units = {}; nextReadOffset = {}; channelIndex = orderingErrors = liveUnits = 0; failNext = Operation::NONE; failWheel = -1; }
void reportBeforeOverflowISR(unsigned wheel, int limit) { nextReadOffset[wheel] = -limit; }
void setCount(unsigned wheel, int count) { units[wheel].hardware = 0; units[wheel].accumulator = count; }
void advance(unsigned wheel, int pulses) {
    auto& u = units[wheel];
    if (!u.running) return;
    const int direction = pulses < 0 ? -1 : 1;
    while (pulses != 0) {
        u.hardware += direction; pulses -= direction;
        if (u.hardware == u.high || u.hardware == u.low) {
            if (u.accumulate && (u.hardware == u.high ? u.highWatch : u.lowWatch)) u.accumulator += u.hardware;
            u.hardware = 0; // Hardware resets at the limit; never high -> low.
        }
    }
}
}
extern "C" {
esp_err_t pcnt_new_unit(const pcnt_unit_config_t* config, pcnt_unit_handle_t* result) {
    for (auto& unit : pcnt_fake::units) if (!unit.allocated) {
        unit = {}; unit.allocated = true; unit.low = config->low_limit; unit.high = config->high_limit;
        unit.accumulate = config->flags.accum_count;
        *result = &unit; ++pcnt_fake::liveUnits; return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}
esp_err_t pcnt_del_unit(pcnt_unit_handle_t unit) { unit->allocated = false; --pcnt_fake::liveUnits; return ESP_OK; }
esp_err_t pcnt_new_channel(pcnt_unit_handle_t, const pcnt_chan_config_t*, pcnt_channel_handle_t* result) {
    *result = &pcnt_fake::channels[pcnt_fake::channelIndex++ % 4]; return ESP_OK;
}
esp_err_t pcnt_del_channel(pcnt_channel_handle_t) { return ESP_OK; }
esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t, pcnt_channel_edge_action_t, pcnt_channel_edge_action_t) { return ESP_OK; }
esp_err_t pcnt_channel_set_level_action(pcnt_channel_handle_t, pcnt_channel_level_action_t, pcnt_channel_level_action_t) { return ESP_OK; }
esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t, const pcnt_glitch_filter_config_t*) { return ESP_OK; }
esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t u, int point) {
    if (pcnt_fake::fail(point < 0 ? pcnt_fake::Operation::LOW_WATCH : pcnt_fake::Operation::HIGH_WATCH, u)) return ESP_FAIL;
    if (u->enabled) ++pcnt_fake::orderingErrors;
    if (point == u->low) u->lowWatch = true;
    else if (point == u->high) u->highWatch = true;
    else return ESP_ERR_INVALID_ARG;
    u->cleared = false; return ESP_OK;
}
esp_err_t pcnt_unit_remove_watch_point(pcnt_unit_handle_t u, int point) {
    if (point == u->low) u->lowWatch = false;
    if (point == u->high) u->highWatch = false;
    return ESP_OK;
}
esp_err_t pcnt_unit_enable(pcnt_unit_handle_t u) {
    if (pcnt_fake::fail(pcnt_fake::Operation::ENABLE, u)) return ESP_FAIL;
    if (!u->lowWatch || !u->highWatch || !u->cleared) ++pcnt_fake::orderingErrors;
    u->enabled = true; return ESP_OK;
}
esp_err_t pcnt_unit_disable(pcnt_unit_handle_t u) { u->enabled = false; return ESP_OK; }
esp_err_t pcnt_unit_start(pcnt_unit_handle_t u) {
    if (pcnt_fake::fail(pcnt_fake::Operation::START, u)) return ESP_FAIL;
    u->running = true; return ESP_OK;
}
esp_err_t pcnt_unit_stop(pcnt_unit_handle_t u) {
    if (pcnt_fake::fail(pcnt_fake::Operation::STOP, u)) return ESP_FAIL;
    u->running = false; return ESP_OK;
}
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t u) {
    if (pcnt_fake::fail(pcnt_fake::Operation::CLEAR, u)) return ESP_FAIL;
    if (u->running) ++pcnt_fake::orderingErrors;
    u->hardware = u->accumulator = 0; u->cleared = true; return ESP_OK;
}
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t u, int* count) {
    if (pcnt_fake::fail(pcnt_fake::Operation::READ, u)) return ESP_FAIL;
    const auto wheel = u - pcnt_fake::units.data();
    *count = u->accumulator + u->hardware + pcnt_fake::nextReadOffset[wheel];
    pcnt_fake::nextReadOffset[wheel] = 0; // The deferred ISR completes after this observation.
    return ESP_OK;
}
}
