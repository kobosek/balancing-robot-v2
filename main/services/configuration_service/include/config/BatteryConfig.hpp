#pragma once

struct BatteryConfig {
    int adc_pin = 2;
    float voltage_divider_ratio = 2.0f;
    float voltage_max = 4.2f;
    float voltage_min = 3.3f;
    int adc_bitwidth = 12; // ADC_BITWIDTH_12
    int adc_atten = 3;     // ADC_ATTEN_DB_12

    bool operator!=(const BatteryConfig& other) const {
        return adc_pin != other.adc_pin ||
               voltage_divider_ratio != other.voltage_divider_ratio ||
               voltage_max != other.voltage_max ||
               voltage_min != other.voltage_min ||
               adc_bitwidth != other.adc_bitwidth ||
               adc_atten != other.adc_atten;
    }

    bool operator==(const BatteryConfig& other) const {
        return !(*this != other);
    }

    bool requiresHardwareInit(const BatteryConfig& other) const {
        return adc_pin != other.adc_pin ||
               adc_bitwidth != other.adc_bitwidth ||
               adc_atten != other.adc_atten;
    }
};
