#pragma once
#include "BaseEvent.hpp"
#include "IMUDataTypes.hpp"
struct ImuControlFault {
    uint64_t armId = 0;
    uint32_t generation = 0;
    int64_t observedUs = 0;
    IMUFaultReason reason = IMUFaultReason::STALE;
    int64_t sampleTimestampUs = 0, latestTimestampUs = 0;
    const char* cause = "unspecified";
    esp_err_t error = ESP_OK;
};
class CONTROL_ImuDataInvalid : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(CONTROL_ImuDataInvalid)
    const ImuControlFault fault;
    explicit CONTROL_ImuDataInvalid(const ImuControlFault& value) : fault(value) {}
};
