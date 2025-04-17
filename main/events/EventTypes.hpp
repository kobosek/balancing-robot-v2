#pragma once

enum class EventType {
    // Configuration Events
    CONFIG_UPDATED,

    // State Events
    SYSTEM_STATE_CHANGED,

    // Sensor Events
    ORIENTATION_DATA_READY,
    BATTERY_STATUS_UPDATED,

    // Command Events (Received from external sources like Web UI)
    START_COMMAND_RECEIVED,
    STOP_COMMAND_RECEIVED,
    CALIBRATE_COMMAND_RECEIVED,
    ENABLE_RECOVERY_COMMAND_RECEIVED,
    DISABLE_RECOVERY_COMMAND_RECEIVED,
    ENABLE_FALL_DETECT_COMMAND_RECEIVED,
    DISABLE_FALL_DETECT_COMMAND_RECEIVED,
    JOYSTICK_INPUT_RECEIVED,

    // Internal Requests / Notifications
    START_CALIBRATION_REQUEST,
    CALIBRATION_STARTED,
    CALIBRATION_COMPLETE,
    TARGET_MOVEMENT_CMD_SET,

    // Diagnostic/Status Events
    TELEMETRY_SNAPSHOT_READY, // Likely unused now
    FALL_DETECTED,
    LOW_BATTERY_WARNING,
    IMU_COMMUNICATION_ERROR,

    // IMU Recovery Events
    ATTEMPT_IMU_RECOVERY_COMMAND,
    IMU_RECOVERY_SUCCEEDED,
    IMU_RECOVERY_FAILED,

    // IMU Calibration Events
    GYRO_OFFSETS_UPDATED,            // IMUCalibrationService -> ConfigurationService (Save offsets)

    ERROR_REPORTED
};

class BaseEvent; // Forward declaration