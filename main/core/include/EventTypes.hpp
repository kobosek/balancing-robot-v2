#pragma once

enum class EventType {
    // Configuration Events
    CONFIG_FULL_UPDATE,              // CONFIG_FullConfigUpdate
    CONFIG_PID_UPDATE,               // CONFIG_PidConfigUpdate
    CONFIG_IMU_UPDATE,               // CONFIG_ImuConfigUpdate
    CONFIG_MOTOR_UPDATE,             // CONFIG_MotorConfigUpdate
    CONFIG_ENCODER_UPDATE,           // CONFIG_EncoderConfigUpdate
    CONFIG_BATTERY_UPDATE,           // CONFIG_BatteryConfigUpdate
    CONFIG_BEHAVIOR_UPDATE,          // CONFIG_BehaviorConfigUpdate
    CONFIG_WIFI_UPDATE,              // CONFIG_WiFiConfigUpdate
    CONFIG_CONTROL_UPDATE,           // CONFIG_ControlConfigUpdate
    CONFIG_GYRO_OFFSETS_UPDATE,      // IMU_GyroOffsetsUpdated

    // System Events
    SYSTEM_STATE_CHANGED,            // SYSTEM_StateChanged

    // Sensor Events
    IMU_ORIENTATION_DATA,            // IMU_OrientationData
    BATTERY_STATUS_UPDATE,           // BATTERY_StatusUpdate

    // UI Command Events
    UI_START_BALANCING,              // UI_StartBalancing
    UI_STOP,                         // UI_Stop
    UI_CALIBRATE_IMU,                // UI_CalibrateImu
    UI_ENABLE_FALL_RECOVERY,         // UI_EnableFallRecovery
    UI_DISABLE_FALL_RECOVERY,        // UI_DisableFallRecovery
    UI_ENABLE_FALL_DETECTION,        // UI_EnableFallDetection
    UI_DISABLE_FALL_DETECTION,       // UI_DisableFallDetection
    UI_JOYSTICK_INPUT,               // UI_JoystickInput

    // Internal Requests / Notifications
    IMU_CALIBRATION_REQUEST,         // IMU_CalibrationRequest
    IMU_CALIBRATION_COMPLETED,       // IMU_CalibrationCompleted
    MOTION_TARGET_SET,               // MOTION_TargetMovement

    // Diagnostic/Status Events
    TELEMETRY_SNAPSHOT,              // TELEMETRY_Snapshot
    MOTION_FALL_DETECTED,            // MOTION_FallDetected
    BATTERY_LOW_WARNING,             // BATTERY_LowWarning (not implemented yet)
    IMU_COMMUNICATION_ERROR,         // IMU_CommunicationError

    // IMU Recovery Events
    IMU_RECOVERY_SUCCEEDED,          // IMU_RecoverySucceeded
    IMU_RECOVERY_FAILED,             // IMU_RecoveryFailed

    // IMU State Management Events
    IMU_STATE_CHANGED,               // IMU_StateChanged
    IMU_CALIBRATION_REJECTED,        // IMU_CalibrationRequestRejected
    MOTION_USING_FALLBACK_VALUES,    // MOTION_UsingFallbackValues

    // Error reporting
    ERROR_REPORTED                   // Not implemented yet
};

class BaseEvent; // Forward declaration