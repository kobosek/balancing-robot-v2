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
    UI_ENABLE_AUTO_BALANCING,        // UI_EnableAutoBalancing
    UI_DISABLE_AUTO_BALANCING,       // UI_DisableAutoBalancing
    UI_ENABLE_FALL_DETECTION,        // UI_EnableFallDetection
    UI_DISABLE_FALL_DETECTION,       // UI_DisableFallDetection
    UI_JOYSTICK_INPUT,               // UI_JoystickInput
    UI_START_PID_TUNING,             // UI_StartPidTuning
    UI_CANCEL_PID_TUNING,            // UI_CancelPidTuning
    UI_SAVE_PID_TUNING,              // UI_SavePidTuning
    UI_DISCARD_PID_TUNING,           // UI_DiscardPidTuning
    UI_START_GUIDED_CALIBRATION,     // UI_StartGuidedCalibration
    UI_CANCEL_GUIDED_CALIBRATION,    // UI_CancelGuidedCalibration

    // Internal Requests / Notifications
    IMU_CALIBRATION_REQUEST,         // IMU_CalibrationRequest
    IMU_CALIBRATION_COMPLETED,       // IMU_CalibrationCompleted
    MOTION_TARGET_SET,               // MOTION_TargetMovement
    PID_TUNING_FINISHED,             // PID_TuningFinished
    GUIDED_CALIBRATION_FINISHED,     // GUIDED_CalibrationFinished

    // Diagnostic/Status Events
    TELEMETRY_SNAPSHOT,              // TELEMETRY_Snapshot
    BATTERY_LOW_WARNING,             // BATTERY_LowWarning (not implemented yet)
    IMU_COMMUNICATION_ERROR,         // IMU_CommunicationError

    //Balance Monitoring
    BALANCE_FALL_DETECTED,            // BALANCE_FallDetected
    BALANCE_AUTO_BALANCE_READY,       // BALANCE_AutoBalanceReady

    // IMU State Management Events
    IMU_CALIBRATION_REJECTED,        // IMU_CalibrationRequestRejected
    IMU_AVAILABILITY_CHANGED,        // IMU_AvailabilityChanged
    IMU_ATTACH_REQUESTED,            // IMU_AttachRequested

    // Error reporting
    ERROR_REPORTED                   // Not implemented yet
};

class BaseEvent; // Forward declaration
