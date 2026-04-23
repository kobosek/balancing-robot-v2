#pragma once

enum class PidTuningTarget {
    MOTOR_SPEED_LEFT,
    MOTOR_SPEED_RIGHT
};

enum class PidTuningState {
    IDLE,
    RUNNING,
    PREVIEW_READY,
    SAVED,
    DISCARDED,
    FAILED,
    CANCELED
};

enum class PidTuningPhase {
    IDLE,
    RESET,
    REST_BEFORE_STEP,
    LEFT_FORWARD,
    LEFT_REVERSE,
    RIGHT_FORWARD,
    RIGHT_REVERSE,
    COMPUTE,
    VALIDATE_LEFT,
    REST_BEFORE_VALIDATE_RIGHT,
    VALIDATE_RIGHT,
    PREVIEW
};
