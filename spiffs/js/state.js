import { normalizeImuStatus } from './imuStatus.js';
import { MAX_DATA_POINTS } from './constants.js';

// Helper to create initial data arrays
const createDataArray = () => Array(MAX_DATA_POINTS).fill(null);

export const appState = {
    // Telemetry Data Arrays
    telemetryData: {
        timestampUs: createDataArray(),       // Wire sample time when available
        pitchDeg: createDataArray(),          // Index 0 (Actual Pitch)
        speedLDPS: createDataArray(),         // Index 1 (Actual Left)
        speedRDPS: createDataArray(),         // Index 2 (Actual Right)
        speedSetpointLDPS: createDataArray(), // Index 5 (Target Left)
        speedSetpointRDPS: createDataArray(), // Index 6 (Target Right)
        desiredAngleDeg: createDataArray(),   // Index 7 (Target Pitch Offset)
        yawAngleDeg: createDataArray(),       // Index 8 (Actual Yaw Angle)
        targetYawAngleDeg: createDataArray(), // Index 9 (Target Yaw Angle)
        yawRateDPS: createDataArray(),        // Index 10 (Actual Yaw Rate)
        targetYawRateDPS: createDataArray(),  // Index 11 (Target Yaw Rate)
        targetPitchDeg: createDataArray(),    // v4 true thetaTarget
        positionM: createDataArray(),
        holdPositionM: createDataArray(),
        positionErrorM: createDataArray(),
        targetVelocityMps: createDataArray(),
        commandVelocityMps: createDataArray(),
        measuredVelocityMps: createDataArray(),
        distanceDifferenceM: createDataArray(),
        distanceDifferenceTargetM: createDataArray(),
        syncVelocityDifferenceMps: createDataArray(),
        requestedBalanceEffort: createDataArray(),
        balanceEffort: createDataArray(),
        requestedSyncEffort: createDataArray(),
        syncEffort: createDataArray(),
        leftEffort: createDataArray(),
        rightEffort: createDataArray(),
        joystickX: createDataArray(),         // Populated locally
        joystickY: createDataArray(),         // Populated locally
    },

    // <<< UPDATED telemetryKeys - Reflects object structure >>>
    telemetryKeys: [
        'pitchDeg',
        'speedLDPS',
        'speedRDPS',
        'speedSetpointLDPS',
        'speedSetpointRDPS',
        'desiredAngleDeg',
        'yawAngleDeg',
        'targetYawAngleDeg',
        'yawRateDPS',
        'targetYawRateDPS',
        'targetPitchDeg',
        'positionM',
        'holdPositionM',
        'positionErrorM',
        'targetVelocityMps',
        'commandVelocityMps',
        'measuredVelocityMps',
        'distanceDifferenceM',
        'distanceDifferenceTargetM',
        'syncVelocityDifferenceMps',
        'requestedBalanceEffort',
        'balanceEffort',
        'requestedSyncEffort',
        'syncEffort',
        'leftEffort',
        'rightEffort',
        'joystickX',
        'joystickY'
    ],

    // Current snapshot states
    telemetryImuGeneration: null,
    telemetryStrategyId: null,
    telemetryLoopMode: null,
    telemetryControlGeneration: null,
    telemetryOdometryGeneration: null,
    telemetryDroppedSamples: 0,
    telemetryPendingSamples: 0,
    currentSystemState: {
        imu: normalizeImuStatus(null),
        id: -1,
        state_id: null,
        name: 'UNKNOWN',
        state_name: 'UNKNOWN',
        active_balance_strategy: 'unknown',
        configured_balance_strategy: 'unknown',
        balance_strategy_config_revision: 0,
        config_revision: 0,
        command_session_id: '0',
        command_input_enabled: false,
        control_generation: 0,
        strategy_change_in_progress: false,
        operation_recovery_pending: false,
        operation_known: false,
        operation_active: false,
        operation_id: '0',
        operation_kind: 'none',
        operation_phase: 'idle',
        operation_result_code: 0,
        operation_base_revision: 0,
        operation_target_revision: 0,
        longitudinal_cascade_available: false,
        strategy_capabilities: {
            nested_pid: { active: false, configured: false, can_activate: false, loop_mode: 'nested_pid', turn_control_supported: true, revision: 0, reason: 'state unavailable' },
            longitudinal_cascade: { active: false, configured: false, can_activate: false, loop_mode: 'pitch_only', turn_control_supported: false, revision: 0, reason: 'state unavailable' }
        },
        auto_balancing_enabled: true,
        fall_detection_enabled: false,
        yaw_control_enabled: false,
        critical_battery_motor_shutdown_enabled: false,
        battery_voltage: 0,
        battery_adc_pin_voltage: 0,
        battery_percentage: 0,
        battery_is_low: false,
        battery_is_critical: false,
        battery_adc_calibrated: false,
        pid_tuning: {
            state: 'IDLE',
            target: 'motor_speed_left',
            phase: 'IDLE',
            progress: 0,
            message: 'Idle',
            has_candidate: false,
            candidate_strategy: 'nested_pid',
            candidate_base_revision_valid: false,
            candidate_base_config_revision: 0,
            save_in_progress: false,
            candidate: null,
            metrics: null
        },
        guided_calibration: {
            state: 'IDLE',
            phase: 'IDLE',
            progress: 0,
            message: 'Idle',
            left_direction_ok: false,
            right_direction_ok: false,
            left_deadzone_effort: 0,
            right_deadzone_effort: 0
        },
        ota: {
            available: false,
            spiffs_available: false,
            update_allowed: false,
            update_in_progress: false,
            reboot_required: false,
            bytes_written: 0,
            expected_size: 0,
            spiffs_partition_size: 0,
            app_version: 'unknown',
            active_target: 'none',
            bundle_id: '0',
            bundle_stage: 'none',
            bundle_recovery_pending: false,
            message: 'Unknown'
        }
    },
    currentBattery: { voltage: 0, percentage: 0 },

    // Caches
    configDataCache: null,
    telemetryJsonCache: null,
    configDrafts: {},

    // Timers
    timers: { dataFetch: null, stateFetch: null, wsReconnect: null, joystickSend: null, logsFetch: null },

    // WebSocket instance
    ws: null,

    logsPanelExpanded: false,
    webLogLines: [],
    webLogNextSequence: 0,

    // Joystick state
    joystick: { instance: null, currentData: { x: 0, y: 0 }, lastSentData: { x: -99, y: -99 }, isActive: false },

    // Graph states ( <<< ADDED Graph 3 state placeholder >>> )
    graphs: [
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 1
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 2
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 3
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 4
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }  // Graph 5
    ]
};

// --- State Modification Functions ---
// (updateConfigCache, invalidateConfigCache, updateTelemetryJsonCache, updateCurrentSystemState, updateBatteryState, updateTelemetryArray)
// No changes needed in the modification functions themselves.

export function updateConfigCache(data) { appState.configDataCache = data; console.log("Config cache updated."); }
export function invalidateConfigCache() { appState.configDataCache = null; console.log("Config cache invalidated."); }
export function updateTelemetryJsonCache(data) { appState.telemetryJsonCache = data; }
export function updateCurrentSystemState(newStateData) {
    const previousState = { ...appState.currentSystemState };
    appState.currentSystemState = { ...previousState, ...newStateData, imu: normalizeImuStatus(newStateData.imu) };
    if (previousState.state_id !== appState.currentSystemState.state_id ||
        previousState.state_name !== appState.currentSystemState.state_name ||
        previousState.active_balance_strategy !== appState.currentSystemState.active_balance_strategy ||
        previousState.configured_balance_strategy !== appState.currentSystemState.configured_balance_strategy ||
        previousState.command_session_id !== appState.currentSystemState.command_session_id ||
        previousState.operation_id !== appState.currentSystemState.operation_id ||
        previousState.operation_kind !== appState.currentSystemState.operation_kind ||
        previousState.operation_phase !== appState.currentSystemState.operation_phase ||
        previousState.operation_active !== appState.currentSystemState.operation_active ||
        previousState.operation_result_code !== appState.currentSystemState.operation_result_code ||
        previousState.operation_recovery_pending !== appState.currentSystemState.operation_recovery_pending ||
        previousState.operation_base_revision !== appState.currentSystemState.operation_base_revision ||
        previousState.operation_target_revision !== appState.currentSystemState.operation_target_revision ||
        previousState.strategy_change_in_progress !== appState.currentSystemState.strategy_change_in_progress ||
        previousState.auto_balancing_enabled !== appState.currentSystemState.auto_balancing_enabled ||
        previousState.fall_detection_enabled !== appState.currentSystemState.fall_detection_enabled ||
        previousState.yaw_control_enabled !== appState.currentSystemState.yaw_control_enabled ||
        previousState.critical_battery_motor_shutdown_enabled !== appState.currentSystemState.critical_battery_motor_shutdown_enabled ||
        previousState.pid_tuning?.state !== appState.currentSystemState.pid_tuning?.state ||
        previousState.pid_tuning?.phase !== appState.currentSystemState.pid_tuning?.phase ||
        previousState.guided_calibration?.state !== appState.currentSystemState.guided_calibration?.state ||
        previousState.guided_calibration?.phase !== appState.currentSystemState.guided_calibration?.phase ||
        previousState.ota?.message !== appState.currentSystemState.ota?.message ||
        previousState.ota?.reboot_required !== appState.currentSystemState.ota?.reboot_required ||
        previousState.ota?.bundle_stage !== appState.currentSystemState.ota?.bundle_stage ||
        previousState.ota?.bundle_recovery_pending !== appState.currentSystemState.ota?.bundle_recovery_pending)
    { console.log("State Update:", appState.currentSystemState); return true; }
    return false;
}
export function updateBatteryState(voltage, percentage) { appState.currentBattery.voltage = voltage; appState.currentBattery.percentage = percentage; }
function normalizeTelemetryValue(value, fallbackValue) {
    if (value !== null && value !== undefined) {
        const parsedValue = parseFloat(value);
        if (!isNaN(parsedValue)) { return parsedValue; }
    }
    return fallbackValue !== null ? fallbackValue : null;
}
export function updateTelemetryArray(key, value) {
    if (!appState.telemetryData.hasOwnProperty(key)) { console.warn(`Skipping non-existent telemetry key: ${key}`); return; }
    const targetArray = appState.telemetryData[key];
    const lastVal = targetArray.length > 0 ? targetArray[targetArray.length - 1] : null;
    const validValue = normalizeTelemetryValue(value, lastVal);
    targetArray.push(validValue);
    const overflow = targetArray.length - MAX_DATA_POINTS;
    if (overflow > 0) { targetArray.splice(0, overflow); }
}
export function updateTelemetryArrays(batchValuesByKey, preserveGaps = false) {
    Object.entries(batchValuesByKey).forEach(([key, values]) => {
        if (!appState.telemetryData.hasOwnProperty(key)) { console.warn(`Skipping non-existent telemetry key: ${key}`); return; }
        const targetArray = appState.telemetryData[key];
        let lastVal = targetArray.length > 0 ? targetArray[targetArray.length - 1] : null;
        values.forEach((value) => {
            lastVal = preserveGaps && value === null ? null : normalizeTelemetryValue(value, lastVal);
            targetArray.push(lastVal);
        });
        const overflow = targetArray.length - MAX_DATA_POINTS;
        if (overflow > 0) { targetArray.splice(0, overflow); }
    });
}
