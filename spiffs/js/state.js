import { MAX_DATA_POINTS } from './constants.js';

// Helper to create initial data arrays
const createDataArray = () => Array(MAX_DATA_POINTS).fill(null);

export const appState = {
    // Telemetry Data Arrays
    telemetryData: {
        pitchDeg: createDataArray(),          // Index 0 (Actual Pitch)
        speedLDPS: createDataArray(),         // Index 1 (Actual Left)
        speedRDPS: createDataArray(),         // Index 2 (Actual Right)
        speedSetpointLDPS: createDataArray(), // Index 5 (Target Left)
        speedSetpointRDPS: createDataArray(), // Index 6 (Target Right)
        desiredAngleDeg: createDataArray(),   // Index 7 (Target Pitch Offset)
        yawRateDPS: createDataArray(),        // <<< ADDED: Index 8 (Actual Yaw Rate)
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
        'yawRateDPS', // <<< ADDED Key
        'joystickX',
        'joystickY'
    ],

    // Current snapshot states
    currentSystemState: {
        id: -1,
        state_id: null,
        name: 'UNKNOWN',
        state_name: 'UNKNOWN',
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
            update_in_progress: false,
            reboot_required: false,
            bytes_written: 0,
            expected_size: 0,
            spiffs_partition_size: 0,
            app_version: 'unknown',
            active_target: 'none',
            message: 'Unknown'
        }
    },
    currentBattery: { voltage: 0, percentage: 0 },

    // Caches
    configDataCache: null,
    telemetryJsonCache: null,

    // Timers
    timers: { dataFetch: null, stateFetch: null, wsReconnect: null, joystickSend: null },

    // WebSocket instance
    ws: null,

    // Joystick state
    joystick: { instance: null, currentData: { x: 0, y: 0 }, lastSentData: { x: -99, y: -99 }, isActive: false },

    // Graph states ( <<< ADDED Graph 3 state placeholder >>> )
    graphs: [
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 1
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }, // Graph 2
        { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }  // Graph 3 <<< ADDED
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
    appState.currentSystemState = { ...previousState, ...newStateData };
    if (previousState.state_id !== appState.currentSystemState.state_id ||
        previousState.state_name !== appState.currentSystemState.state_name ||
        previousState.auto_balancing_enabled !== appState.currentSystemState.auto_balancing_enabled ||
        previousState.fall_detection_enabled !== appState.currentSystemState.fall_detection_enabled ||
        previousState.yaw_control_enabled !== appState.currentSystemState.yaw_control_enabled ||
        previousState.critical_battery_motor_shutdown_enabled !== appState.currentSystemState.critical_battery_motor_shutdown_enabled ||
        previousState.pid_tuning?.state !== appState.currentSystemState.pid_tuning?.state ||
        previousState.pid_tuning?.phase !== appState.currentSystemState.pid_tuning?.phase ||
        previousState.guided_calibration?.state !== appState.currentSystemState.guided_calibration?.state ||
        previousState.guided_calibration?.phase !== appState.currentSystemState.guided_calibration?.phase ||
        previousState.ota?.message !== appState.currentSystemState.ota?.message ||
        previousState.ota?.reboot_required !== appState.currentSystemState.ota?.reboot_required)
    { console.log("State Update:", appState.currentSystemState); return true; }
    return false;
}
export function updateBatteryState(voltage, percentage) { appState.currentBattery.voltage = voltage; appState.currentBattery.percentage = percentage; }
export function updateTelemetryArray(key, value) {
    if (!appState.telemetryData.hasOwnProperty(key)) { console.warn(`Skipping non-existent telemetry key: ${key}`); return; }
    const targetArray = appState.telemetryData[key];
    let validValue = null;
    if (value !== null && value !== undefined) { const parsedValue = parseFloat(value); if (!isNaN(parsedValue)) { validValue = parsedValue; } }
    if (validValue === null) { const lastVal = targetArray.length > 0 ? targetArray[targetArray.length - 1] : null; if (lastVal !== null) { validValue = lastVal; } }
    targetArray.push(validValue);
    if (targetArray.length > MAX_DATA_POINTS) { targetArray.shift(); }
}
