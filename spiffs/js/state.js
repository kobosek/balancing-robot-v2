// js/state.js
import { MAX_DATA_POINTS } from './constants.js'; // Assuming constants.js exists

// --- Application State ---
// Note: telemetryData is initialized here but primarily modified by telemetry.js
export const appState = {
    telemetryData: {
        pitchDeg: Array(MAX_DATA_POINTS).fill(null),
        balanceSpeedDPS: Array(MAX_DATA_POINTS).fill(null),
        speedLDPS: Array(MAX_DATA_POINTS).fill(null),
        speedRDPS: Array(MAX_DATA_POINTS).fill(null),
        effortL: Array(MAX_DATA_POINTS).fill(null),
        effortR: Array(MAX_DATA_POINTS).fill(null),
        targetAngVelDPS: Array(MAX_DATA_POINTS).fill(null),
    },
    telemetryKeys: [], // Populated in main.js after initialization
    currentSystemState: { id: -1, name: 'UNKNOWN', auto_recovery_enabled: false, fall_detection_enabled: false },
    currentBattery: { voltage: 0, percentage: 0 },
    configDataCache: null,
    telemetryJsonCache: null, // Stores last received raw telemetry JSON from /data
    timers: {
        dataFetch: null,
        stateFetch: null,
        wsReconnect: null,
        joystickSend: null,
    },
    ws: null, // WebSocket instance managed by websocket.js
    joystick: { // Joystick state managed by joystick.js
        instance: null,
        currentData: { x: 0, y: 0 },
        lastSentData: { x: -99, y: -99 }, // Force initial send
        isActive: false,
    },
    graph: { // Graph state managed by graph.js
        ctx: null,
        canvas: null,
        container: null,
        legendValues: [],
        dpr: 1,
    }
};

// --- State Modification Functions ---

export function updateConfigCache(data) {
    appState.configDataCache = data;
    console.log("Config cache updated.");
}

export function invalidateConfigCache() {
    appState.configDataCache = null;
    console.log("Config cache invalidated.");
}

export function updateTelemetryJsonCache(data) {
    appState.telemetryJsonCache = data;
}

// Example: Update system state safely
export function updateCurrentSystemState(newStateData) {
     // Only update if state actually changed
    if (appState.currentSystemState.id !== newStateData.state_id ||
        appState.currentSystemState.auto_recovery_enabled !== newStateData.auto_recovery_enabled ||
        appState.currentSystemState.fall_detection_enabled !== newStateData.fall_detection_enabled) {
        console.log("State Update:", newStateData);
        appState.currentSystemState = { ...appState.currentSystemState, ...newStateData };
        return true; // Indicate that state changed
    }
    return false; // Indicate no change
}

// Function to update battery state
export function updateBatteryState(voltage, percentage) {
    appState.currentBattery.voltage = voltage;
    appState.currentBattery.percentage = percentage;
}

// Function to shift and push data, maintaining size limit
export function updateTelemetryArray(key, value) {
    if (!appState.telemetryData[key]) return; // Guard against invalid key

    let validValue = null;
     if (value !== null && value !== undefined && !isNaN(parseFloat(value))) {
        validValue = parseFloat(value);
    } else {
        // Handle invalid data: push null or previous value
        const lastVal = appState.telemetryData[key].length > 0 ? appState.telemetryData[key][appState.telemetryData[key].length - 1] : null;
        validValue = lastVal; // Push previous value if current is invalid
        // console.warn(`Parsed invalid value for ${key}, pushing ${validValue}. Raw:`, value);
    }

    appState.telemetryData[key].push(validValue);
    if (appState.telemetryData[key].length > MAX_DATA_POINTS) {
        appState.telemetryData[key].shift();
    }
}