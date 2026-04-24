import { API_CONFIG_URL, API_COMMAND_URL, API_STATE_URL, API_DATA_URL, API_OTA_URL, API_LOGS_URL } from './constants.js';
import { updateConfigCache, invalidateConfigCache, updateCurrentSystemState } from './state.js';
import { updateStatusSectionUI } from './ui.js'; // For state updates

async function parseJsonOrText(response) {
    const text = await response.text();
    if (!text) return {};
    try {
        return JSON.parse(text);
    } catch (error) {
        return { message: text };
    }
}

async function apiCall(url, options = {}) {
    const response = await fetch(url, options);
    const result = await parseJsonOrText(response);
    if (!response.ok) {
        const message = result && typeof result === 'object' ? result.message : null;
        throw new Error(message || `HTTP error ${response.status}`);
    }
    return result;
}

// --- Config API ---
export async function fetchConfigApi() {
    console.log("Fetching config from server...");
    try {
        const data = await apiCall(API_CONFIG_URL, { cache: 'no-cache' });
        updateConfigCache(data); // Update cache in state.js
        return data;
    } catch (error) {
        console.error("Failed to fetch config:", error);
        alert("Error loading configuration from robot.");
        return null;
    }
}

export async function postConfigApi(configData) {
    console.log("Posting updated config...");
    try {
        const result = await apiCall(API_CONFIG_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(configData)
        });
        console.log("Config saved successfully:", result);
        invalidateConfigCache(); // Invalidate cache via state.js
        return result;
    } catch (error) {
        console.error("Failed to save config:", error);
        alert(`Error saving configuration: ${error.message || 'Unknown error'}`);
        return null;
    }
}

// --- Command API ---
export async function sendCommandApi(commandName, extraPayload = {}) {
    console.log(`Sending HTTP command: ${commandName}`);
    const configChangingCommands = new Set([
        'start_pid_tuning',
        'cancel_pid_tuning',
        'save_pid_tuning',
        'discard_pid_tuning'
    ]);
    try {
        const result = await apiCall(API_COMMAND_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: commandName, ...extraPayload })
        });
        console.log('HTTP Command response:', result);
        if (configChangingCommands.has(commandName)) {
            invalidateConfigCache();
        }
        // Optionally provide feedback based on commandName
        // Fetch state soon after sending a command to update UI faster
        setTimeout(fetchStateApi, 300);
        return true;
    } catch (error) {
        console.error(`Error sending HTTP command '${commandName}':`, error);
        alert(`Error sending command: ${error.message || 'Unknown error'}`);
        return false;
    }
}

// --- State API ---
export async function fetchStateApi() {
    try {
        const data = await apiCall(API_STATE_URL, { cache: 'no-cache' });
        if (data && typeof data === 'object') {
            updateCurrentSystemState(data); // Update the state in appState
            updateStatusSectionUI();
            return data;
        } else {
            throw new Error("Invalid state data format");
        }
    } catch (error) {
        console.error('Error fetching state:', error);
        updateCurrentSystemState({
            state_id: -1,
            name: 'ERROR',
            state_name: 'ERROR',
            auto_balancing_enabled: false,
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
                message: 'State unavailable',
                has_candidate: false
            },
            guided_calibration: {
                state: 'IDLE',
                phase: 'IDLE',
                progress: 0,
                message: 'State unavailable'
            },
            ota: {
                available: false,
                spiffs_available: false,
                update_allowed: false,
                update_in_progress: false,
                reboot_required: false,
                active_target: 'none',
                message: 'State unavailable'
            }
        });
        updateStatusSectionUI(); // Update UI to show error
        return null;
    }
}

// --- Telemetry Data API ---
export async function fetchDataApi() {
    try {
        return await apiCall(API_DATA_URL, { cache: 'no-cache' }); // Return the raw JSON data
    } catch (error) {
        console.error('Telemetry Update Error:', error);
        return null; // Indicate failure
    }
}

export async function uploadOtaImage(file, target) {
    if (!file) {
        alert("Select a .bin file first.");
        return null;
    }

    try {
        const uploadUrl = `${API_OTA_URL}?target=${encodeURIComponent(target)}`;
        const result = await apiCall(uploadUrl, {
            method: 'POST',
            headers: { 'Content-Type': 'application/octet-stream' },
            body: file
        });
        setTimeout(fetchStateApi, 300);
        return result;
    } catch (error) {
        console.error("OTA upload failed:", error);
        alert(`OTA upload failed: ${error.message || 'Unknown error'}`);
        return null;
    }
}

export const uploadOtaFirmware = uploadOtaImage;

export async function fetchLogsApi(sinceSequence = 0) {
    try {
        const url = `${API_LOGS_URL}?since=${encodeURIComponent(sinceSequence || 0)}`;
        return await apiCall(url, { cache: 'no-cache' });
    } catch (error) {
        console.error('Error fetching logs:', error);
        return null;
    }
}

export async function clearLogsApi() {
    try {
        return await apiCall(API_LOGS_URL, { method: 'DELETE' });
    } catch (error) {
        console.error('Error clearing logs:', error);
        alert(`Error clearing logs: ${error.message || 'Unknown error'}`);
        return null;
    }
}
