import { API_CONFIG_URL, API_COMMAND_URL, API_STATE_URL, API_DATA_URL } from './constants.js';
import { updateConfigCache, invalidateConfigCache, appState, updateCurrentSystemState } from './state.js';
import { updateStatusSectionUI } from './ui.js'; // For state updates

// --- Config API ---
export async function fetchConfigApi() {
    console.log("Fetching config from server...");
    try {
        const response = await fetch(API_CONFIG_URL, { cache: 'no-cache' });
        if (!response.ok) throw new Error(`HTTP error ${response.status}`);
        const data = await response.json();
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
        const response = await fetch(API_CONFIG_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(configData)
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.message || `HTTP error ${response.status}`);
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
export async function sendCommandApi(commandName) {
    console.log(`Sending HTTP command: ${commandName}`);
    try {
        const response = await fetch(API_COMMAND_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: commandName })
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.message || `HTTP error ${response.status}`);
        console.log('HTTP Command response:', result);
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
        const response = await fetch(API_STATE_URL, { cache: 'no-cache' });
        if (!response.ok) throw new Error(`HTTP error ${response.status}`);
        const data = await response.json();
        if (data && typeof data === 'object') {
            updateCurrentSystemState(data); // Update the state in appState
            updateStatusSectionUI();
        } else {
            throw new Error("Invalid state data format");
        }
    } catch (error) {
        console.error('Error fetching state:', error);
        updateCurrentSystemState({ state_id: -1, name: 'ERROR', auto_recovery_enabled: false, fall_detection_enabled: false }); // Update state to ERROR
        updateStatusSectionUI(); // Update UI to show error
    }
}

// --- Telemetry Data API ---
export async function fetchDataApi() {
    try {
        const response = await fetch(API_DATA_URL, { cache: 'no-cache' });
        if (!response.ok) throw new Error(`HTTP error ${response.status}`);
        return await response.json(); // Return the raw JSON data
    } catch (error) {
        console.error('Telemetry Update Error:', error);
        return null; // Indicate failure
    }
}