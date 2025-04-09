"use strict";

// Import necessary modules and functions
import { appState } from './state.js';
import { assignElements, uiElements, updateStatusSectionUI, disableCommandButton } from './ui.js';
import { setupWebSocket } from './websocket.js';
import { setupJoystick, stopJoystickSendInterval } from './joystick.js'; // Import stop function
import { setupGraphs, drawAllGraphs } from './graph.js'; // Use new names
import { createConfigForms, toggleConfigMenu, showConfigForm, showGeneralConfigForm } from './configUI.js'; // Import showGeneralConfigForm
import { sendCommandApi, fetchStateApi } from './api.js';
import { updateTelemetryData } from './telemetry.js';
import { DATA_FETCH_INTERVAL_MS, STATE_FETCH_INTERVAL_MS } from './constants.js';

// --- Initialization ---
document.addEventListener('DOMContentLoaded', initialize);

function initialize() {
    console.log("DOM Loaded, Initializing Application...");
    try {
        // Ensure telemetryData exists before accessing keys
        if (appState.telemetryData) {
            // Populate telemetryKeys based on the defined keys in appState.telemetryData
            // This assumes state.js defines all expected keys upfront.
            appState.telemetryKeys = Object.keys(appState.telemetryData);
             console.log("Telemetry keys populated:", appState.telemetryKeys);
        } else {
             console.error("appState.telemetryData is not defined during initialization!");
             appState.telemetryKeys = []; // Initialize empty to prevent errors later
        }

        assignElements(); // Assign DOM elements to uiElements cache
        setupEventListeners(); // Set up basic UI event listeners
        createConfigForms(); // Create the structure for config forms
        setupWebSocket();    // Establish WebSocket connection
        setupJoystick();     // Initialize the joystick controller
        setupGraphs();     // Setup multiple graphs
        startDataFetching(); // Start fetching telemetry and state

        console.log("Initialization complete.");
    } catch (error) {
        console.error("Initialization failed:", error);
        alert("Application failed to initialize. Please check console for details.");
    }
}

function setupEventListeners() {
    console.log("Setting up event listeners...");

    // Config Menu
    uiElements.openConfigBtn?.addEventListener('click', toggleConfigMenu);
    uiElements.angleConfigBtn?.addEventListener('click', () => showConfigForm('pid_angle', uiElements.angleConfigFormContainer));
    uiElements.speedLeftConfigBtn?.addEventListener('click', () => showConfigForm('pid_speed_left', uiElements.speedLeftConfigFormContainer));
    uiElements.speedRightConfigBtn?.addEventListener('click', () => showConfigForm('pid_speed_right', uiElements.speedRightConfigFormContainer));
    uiElements.yawRateConfigBtn?.addEventListener('click', () => showConfigForm('pid_yaw_rate', uiElements.yawRateConfigFormContainer)); // <<< ADDED Yaw Rate
    uiElements.generalConfigBtn?.addEventListener('click', () => showGeneralConfigForm(uiElements.generalConfigFormContainer));

    // Command Buttons
    uiElements.startBtn?.addEventListener('click', () => handleCommandClick('start', uiElements.startBtn));
    uiElements.stopBtn?.addEventListener('click', () => handleCommandClick('stop', uiElements.stopBtn));
    uiElements.calibrateBtn?.addEventListener('click', () => handleCommandClick('calibrate', uiElements.calibrateBtn));

    // Toggle Buttons
    uiElements.toggleFallDetectBtn?.addEventListener('click', handleToggleFallDetect);
    uiElements.toggleRecoveryBtn?.addEventListener('click', handleToggleRecovery);

    console.log("Event listeners setup complete.");
}


// --- Command Handling Wrappers ---
async function handleCommandClick(command, buttonElement) {
    console.log(`--- handleCommandClick called for: ${command} ---`);
    if (!buttonElement) {
        console.error(`Button element for command '${command}' is null!`);
        return false; // Indicate failure
    }
    disableCommandButton(buttonElement, true);
    let success = false; // Default to false
    try {
        success = await sendCommandApi(command); // Await the API call result
        if (!success) {
             console.warn(`Command '${command}' failed or API returned false.`);
        }
    } catch (error) {
        console.error(`Error during command '${command}':`, error);
        success = false; // Ensure failure on exception
    } finally {
        // Re-enable button after a short delay, regardless of success/failure
        setTimeout(() => {
            // Check if button still exists (might be edge case if UI changes rapidly)
            if (document.body.contains(buttonElement)) {
                 disableCommandButton(buttonElement, false);
            }
        }, 500); // 500ms delay
    }
    return success; // Return the success status
}

async function handleToggleRecovery() {
    console.log("--- handleToggleRecovery called ---");
    // Ensure state and button exist
    if (!appState.currentSystemState) {
        console.error("Cannot toggle recovery: appState.currentSystemState is null.");
        return;
    }
    if (!uiElements.toggleRecoveryBtn) {
        console.error("Cannot toggle recovery: uiElements.toggleRecoveryBtn is null.");
        return;
    }

    const isRecoveryEnabled = appState.currentSystemState.auto_recovery_enabled;
    const isFallDetectEnabled = appState.currentSystemState.fall_detection_enabled;
    console.log(`Current recovery state: ${isRecoveryEnabled}, Fall detect state: ${isFallDetectEnabled}`);

    // Determine desired command
    const command = isRecoveryEnabled ? 'disable_recovery' : 'enable_recovery';

    // Prevent enabling recovery if fall detect is off
    if (command === 'enable_recovery' && !isFallDetectEnabled) {
        console.log("Cannot enable Auto Recovery because Fall Detection is disabled.");
        alert("Cannot enable Auto Recovery while Fall Detection is disabled.");
        // No command sent, ensure button is immediately re-enabled if it was somehow disabled
        disableCommandButton(uiElements.toggleRecoveryBtn, false);
        return; // Exit the function
    }

    // Proceed to send the command
    console.log(`Sending command: ${command}`);
    await handleCommandClick(command, uiElements.toggleRecoveryBtn);
    // UI update will be handled by the next state fetch
}

async function handleToggleFallDetect() {
    console.log("--- handleToggleFallDetect called ---");
     // Ensure state and button exist
    if (!appState.currentSystemState) {
        console.error("Cannot toggle fall detect: appState.currentSystemState is null.");
        return;
    }
     if (!uiElements.toggleFallDetectBtn) {
        console.error("Cannot toggle fall detect: uiElements.toggleFallDetectBtn is null.");
        return;
    }

    const isFallDetectEnabled = appState.currentSystemState.fall_detection_enabled;
    console.log(`Current fall detect state: ${isFallDetectEnabled}`);

    // Determine command
    const command = isFallDetectEnabled ? 'disable_fall_detect' : 'enable_fall_detect';
    console.log(`Sending command: ${command}`);

    // Send the primary command and wait for success/failure indication
    const success = await handleCommandClick(command, uiElements.toggleFallDetectBtn);

    // If we successfully initiated the command to DISABLE fall detect...
    if (command === 'disable_fall_detect' && success) {
        console.log("Fall detection disable command sent. Checking auto recovery...");
        // ...and if auto recovery is currently enabled (check state again just in case)...
        // Use optional chaining ?. just in case state becomes invalid between checks
        if (appState.currentSystemState?.auto_recovery_enabled === true) {
            console.log("Auto recovery is enabled, sending disable_recovery command...");
            // ...send the command to disable recovery as well.
            // We don't need to wait for this second command's result explicitly here.
            // Call sendCommandApi directly to avoid messing with the fall detect button's timeout
            sendCommandApi('disable_recovery')
                .catch(e => console.error("Error sending disable_recovery after fall detect disable:", e));
        } else {
            console.log("Auto recovery is already disabled or state unknown.");
        }
    }
    // UI update will be handled by the next state fetch
}


// --- Data Fetching Control ---
function startDataFetching() {
    stopDataFetching(); // Clear existing timers first
    console.log("Starting periodic data fetching...");

    // Fetch initial state and data immediately, handle potential errors
    // Use Promise.all to fetch state and first telemetry data concurrently
    Promise.all([fetchStateApi(), updateTelemetryData()])
        .then(() => {
            console.log("Initial state and telemetry fetched.");
            updateStatusSectionUI(); // Update UI after first state fetch
            drawAllGraphs(); // Draw graphs after initial data
        })
        .catch(error => {
            console.error("Error during initial data fetch sequence:", error);
            // Attempt to update UI even on error (to show error state)
            updateStatusSectionUI();
            console.warn("Attempting to draw graphs even after initial fetch error.");
            drawAllGraphs(); // Attempt to draw with default/empty data
        })
        .finally(() => {
            // Set intervals regardless of initial fetch success/failure
            if (appState.timers.dataFetch === null) {
                appState.timers.dataFetch = setInterval(updateTelemetryData, DATA_FETCH_INTERVAL_MS);
                console.log(`Set data fetch interval (${DATA_FETCH_INTERVAL_MS}ms), Timer ID: ${appState.timers.dataFetch}`);
            }
            if (appState.timers.stateFetch === null) {
                appState.timers.stateFetch = setInterval(fetchStateApi, STATE_FETCH_INTERVAL_MS);
                console.log(`Set state fetch interval (${STATE_FETCH_INTERVAL_MS}ms), Timer ID: ${appState.timers.stateFetch}`);
            }
        });
}

function stopDataFetching() {
    console.log("Stopping periodic data fetching...");
    if (appState.timers.dataFetch !== null) {
        clearInterval(appState.timers.dataFetch);
        console.log("Cleared data fetch timer:", appState.timers.dataFetch);
        appState.timers.dataFetch = null;
    }
    if (appState.timers.stateFetch !== null) {
        clearInterval(appState.timers.stateFetch);
        console.log("Cleared state fetch timer:", appState.timers.stateFetch);
        appState.timers.stateFetch = null;
    }
}

// --- Page Unload Cleanup ---
window.addEventListener('beforeunload', () => {
    console.log("beforeunload event triggered - cleaning up.");
    stopDataFetching();
    if (appState.ws) {
        console.log("Closing WebSocket connection.");
        appState.ws.onclose = null; // Prevent reconnection attempts
        appState.ws.close();
        appState.ws = null;
    }
    if (appState.timers.wsReconnect) {
        clearTimeout(appState.timers.wsReconnect);
        console.log("Cleared WS reconnect timer:", appState.timers.wsReconnect);
        appState.timers.wsReconnect = null;
    }
    // Explicitly stop joystick sending interval if it exists
    if (appState.timers.joystickSend !== null) {
         try {
            stopJoystickSendInterval(); // Call the exported function
            console.log("Called stopJoystickSendInterval from main.js cleanup.");
         } catch (e) {
             console.warn("Could not call stopJoystickSendInterval:", e);
         }
         // Explicitly clear the timer ID in appState as well, belt-and-suspenders
         appState.timers.joystickSend = null;
    }
    // Destroy joystick instance if it exists
    if (appState.joystick.instance) {
        try {
            appState.joystick.instance.destroy();
            console.log("Destroyed joystick instance.");
            appState.joystick.instance = null;
        } catch(e) { console.warn("Error destroying joystick instance:", e); }
    }
});