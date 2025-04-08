// js/main.js
"use strict";

// Import necessary modules and functions
import { appState } from './state.js';
import { assignElements, uiElements, updateStatusSectionUI, disableCommandButton } from './ui.js';
import { setupWebSocket } from './websocket.js';
import { setupJoystick } from './joystick.js';
import { setupGraphing, drawGraph } from './graph.js'; // Import drawGraph
import { createConfigForms, toggleConfigMenu, showConfigForm, showGeneralConfigForm } from './configUI.js';
import { sendCommandApi, fetchStateApi } from './api.js';
import { updateTelemetryData } from './telemetry.js';
import { DATA_FETCH_INTERVAL_MS, STATE_FETCH_INTERVAL_MS } from './constants.js';

// --- Initialization ---
document.addEventListener('DOMContentLoaded', initialize);

function initialize() {
    console.log("DOM Loaded, Initializing Application...");
    try {
        appState.telemetryKeys = Object.keys(appState.telemetryData); // Populate keys based on state definition
        assignElements(); // Assign DOM elements to uiElements cache
        setupEventListeners(); // Set up basic UI event listeners
        createConfigForms(); // Create the structure for config forms
        setupWebSocket();    // Establish WebSocket connection
        setupJoystick();     // Initialize the joystick controller
        setupGraphing();     // Prepare the graphing canvas
        startDataFetching(); // Start fetching telemetry and state
        // Load initial config form view (optional, e.g., show Angle PID by default)
        showConfigForm('pid_angle', uiElements.angleFormContainer);
        console.log("Initialization complete.");
    } catch (error) {
        console.error("Initialization failed:", error);
        alert("Application failed to initialize. Please check console for details.");
    }
}

function setupEventListeners() {
    // Note: Graph resize listener is added in graph.js's setupGraphing
    uiElements.openConfigBtn?.addEventListener('click', toggleConfigMenu);
    uiElements.angleConfigBtn?.addEventListener('click', () => showConfigForm('pid_angle', uiElements.angleFormContainer));
    uiElements.speedLeftConfigBtn?.addEventListener('click', () => showConfigForm('pid_speed_left', uiElements.speedLeftFormContainer));
    uiElements.speedRightConfigBtn?.addEventListener('click', () => showConfigForm('pid_speed_right', uiElements.speedRightFormContainer));
    uiElements.generalConfigBtn?.addEventListener('click', () => showGeneralConfigForm(uiElements.generalFormContainer));

    // Command Buttons
    uiElements.startBtn?.addEventListener('click', () => handleCommandClick('start', uiElements.startBtn));
    uiElements.stopBtn?.addEventListener('click', () => handleCommandClick('stop', uiElements.stopBtn));
    uiElements.calibrateBtn?.addEventListener('click', () => handleCommandClick('calibrate', uiElements.calibrateBtn));
    uiElements.toggleRecoveryBtn?.addEventListener('click', handleToggleRecovery);
    uiElements.toggleFallDetectBtn?.addEventListener('click', handleToggleFallDetect);
}

// --- Command Handling Wrappers ---
async function handleCommandClick(command, buttonElement) {
    disableCommandButton(buttonElement, true); // Disable button
    const success = await sendCommandApi(command);
    // Provide feedback only for critical commands, maybe via a status message area later
    if (['start', 'stop', 'calibrate'].includes(command) && success) {
         // alert(`Command '${command}' sent successfully.`); // Optional alert
    }
    // Re-enable button after a short delay, state fetch will update UI implicitly
    setTimeout(() => disableCommandButton(buttonElement, false), 500);
}

function handleToggleRecovery() {
    const command = appState.currentSystemState.auto_recovery_enabled ? 'disable_recovery' : 'enable_recovery';
    handleCommandClick(command, uiElements.toggleRecoveryBtn);
    // UI update (text, class) is handled by fetchStateApi -> updateStatusSectionUI
}

function handleToggleFallDetect() {
    const command = appState.currentSystemState.fall_detection_enabled ? 'disable_fall_detect' : 'enable_fall_detect';
    handleCommandClick(command, uiElements.toggleFallDetectBtn);
    // UI update (text, class) is handled by fetchStateApi -> updateStatusSectionUI
}


// --- Data Fetching Control ---
function startDataFetching() {
    stopDataFetching(); // Clear existing timers first
    console.log("Starting periodic data fetching...");
    // Fetch initial state and data immediately
    fetchStateApi().then(() => {
         updateStatusSectionUI(); // Update UI after first state fetch
         return updateTelemetryData(); // Fetch telemetry data
    }).then(() => {
         drawGraph(); // Draw graph after first telemetry fetch
    }).catch(error => {
         console.error("Error during initial data fetch:", error);
    });

    // Set intervals
    appState.timers.dataFetch = setInterval(updateTelemetryData, DATA_FETCH_INTERVAL_MS);
    appState.timers.stateFetch = setInterval(fetchStateApi, STATE_FETCH_INTERVAL_MS);
}

function stopDataFetching() {
    console.log("Stopping periodic data fetching...");
    clearInterval(appState.timers.dataFetch);
    clearInterval(appState.timers.stateFetch);
    appState.timers.dataFetch = null;
    appState.timers.stateFetch = null;
}

// --- Make sure timers are cleared on page unload ---
window.addEventListener('beforeunload', () => {
    stopDataFetching();
    if (appState.ws) {
        appState.ws.close(); // Cleanly close WebSocket
    }
    // Clear other timers if necessary
    clearTimeout(appState.timers.wsReconnect);
    clearInterval(appState.timers.joystickSend);
});