"use strict";

// Import necessary modules and functions
import { appState } from './state.js';
import { assignElements, uiElements, updateStatusSectionUI, disableCommandButton } from './ui.js';
import { setupWebSocket, setWebSocketDisconnectHandler, sendWebSocketMessage } from './websocket.js';
import { setupJoystick, setJoystickTransport, stopJoystickSendInterval } from './joystick.js'; // Import stop function
import { setupGraphs, drawAllGraphs } from './graph.js'; // Use new names
import { createConfigForms, toggleConfigMenu, showConfigForm, showGeneralConfigForm } from './configUI.js'; // Import showGeneralConfigForm
import { sendCommandApi, fetchConfigApi, fetchStateApi, postConfigApi, uploadOtaFirmware } from './api.js';
import { updateTelemetryData } from './telemetry.js';
import { DATA_FETCH_INTERVAL_MS, STATE_FETCH_INTERVAL_MS } from './constants.js';
import { loadPersistedLogs, startLogsPolling, stopLogsPolling, handleClearLogs, handleSaveLogsHtml, handleSaveLogsText, renderPersistedLogs, toggleLogsPanel } from './logs.js';

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

        assignElements(); // Assign static DOM elements to uiElements cache
        loadPersistedLogs();
        renderPersistedLogs();
        createConfigForms(); // Create config forms and their dynamic controls
        setupEventListeners(); // Set up basic UI event listeners
        setWebSocketDisconnectHandler(stopJoystickSendInterval);
        setJoystickTransport(sendWebSocketMessage);
        setupWebSocket();    // Establish WebSocket connection
        setupJoystick();     // Initialize the joystick controller
        setupGraphs();     // Setup multiple graphs
        startDataFetching(); // Start fetching telemetry and state
        startLogsPolling(); // Collect logs even while the panel is collapsed

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
    uiElements.tuneLeftPidBtn?.addEventListener('click', () => handleCommandClick('start_pid_tuning', uiElements.tuneLeftPidBtn, { target: 'motor_speed_left' }));
    uiElements.cancelLeftPidTuningBtn?.addEventListener('click', () => handleCommandClick('cancel_pid_tuning', uiElements.cancelLeftPidTuningBtn, { target: 'motor_speed_left' }));
    uiElements.saveLeftPidTuningBtn?.addEventListener('click', () => handleCommandClick('save_pid_tuning', uiElements.saveLeftPidTuningBtn, { target: 'motor_speed_left' }));
    uiElements.discardLeftPidTuningBtn?.addEventListener('click', () => handleCommandClick('discard_pid_tuning', uiElements.discardLeftPidTuningBtn, { target: 'motor_speed_left' }));
    uiElements.tuneRightPidBtn?.addEventListener('click', () => handleCommandClick('start_pid_tuning', uiElements.tuneRightPidBtn, { target: 'motor_speed_right' }));
    uiElements.cancelRightPidTuningBtn?.addEventListener('click', () => handleCommandClick('cancel_pid_tuning', uiElements.cancelRightPidTuningBtn, { target: 'motor_speed_right' }));
    uiElements.saveRightPidTuningBtn?.addEventListener('click', () => handleCommandClick('save_pid_tuning', uiElements.saveRightPidTuningBtn, { target: 'motor_speed_right' }));
    uiElements.discardRightPidTuningBtn?.addEventListener('click', () => handleCommandClick('discard_pid_tuning', uiElements.discardRightPidTuningBtn, { target: 'motor_speed_right' }));

    // Toggle Buttons
    uiElements.toggleFallDetectBtn?.addEventListener('click', handleToggleFallDetect);
    uiElements.toggleAutoBalancingBtn?.addEventListener('click', handleToggleAutoBalancing);
    uiElements.toggleYawControlBtn?.addEventListener('click', handleToggleYawControl);
    uiElements.toggleCriticalBatteryShutdownBtn?.addEventListener('click', handleToggleCriticalBatteryShutdown);
    uiElements.startGuidedCalibrationBtn?.addEventListener('click', () => handleCommandClick('start_guided_calibration', uiElements.startGuidedCalibrationBtn));
    uiElements.cancelGuidedCalibrationBtn?.addEventListener('click', () => handleCommandClick('cancel_guided_calibration', uiElements.cancelGuidedCalibrationBtn));
    uiElements.otaUploadBtn?.addEventListener('click', handleOtaUpload);
    uiElements.toggleLogsBtn?.addEventListener('click', toggleLogsPanel);
    uiElements.clearLogsBtn?.addEventListener('click', handleClearLogs);
    uiElements.saveLogsTextBtn?.addEventListener('click', handleSaveLogsText);
    uiElements.saveLogsHtmlBtn?.addEventListener('click', handleSaveLogsHtml);

    console.log("Event listeners setup complete.");
}


// --- Command Handling Wrappers ---
async function handleCommandClick(command, buttonElement, extraPayload = {}) {
    console.log(`--- handleCommandClick called for: ${command} ---`);
    if (!buttonElement) {
        console.error(`Button element for command '${command}' is null!`);
        return false; // Indicate failure
    }
    disableCommandButton(buttonElement, true);
    let success = false; // Default to false
    try {
        success = await sendCommandApi(command, extraPayload); // Await the API call result
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

async function handleToggleAutoBalancing() {
    console.log("--- handleToggleAutoBalancing called ---");
    // Ensure state and button exist
    if (!appState.currentSystemState) {
        console.error("Cannot toggle auto balancing: appState.currentSystemState is null.");
        return;
    }
    if (!uiElements.toggleAutoBalancingBtn) {
        console.error("Cannot toggle auto balancing: uiElements.toggleAutoBalancingBtn is null.");
        return;
    }

    const isAutoBalancingEnabled = appState.currentSystemState.auto_balancing_enabled;
    console.log(`Current auto balancing state: ${isAutoBalancingEnabled}`);

    const command = isAutoBalancingEnabled ? 'disable_auto_balancing' : 'enable_auto_balancing';

    // Proceed to send the command
    console.log(`Sending command: ${command}`);
    await handleCommandClick(command, uiElements.toggleAutoBalancingBtn);
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
    await handleCommandClick(command, uiElements.toggleFallDetectBtn);
    // UI update will be handled by the next state fetch
}

async function handleToggleYawControl() {
    console.log("--- handleToggleYawControl called ---");

    if (!uiElements.toggleYawControlBtn) {
        console.error("Cannot toggle yaw control: button is null.");
        return;
    }

    disableCommandButton(uiElements.toggleYawControlBtn, true);

    try {
        const currentConfig = appState.configDataCache || await fetchConfigApi();
        if (!currentConfig?.control) {
            alert("Cannot change yaw control because control config is unavailable.");
            return;
        }

        const isEnabled = !!currentConfig.control.yaw_control_enabled;
        const configToSend = JSON.parse(JSON.stringify(currentConfig));
        configToSend.control.yaw_control_enabled = !isEnabled;

        const saveResult = await postConfigApi(configToSend);
        if (!saveResult) {
            return;
        }

        await fetchConfigApi();
        await fetchStateApi();
        updateStatusSectionUI();
    } catch (error) {
        console.error("Error toggling yaw control:", error);
        alert(`Error toggling yaw control: ${error.message || 'Unknown error'}`);
    } finally {
        disableCommandButton(uiElements.toggleYawControlBtn, false);
    }
}

async function handleToggleCriticalBatteryShutdown() {
    console.log("--- handleToggleCriticalBatteryShutdown called ---");

    if (!uiElements.toggleCriticalBatteryShutdownBtn) {
        console.error("Cannot toggle critical battery shutdown: button is null.");
        return;
    }

    disableCommandButton(uiElements.toggleCriticalBatteryShutdownBtn, true);

    try {
        const currentConfig = appState.configDataCache || await fetchConfigApi();
        if (!currentConfig?.battery) {
            console.error("Battery config is unavailable.");
            alert("Cannot change critical battery shutdown because battery config is unavailable.");
            return;
        }

        const isEnabled = !!currentConfig.battery.critical_battery_motor_shutdown_enabled;
        const configToSend = JSON.parse(JSON.stringify(currentConfig));
        configToSend.battery.critical_battery_motor_shutdown_enabled = !isEnabled;

        console.log(`Saving critical battery shutdown = ${!isEnabled}`);
        const saveResult = await postConfigApi(configToSend);
        if (!saveResult) {
            return;
        }

        await fetchConfigApi();
        await fetchStateApi();
        updateStatusSectionUI();
    } catch (error) {
        console.error("Error toggling critical battery shutdown:", error);
        alert(`Error toggling critical battery shutdown: ${error.message || 'Unknown error'}`);
    } finally {
        disableCommandButton(uiElements.toggleCriticalBatteryShutdownBtn, false);
    }
}

async function handleOtaUpload() {
    const firmwareFile = uiElements.otaFirmwareFileInput?.files?.[0];
    const spiffsFile = uiElements.otaSpiffsFileInput?.files?.[0];
    const ota = appState.currentSystemState?.ota || {};

    if (!ota.update_allowed) {
        alert("OTA update is allowed only while the robot is IDLE.");
        return;
    }
    if (!spiffsFile || !firmwareFile) {
        alert("Select both storage.bin and firmware .bin files.");
        return;
    }
    if (spiffsFile.name && spiffsFile.name.toLowerCase() !== 'storage.bin') {
        const proceed = confirm("SPIFFS OTA expects build/storage.bin. Continue with the selected SPIFFS file?");
        if (!proceed) {
            return;
        }
    }

    disableCommandButton(uiElements.otaUploadBtn, true);
    try {
        const spiffsResult = await uploadOtaFirmware(spiffsFile, 'spiffs');
        if (!spiffsResult) {
            return;
        }

        await fetchStateApi();
        const firmwareResult = await uploadOtaFirmware(firmwareFile, 'app');
        if (firmwareResult) {
            await fetchStateApi();
            updateStatusSectionUI();

            if (uiElements.otaSpiffsFileInput) uiElements.otaSpiffsFileInput.value = '';
            if (uiElements.otaFirmwareFileInput) uiElements.otaFirmwareFileInput.value = '';

            alert(firmwareResult.reboot_required
                ? "OTA bundle uploaded successfully. The robot will now restart automatically."
                : "OTA bundle uploaded.");
        }
    } finally {
        disableCommandButton(uiElements.otaUploadBtn, false);
    }
}


// --- Data Fetching Control ---
function startDataFetching() {
    stopDataFetching(); // Clear existing timers first
    console.log("Starting periodic data fetching...");

    // Fetch initial state and data immediately, handle potential errors
    // Use Promise.all to fetch state and first telemetry data concurrently
    Promise.all([fetchConfigApi(), fetchStateApi(), updateTelemetryData()])
        .then(() => {
            console.log("Initial config, state, and telemetry fetched.");
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
    stopLogsPolling();
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
