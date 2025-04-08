// js/ui.js
import { appState } from './state.js';
import { GRAPH_COLORS } from './constants.js'; // Assuming constants.js

// --- DOM Elements Cache ---
export const uiElements = {};

// --- UI Initialization ---
export function assignElements() {
    const ids = [
        'openConfigBtn', 'configMenu', 'telemetryGraph', 'angleConfigBtn',
        'speedLeftConfigBtn', 'speedRightConfigBtn', 'generalConfigBtn',
        'angleConfigFormContainer', 'speedLeftConfigFormContainer',
        'speedRightConfigFormContainer', 'generalConfigFormContainer',
        'systemStateValue', 'systemStateId', 'batteryVoltageValue',
        'batteryPercentValue', 'autoRecoveryStatus', 'fallDetectStatus',
        'startBtn', 'stopBtn', 'calibrateBtn', 'toggleRecoveryBtn',
        'toggleFallDetectBtn', 'joystickZone', 'wsStatus', 'legend'
    ];
    ids.forEach(id => uiElements[id] = document.getElementById(id));

    appState.graph.container = document.querySelector('.graph-container');
    appState.graph.canvas = uiElements.telemetryGraph;

    // Validate essential elements
    if (!appState.graph.container || !appState.graph.canvas || !uiElements.joystickZone || !uiElements.wsStatus) {
        throw new Error("CRITICAL ERROR: Essential UI element missing. Check IDs/Selectors.");
    }

    try {
        appState.graph.ctx = appState.graph.canvas.getContext('2d');
        if (!appState.graph.ctx) throw new Error("Failed to get 2D context");

        appState.graph.legendValues = [];
        for (let i = 0; i < GRAPH_COLORS.length; i++) {
            const el = document.getElementById(`telemetryValue${i}`);
            if (!el) throw new Error(`Legend value element telemetryValue${i} not found`);
            appState.graph.legendValues.push(el);
        }
    } catch (error) {
        console.error("UI Initialization Error:", error);
        alert("Page Error: " + error.message);
        throw error; // Re-throw to stop further execution if critical
    }
    console.log("UI Elements Assigned.");
}

// --- UI Update Functions ---

export function updateWsStatusUI(text, color) {
    if (uiElements.wsStatus) {
        uiElements.wsStatus.textContent = text;
        uiElements.wsStatus.style.color = color;
    }
}

export function updateStatusSectionUI() {
    const state = appState.currentSystemState;
    if (uiElements.systemStateValue) uiElements.systemStateValue.textContent = state.name;
    if (uiElements.systemStateId) uiElements.systemStateId.textContent = state.id;

    updateRecoveryButtonUI();
    updateFallDetectButtonUI();

    const batt = appState.currentBattery;
    if (uiElements.batteryVoltageValue) uiElements.batteryVoltageValue.textContent = batt.voltage.toFixed(2);
    if (uiElements.batteryPercentValue) uiElements.batteryPercentValue.textContent = batt.percentage;
}

export function updateRecoveryButtonUI() {
    const button = uiElements.toggleRecoveryBtn;
    const statusEl = uiElements.autoRecoveryStatus;
    if (!button || !statusEl) return;

    const isEnabled = appState.currentSystemState.auto_recovery_enabled;
    button.textContent = isEnabled ? 'Disable Auto Recovery' : 'Enable Auto Recovery';
    statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED';
    button.classList.toggle('enabled', isEnabled);
    button.disabled = false; // Ensure it's re-enabled
}

export function updateFallDetectButtonUI() {
    const button = uiElements.toggleFallDetectBtn;
    const statusEl = uiElements.fallDetectStatus;
    if (!button || !statusEl) return;

    const isEnabled = appState.currentSystemState.fall_detection_enabled;
    button.textContent = isEnabled ? 'Disable Fall Detect' : 'Enable Fall Detect';
    statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED';
    button.classList.toggle('enabled', isEnabled);
    button.disabled = false; // Ensure it's re-enabled
}

export function updateLegendUI(latestPoint) {
    if (!latestPoint || !Array.isArray(latestPoint)) return;

    const values = latestPoint.map(v => parseFloat(v)); // Parse all safely

    if (appState.graph.legendValues.length >= 7) {
        const formatVal = (val, digits = 1) => isNaN(val) ? 'N/A' : val.toFixed(digits);
        appState.graph.legendValues[0].textContent = formatVal(values[0]);      // Pitch
        appState.graph.legendValues[1].textContent = formatVal(values[1]);      // Bal Spd
        appState.graph.legendValues[2].textContent = formatVal(values[2]);      // Speed L
        appState.graph.legendValues[3].textContent = formatVal(values[3]);      // Speed R
        appState.graph.legendValues[4].textContent = formatVal(values[4], 2);   // Effort L
        appState.graph.legendValues[5].textContent = formatVal(values[5], 2);   // Effort R
        appState.graph.legendValues[6].textContent = formatVal(values[6]);      // Targ AngVel
    }
}

export function updateBatteryUI(voltage, percentage) {
     if (uiElements.batteryVoltageValue) uiElements.batteryVoltageValue.textContent = voltage.toFixed(2);
     if (uiElements.batteryPercentValue) uiElements.batteryPercentValue.textContent = percentage;
}

export function disableCommandButton(buttonElement, disable = true) {
    if (buttonElement) {
        buttonElement.disabled = disable;
    }
}