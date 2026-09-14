import { imuStatusText } from './imuStatus.js';
import { appState } from './state.js';
import { GRAPH_COLORS, Y_ANGLE_RANGE_DEG, Y_EFFORT_RANGE, Y_SPEED_RANGE_DPS, Y_YAWRATE_RANGE_DPS } from './constants.js';
import { STATIC_UI_ELEMENT_IDS } from './uiElementsRegistry.js';

export const uiElements = {};

export function assignElements() {
    console.log('Assigning elements...');

    STATIC_UI_ELEMENT_IDS.forEach(id => {
        const el = document.getElementById(id);
        if (!el) {
            console.warn(`UI element with ID '${id}' NOT FOUND!`);
            uiElements[id] = null;
        } else {
            uiElements[id] = el;
        }
    });

    for (let i = 0; i < 3; i++) {
        const graphIndex = i + 1;
        if (!appState.graphs[i]) {
            appState.graphs[i] = { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null };
        }
        const graphState = appState.graphs[i];
        graphState.container = uiElements[`graphContainer${graphIndex}`];
        graphState.canvas = uiElements[`telemetryGraph${graphIndex}`];

        if (!graphState.container || !graphState.canvas) {
            console.warn(`Graph ${graphIndex} container or canvas element missing from uiElements cache.`);
            continue;
        }

        try {
            graphState.ctx = graphState.canvas.getContext('2d');
            if (!graphState.ctx) {
                throw new Error('Failed context');
            }
            graphState.legendValueElements = [];
            const legendElement = uiElements[`legend${graphIndex}`];
            if (legendElement) {
                legendElement.querySelectorAll('.legend-value').forEach(span => {
                    graphState.legendValueElements.push(span);
                });
            } else {
                console.warn(`Legend element legend${graphIndex} not found in cache.`);
            }
        } catch (error) {
            console.error(`UI Init Error Graph ${graphIndex}:`, error);
        }
    }

    if (!uiElements.joystickZone) {
        console.error('CRITICAL: Joystick Zone element missing after assignment.');
    }
    if (!uiElements.wsStatus) {
        console.error('CRITICAL: WS Status element missing after assignment.');
    }

    console.log('UI Elements Assignment finished.');
}

export function updateWsStatusUI(text, color) { if (uiElements.wsStatus) { uiElements.wsStatus.textContent = text; uiElements.wsStatus.style.color = color; } }
export function updateStatusSectionUI() { if (uiElements.imuStatusValue) uiElements.imuStatusValue.textContent = imuStatusText(appState.currentSystemState.imu); const state = appState.currentSystemState; if (uiElements.systemStateValue) uiElements.systemStateValue.textContent = state.state_name || 'UNKNOWN'; if (uiElements.systemStateId) uiElements.systemStateId.textContent = (state.state_id !== undefined && state.state_id !== null) ? state.state_id : '?'; updateStrategyUI(); updateAutoBalancingButtonUI(); updateFallDetectButtonUI(); updateYawControlButtonUI(); updateCriticalBatteryShutdownButtonUI(); updateGuidedCalibrationUI(); updateOtaUI(); updateConfigurationOperationUI(); updatePidTuningUI(); updateConfigControlsUI(); const batt = appState.currentBattery; updateBatteryUI(batt.voltage, batt.percentage); }

function strategyLabel(id) {
    return id === 'longitudinal_cascade' ? 'Longitudinal Cascade' :
        (id === 'nested_pid' ? 'Nested PID' : String(id || 'Unknown'));
}

export function updateStrategyUI() {
    const state = appState.currentSystemState || {};
    const active = state.active_balance_strategy || 'unknown';
    const configured = state.configured_balance_strategy || 'unknown';
    const knownStrategy = id => id === 'nested_pid' || id === 'longitudinal_cascade';
    const activeEl = uiElements.activeBalanceStrategyValue;
    const configuredEl = uiElements.configuredBalanceStrategyValue;
    if (activeEl) activeEl.textContent = strategyLabel(active);
    if (configuredEl) configuredEl.textContent = strategyLabel(configured);

    const select = uiElements.balanceStrategySelect;
    const stateUnknown = !knownStrategy(active) || !knownStrategy(configured);
    if (select && !select.dataset.userEditing) {
        if (knownStrategy(configured) && select.querySelector?.(`option[value="${configured}"]`)) {
            select.value = configured;
        } else {
            select.value = '';
        }
    }
    if (select) {
        select.disabled = stateUnknown;
        select.title = stateUnknown
            ? 'Strategy state is unknown or incompatible; reload or repair the configuration first.'
            : '';
    }
    const selected = select?.value || (knownStrategy(configured) ? configured : 'unknown');
    const capabilities = state.strategy_capabilities || {};
    const capability = capabilities[selected] || {};
    const capabilityEl = uiElements.balanceStrategyCapabilityValue;
    if (capabilityEl) {
        const loop = capability.loop_mode ? ` · loop ${capability.loop_mode}` : '';
        const turn = capability.turn_control_supported === false ? ' · turn unavailable' : '';
        capabilityEl.textContent = capability.can_activate
            ? `READY${loop}${turn}`
            : `${capability.reason || 'Unavailable'}${turn}`;
    }
    const draftNames = selected === 'longitudinal_cascade'
        ? ['longitudinal_cascade', 'longitudinal_pid_pitch', 'longitudinal_pid_velocity']
        : ['nested_pid', 'pid_angle', 'pid_speed_left', 'pid_speed_right', 'pid_yaw_angle', 'pid_yaw_rate'];
    let hasDraft = false;
    try {
        hasDraft = !!globalThis.sessionStorage && draftNames.some(name =>
            globalThis.sessionStorage.getItem(`balancingRobot.configDraft.${name}`));
    } catch (_) { /* session storage is optional */ }
    if (uiElements.editedBalanceStrategyValue) {
        uiElements.editedBalanceStrategyValue.textContent = hasDraft
            ? `${strategyLabel(selected)} draft (unsaved)` : 'NONE';
    }
    const applyButton = uiElements.applyBalanceStrategyBtn;
    const isIdle = state.state_name === 'IDLE';
    // An unresolved client record is replayable with the same frozen payload;
    // only the firmware's active/recovery gate blocks a new click here.
    const blocked = stateUnknown || !!state.operation_active || !!state.operation_recovery_pending || !isIdle;
    if (applyButton) {
        applyButton.disabled = blocked || !capability.can_activate || selected === active || selected === 'unknown';
        applyButton.title = blocked
            ? 'Strategy changes require IDLE and no active configuration operation.'
            : (!capability.can_activate ? (capability.reason || 'Strategy is not ready.') : 'Apply the selected strategy');
    }
    if (uiElements.strategySelectionStatus) {
        uiElements.strategySelectionStatus.textContent = stateUnknown
            ? 'Strategy state unavailable; selection is read-only until a compatible state is received.'
            : selected === active
            ? `Active: ${strategyLabel(active)}`
            : `Selected: ${strategyLabel(selected)} · ${capability.can_activate ? 'ready to apply' : (capability.reason || 'not ready')} · runtime: ${strategyLabel(active)}`;
    }
}

export function updateConfigControlsUI() {
    const state = appState.currentSystemState || {};
    const canSave = state.state_name === 'IDLE' && !state.operation_active &&
        !state.operation_recovery_pending;
    document.querySelectorAll?.('#configMenu .config-save-btn').forEach(button => {
        const formUnknown = button.closest?.('[data-config-unknown="true"]');
        button.disabled = !canSave || !!formUnknown;
        button.title = formUnknown
            ? (formUnknown.dataset.configUnknownReason || 'Configuration version is unsupported; reload or repair before saving.')
            : (canSave ? 'Save configuration' : 'Stop the robot and finish any configuration operation before saving.');
    });
}
export function updateConfigurationOperationUI() {
    const state = appState.currentSystemState || {};
    const statusEl = uiElements.configurationOperationStatus;
    const acknowledgeButton = uiElements.acknowledgeConfigOperationBtn;
    const operationId = String(state.operation_id || '0');
    const phase = String(state.operation_phase || 'idle').toUpperCase();
    const kind = String(state.operation_kind || 'configuration').toUpperCase();
    const recoveryPending = !!state.operation_recovery_pending;
    const known = !!state.operation_known;

    let text = 'IDLE';
    if (recoveryPending) {
        text = operationId !== '0'
            ? `RECOVERY REQUIRED #${operationId}`
            : 'RECOVERY REQUIRED (MANUAL)';
    } else if (known && phase !== 'IDLE') {
        text = `${kind} ${phase}${operationId !== '0' ? ` #${operationId}` : ''}`;
    }

    if (statusEl) statusEl.textContent = text;
    if (acknowledgeButton) {
        acknowledgeButton.disabled = !recoveryPending || operationId === '0';
        acknowledgeButton.title = recoveryPending
            ? 'Reconcile the interrupted configuration operation before enabling motion.'
            : 'No interrupted configuration operation requires reconciliation.';
    }
}
export function updateAutoBalancingButtonUI() { const button = uiElements.toggleAutoBalancingBtn; const statusEl = uiElements.autoBalancingStatus; const isEnabled = !!(appState.currentSystemState?.auto_balancing_enabled); if (button) { button.textContent = isEnabled ? 'Disable Auto Balancing' : 'Enable Auto Balancing'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateFallDetectButtonUI() { const button = uiElements.toggleFallDetectBtn; const statusEl = uiElements.fallDetectStatus; const isEnabled = !!(appState.currentSystemState?.fall_detection_enabled); if (button) { button.textContent = isEnabled ? 'Disable Fall Detect' : 'Enable Fall Detect'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateYawControlButtonUI() { const button = uiElements.toggleYawControlBtn; const statusEl = uiElements.yawControlStatus; const state = appState.currentSystemState || {}; const isNested = state.active_balance_strategy === 'nested_pid'; const hasStateValue = typeof state.yaw_control_enabled === 'boolean'; const isEnabled = !!state.yaw_control_enabled; if (button) { button.textContent = isNested ? (isEnabled ? 'Disable Yaw Control' : 'Enable Yaw Control') : 'Yaw Control (Nested PID only)'; button.classList.toggle('enabled', isEnabled && isNested); button.disabled = !isNested || !hasStateValue || state.state_name !== 'IDLE'; } if (statusEl) statusEl.textContent = !isNested ? 'UNAVAILABLE (LONGITUDINAL)' : (hasStateValue ? (isEnabled ? 'ENABLED' : 'DISABLED') : 'UNKNOWN'); }
export function updateCriticalBatteryShutdownButtonUI() {
    const button = uiElements.toggleCriticalBatteryShutdownBtn;
    const statusEl = uiElements.criticalBatteryShutdownStatus;
    const stateValue = appState.currentSystemState?.critical_battery_motor_shutdown_enabled;
    const hasStateValue = typeof stateValue === 'boolean';
    const isEnabled = hasStateValue ? stateValue : false;

    if (button) {
        button.textContent = hasStateValue
            ? (isEnabled ? 'Disable Crit Batt Shutdown' : 'Enable Crit Batt Shutdown')
            : 'Critical Battery Shutdown';
        button.classList.toggle('enabled', isEnabled);
        button.disabled = !hasStateValue;
    }

    if (statusEl) {
        statusEl.textContent = hasStateValue ? (isEnabled ? 'ENABLED' : 'DISABLED') : 'UNKNOWN';
    }
}
export function updateOtaUI() {
    const ota = appState.currentSystemState?.ota || {};
    const statusEl = uiElements.otaStatusValue;
    const uploadBtn = uiElements.otaUploadBtn;
    const bundleAvailable = !!ota.available && !!ota.spiffs_available;
    const updateAllowed = !!ota.update_allowed;
    const recoveryPending = !!ota.bundle_recovery_pending;
    const bundleStage = String(ota.bundle_stage || 'unknown').toUpperCase();
    const text = ota.reboot_required
        ? 'REBOOT REQUIRED'
        : (ota.update_in_progress
            ? `UPLOADING ${String(ota.active_target || '').toUpperCase()}`
            : (recoveryPending && bundleStage !== 'SPIFFS_READY'
                ? `OTA RECOVERY ${bundleStage}`
                : (!updateAllowed ? 'IDLE REQUIRED' : (bundleAvailable ? (ota.message || 'READY') : 'UNAVAILABLE'))));
    if (statusEl) statusEl.textContent = text;
    if (uploadBtn) {
        uploadBtn.textContent = 'Upload OTA Bundle';
        uploadBtn.disabled = !bundleAvailable || !updateAllowed || !!ota.update_in_progress;
    }
}
export function updateGuidedCalibrationUI() {
    const guided = appState.currentSystemState?.guided_calibration || {};
    const state = guided.state || 'IDLE';
    const phase = guided.phase || 'IDLE';
    const progress = Math.round(Math.max(0, Math.min(1, Number(guided.progress || 0))) * 100);
    const statusText = state === 'RUNNING'
        ? `${phase} ${progress}%`
        : (guided.message || state);
    if (uiElements.guidedCalibrationStatus) uiElements.guidedCalibrationStatus.textContent = statusText;
    const isIdle = appState.currentSystemState?.state_name === 'IDLE';
    const isRunning = state === 'RUNNING' || appState.currentSystemState?.state_name === 'GUIDED_CALIBRATION';
    if (uiElements.startGuidedCalibrationBtn) uiElements.startGuidedCalibrationBtn.disabled = !isIdle || isRunning;
    if (uiElements.cancelGuidedCalibrationBtn) uiElements.cancelGuidedCalibrationBtn.disabled = !isRunning;
}
export function updateBatteryUI(voltage, percentage) {
    const stateBatteryVoltage = parseFloat(appState.currentSystemState?.battery_voltage);
    const stateBatteryPercentage = parseFloat(appState.currentSystemState?.battery_percentage);
    const stateAdcPinVoltage = parseFloat(appState.currentSystemState?.battery_adc_pin_voltage);
    const adcCalibrated = !!appState.currentSystemState?.battery_adc_calibrated;

    const numericVoltage = parseFloat(voltage);
    const numericPercentage = parseFloat(percentage);
    const hasTelemetryVoltage = !isNaN(numericVoltage) && numericVoltage > 0.05;
    const effectiveVoltage = hasTelemetryVoltage ? numericVoltage : stateBatteryVoltage;
    const effectivePercentage = hasTelemetryVoltage && !isNaN(numericPercentage) ? numericPercentage : stateBatteryPercentage;
    const hasValidVoltage = !isNaN(effectiveVoltage) && effectiveVoltage >= 0.0;
    const clampedPercentage = !isNaN(effectivePercentage) ? Math.max(0, Math.min(100, Math.round(effectivePercentage))) : 0;
    const vMin = appState.configDataCache?.battery?.voltage_min ?? 3.3;
    const vMax = appState.configDataCache?.battery?.voltage_max ?? 4.2;
    const lowWarningVoltage = vMin + (vMax - vMin) * 0.1;

    const displayVoltage = hasValidVoltage ? effectiveVoltage.toFixed(2) : '?.??';
    const displayPercentage = hasValidVoltage ? clampedPercentage : '??';
    const displayAdcPinVoltage = !isNaN(stateAdcPinVoltage) && stateAdcPinVoltage >= 0.0 ? stateAdcPinVoltage.toFixed(2) : '?.??';
    let batteryStateText = 'UNKNOWN';
    let batteryStateClass = 'unknown';

    if (hasValidVoltage) {
        if (effectiveVoltage <= vMin) {
            batteryStateText = 'CRITICAL';
            batteryStateClass = 'critical';
        } else if (effectiveVoltage <= lowWarningVoltage) {
            batteryStateText = 'LOW';
            batteryStateClass = 'low';
        } else {
            batteryStateText = 'OK';
            batteryStateClass = 'ok';
        }
    }

    if (uiElements.batteryVoltageValue) uiElements.batteryVoltageValue.textContent = displayVoltage;
    if (uiElements.batteryPercentValue) uiElements.batteryPercentValue.textContent = displayPercentage;
    if (uiElements.batteryStateValue) {
        uiElements.batteryStateValue.textContent = batteryStateText;
        uiElements.batteryStateValue.className = `battery-state ${batteryStateClass}`;
    }
    if (uiElements.batteryAdcPinVoltageValue) uiElements.batteryAdcPinVoltageValue.textContent = displayAdcPinVoltage;
    if (uiElements.batteryAdcCalibrationValue) {
        uiElements.batteryAdcCalibrationValue.textContent = adcCalibrated ? 'CAL ON' : 'CAL OFF';
        uiElements.batteryAdcCalibrationValue.className = `battery-state ${adcCalibrated ? 'ok' : 'unknown'}`;
    }
    if (uiElements.batteryBarFill) {
        uiElements.batteryBarFill.style.width = `${hasValidVoltage ? clampedPercentage : 0}%`;
        uiElements.batteryBarFill.className = `battery-bar-fill ${batteryStateClass}`;
    }
}
export function updatePidTuningUI() {
    const tuning = appState.currentSystemState?.pid_tuning || {};
    const state = tuning.state || 'IDLE';
    const phase = tuning.phase || 'IDLE';
    const target = tuning.target || 'motor_speed_left';
    const progress = Math.max(0, Math.min(1, Number(tuning.progress || 0)));
    const hasCandidate = !!tuning.has_candidate;
    const saveInProgress = !!tuning.save_in_progress;
    const isRunning = state === 'RUNNING';
    const isPreviewReady = state === 'PREVIEW_READY' && hasCandidate;
    const canStart = !saveInProgress && !isRunning && !isPreviewReady &&
        appState.currentSystemState?.state_name === 'IDLE';
    const canCommit = !saveInProgress && state === 'PREVIEW_READY' && hasCandidate;
    const fmtGains = (pid) => pid
        ? `Kp ${Number(pid.kp || 0).toFixed(4)} / Ki ${Number(pid.ki || 0).toFixed(4)} / Kd ${Number(pid.kd || 0).toFixed(4)}`
        : 'N/A';
    const getEl = (id) => uiElements[id] || document.getElementById(id);
    const updateWheel = (side, label, candidateKey) => {
        const activeTarget = target === `motor_speed_${side}`;
        const prefix = side === 'left' ? 'leftPidTuning' : 'rightPidTuning';
        const buttonPrefix = side === 'left' ? 'Left' : 'Right';
        const displayedState = activeTarget || state === 'IDLE' ? state : `${label.toUpperCase()} INACTIVE`;
        const displayedPhase = activeTarget ? phase : 'IDLE';
        const displayedMessage = activeTarget ? (tuning.message || 'Idle') :
            (isRunning || isPreviewReady ? `${label} wheel tuning is not active` : 'Idle');
        const candidate = tuning.candidate?.[candidateKey];

        const stateEl = getEl(`${prefix}State`);
        const phaseEl = getEl(`${prefix}Phase`);
        const progressEl = getEl(`${prefix}ProgressBar`);
        const messageEl = getEl(`${prefix}Message`);
        const gainsEl = getEl(`${prefix}Gains`);
        if (stateEl) stateEl.textContent = displayedState;
        if (phaseEl) phaseEl.textContent = displayedPhase;
        if (progressEl) progressEl.style.width = `${activeTarget ? Math.round(progress * 100) : 0}%`;
        if (messageEl) messageEl.textContent = displayedMessage;
        if (gainsEl) gainsEl.textContent = activeTarget && hasCandidate ? fmtGains(candidate) : 'N/A';

        const tuneBtn = getEl(`tune${buttonPrefix}PidBtn`);
        const cancelBtn = getEl(`cancel${buttonPrefix}PidTuningBtn`);
        const saveBtn = getEl(`save${buttonPrefix}PidTuningBtn`);
        const discardBtn = getEl(`discard${buttonPrefix}PidTuningBtn`);
        if (tuneBtn) tuneBtn.disabled = !canStart;
        if (cancelBtn) cancelBtn.disabled = !(isRunning && activeTarget);
        if (saveBtn) saveBtn.disabled = !(canCommit && activeTarget);
        if (discardBtn) discardBtn.disabled = !(canCommit && activeTarget);
    };

    updateWheel('left', 'Left', 'speed_left');
    updateWheel('right', 'Right', 'speed_right');
}
export function disableCommandButton(buttonElement, disable = true) { if (buttonElement) buttonElement.disabled = disable; }
export function updateLegendUI(latestPointMap) {
    const formatVal = (val, digits = 1) => { const num = parseFloat(val); return isNaN(num) ? 'N/A' : num.toFixed(digits); };
    if (appState.graphs[0]?.legendValueElements?.length >= 4) { const l = appState.graphs[0].legendValueElements; l[0].textContent = formatVal(latestPointMap?.pitchDeg, 1); l[1].textContent = formatVal(latestPointMap?.targetPitchDeg, 1); l[2].textContent = formatVal(latestPointMap?.joystickX, 2); l[3].textContent = formatVal(latestPointMap?.joystickY, 2); }
    if (appState.graphs[1]?.legendValueElements?.length >= 4) { const l = appState.graphs[1].legendValueElements; l[0].textContent = formatVal(latestPointMap?.speedSetpointLDPS, 0); l[1].textContent = formatVal(latestPointMap?.speedSetpointRDPS, 0); l[2].textContent = formatVal(latestPointMap?.speedLDPS, 0); l[3].textContent = formatVal(latestPointMap?.speedRDPS, 0); }
    if (appState.graphs[2]?.legendValueElements?.length >= 4) { const l = appState.graphs[2].legendValueElements; l[0].textContent = formatVal(latestPointMap?.targetYawAngleDeg, 1); l[1].textContent = formatVal(latestPointMap?.yawAngleDeg, 1); l[2].textContent = formatVal(latestPointMap?.targetYawRateDPS, 1); l[3].textContent = formatVal(latestPointMap?.yawRateDPS, 1); }
    if (appState.graphs[3]?.legendValueElements?.length >= 6) { const l = appState.graphs[3].legendValueElements; l[0].textContent = formatVal(latestPointMap?.positionM, 3); l[1].textContent = formatVal(latestPointMap?.holdPositionM, 3); l[2].textContent = formatVal(latestPointMap?.positionErrorM, 3); l[3].textContent = formatVal(latestPointMap?.targetVelocityMps, 3); l[4].textContent = formatVal(latestPointMap?.commandVelocityMps, 3); l[5].textContent = formatVal(latestPointMap?.measuredVelocityMps, 3); }
    if (appState.graphs[4]?.legendValueElements?.length >= 7) { const l = appState.graphs[4].legendValueElements; l[0].textContent = formatVal(latestPointMap?.distanceDifferenceM, 3); l[1].textContent = formatVal(latestPointMap?.distanceDifferenceTargetM, 3); l[2].textContent = formatVal(latestPointMap?.syncVelocityDifferenceMps, 3); l[3].textContent = formatVal(latestPointMap?.requestedBalanceEffort, 3); l[4].textContent = formatVal(latestPointMap?.balanceEffort, 3); l[5].textContent = formatVal(latestPointMap?.requestedSyncEffort, 3); l[6].textContent = formatVal(latestPointMap?.syncEffort, 3); }
}

export function updateLongitudinalTelemetryUI(latestPointMap) {
    const element = uiElements.longitudinalTelemetryStatus;
    if (!element || !latestPointMap) return;
    if (latestPointMap.strategyName !== 'longitudinal_cascade') {
        element.textContent = `Inactive (Nested PID) · telemetry dropped ${appState.telemetryDroppedSamples || 0} · pending ${appState.telemetryPendingSamples || 0}`;
        return;
    }
    const phaseNames = ['INACTIVE', 'LEGACY', 'PITCH', 'CAPTURE', 'HOLD', 'DRIVE', 'BRAKE', 'FAULT'];
    const phase = phaseNames[latestPointMap.phase] || 'UNKNOWN';
    const value = (key, digits = 3) => Number.isFinite(latestPointMap[key]) ? Number(latestPointMap[key]).toFixed(digits) : 'N/A';
    const flags = [];
    const faultNames = {
        1: 'encoder',
        2: 'input',
        3: 'arm',
        4: 'odometry',
        5: 'step',
        6: 'motor commit',
        7: 'control'
    };
    const faultReason = Number(latestPointMap.faultReason);
    if (latestPointMap.motionRequestLimited) flags.push('request limited');
    if (latestPointMap.velocityAntiWindup) flags.push('anti-windup');
    if (latestPointMap.syncLimited) flags.push('sync limited');
    if (latestPointMap.phaseReason === 5 || latestPointMap.faultLatched ||
        (Number.isInteger(faultReason) && faultReason > 0)) {
        flags.push(`fault (${faultNames[faultReason] || 'unknown'})`);
    }
    element.textContent = `${phase} · v ${value('measuredVelocityMps')} / ${value('targetVelocityMps')} m/s · s ${value('positionM')} / ${value('holdPositionM')} m${flags.length ? ' · ' + flags.join(', ') : ''} · dropped ${appState.telemetryDroppedSamples || 0} · pending ${appState.telemetryPendingSamples || 0}`;
}
