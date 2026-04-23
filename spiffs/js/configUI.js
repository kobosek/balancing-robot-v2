// ================================================
// File: spiffs/js/configUI.js
// ================================================
import { uiElements } from './ui.js';
import { fetchConfigApi, postConfigApi } from './api.js';
import { appState } from './state.js'; // Needed for config cache access

// --- Configuration Form Creation ---

/**
 * Creates the HTML structure for all configuration forms (PIDs and General).
 */
export function createConfigForms() {
    // Ensure UI elements are available
    if (!uiElements.angleConfigFormContainer || !uiElements.speedLeftConfigFormContainer ||
        !uiElements.speedRightConfigFormContainer || !uiElements.yawRateConfigFormContainer ||
        !uiElements.generalConfigFormContainer) {
        console.error("One or more config form containers are missing. Cannot create forms.");
        return;
    }

    // Clear existing content if any
    uiElements.angleConfigFormContainer.innerHTML = '<h3>Angle PID</h3>';
    uiElements.speedLeftConfigFormContainer.innerHTML = '<h3>Speed PID Left</h3>';
    uiElements.speedRightConfigFormContainer.innerHTML = '<h3>Speed PID Right</h3>';
    uiElements.yawRateConfigFormContainer.innerHTML = '<h3>Yaw Rate PID</h3>';
    uiElements.generalConfigFormContainer.innerHTML = '<h3>General Settings</h3>';

    // Create PID forms
    uiElements.angleConfigFormContainer.appendChild(createPIDFormDiv('pid_angle', 'Angle PID'));
    uiElements.speedLeftConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_left', 'Speed L PID'));
    uiElements.speedLeftConfigFormContainer.appendChild(createPidTuningPanel('left', 'Left Motor PID Tuning'));
    uiElements.speedRightConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_right', 'Speed R PID'));
    uiElements.speedRightConfigFormContainer.appendChild(createPidTuningPanel('right', 'Right Motor PID Tuning'));
    uiElements.yawRateConfigFormContainer.appendChild(createPIDFormDiv('pid_yaw_rate', 'Yaw Rate PID'));

    // Create General settings form
    createGeneralConfigForm(uiElements.generalConfigFormContainer);
    console.log("Config forms created.");
}

function registerDynamicElement(id, element) {
    uiElements[id] = element;
    return element;
}

/**
 * Helper function to add a labeled number input field to a form group.
 * @param {HTMLElement} parent - The container element to append the field to.
 * @param {object} fieldConfig - Configuration object {id, label, step?, min?, max?}.
 * @returns {HTMLInputElement} - The created input element.
 */
function addFormField(parent, fieldConfig) {
    const isCheckbox = fieldConfig.type === 'checkbox';
    const group = document.createElement('div');
    group.className = isCheckbox ? 'form-group form-group-checkbox' : 'form-group';

    const lbl = document.createElement('label');
    lbl.htmlFor = fieldConfig.id;
    lbl.textContent = isCheckbox ? fieldConfig.label : fieldConfig.label + ':';

    const inp = document.createElement('input');
    inp.type = isCheckbox ? 'checkbox' : 'number';
    inp.id = fieldConfig.id;
    inp.name = fieldConfig.id;
    if (fieldConfig.section) inp.dataset.section = fieldConfig.section;
    if (fieldConfig.key) inp.dataset.key = fieldConfig.key;

    if (!isCheckbox) {
        if (fieldConfig.step) inp.step = fieldConfig.step;
        if (fieldConfig.min !== undefined) inp.min = fieldConfig.min;
        if (fieldConfig.max !== undefined) inp.max = fieldConfig.max;
    }

    if (isCheckbox) {
        group.appendChild(inp);
        group.appendChild(lbl);
    } else {
        group.appendChild(lbl);
        group.appendChild(inp);
    }

    parent.appendChild(group);
    return inp;
}

/**
 * Creates a div containing a standard set of PID input fields and a save button.
 * @param {string} idPrefix - The prefix for input element IDs (e.g., 'pid_angle').
 * @param {string} [saveButtonSuffix='PID'] - Text suffix for the save button.
 * @returns {HTMLDivElement} - The created form div element.
 */
function createPIDFormDiv(idPrefix, saveButtonSuffix = 'PID') {
    const formDiv = document.createElement('div');
    // Standard PID fields
    const fields = [
        { label: 'Kp', id: `${idPrefix}_kp`, step: '0.1' },
        { label: 'Ki', id: `${idPrefix}_ki`, step: '0.01' },
        { label: 'Kd', id: `${idPrefix}_kd`, step: '0.001' },
        { label: 'Out Min', id: `${idPrefix}_output_min`, step: 'any' }, // Allow decimals for limits
        { label: 'Out Max', id: `${idPrefix}_output_max`, step: 'any' },
        { label: 'ITerm Min', id: `${idPrefix}_iterm_min`, step: 'any' },
        { label: 'ITerm Max', id: `${idPrefix}_iterm_max`, step: 'any' }
    ];
    fields.forEach(field => addFormField(formDiv, field));
    const btn = document.createElement('button'); btn.textContent = `Save ${saveButtonSuffix}`; btn.className = 'config-save-btn';
    btn.addEventListener('click', () => savePIDConfigSection(idPrefix, formDiv));
    formDiv.appendChild(btn);
    return formDiv;
}

function createPidTuningPanel(side, title) {
    const prefix = side === 'left' ? 'leftPidTuning' : 'rightPidTuning';
    const buttonPrefix = side === 'left' ? 'Left' : 'Right';
    const panel = document.createElement('div');
    panel.className = 'pid-tuning-panel config-pid-tuning-panel';

    const heading = document.createElement('h3');
    heading.textContent = title;
    panel.appendChild(heading);

    const controls = document.createElement('div');
    controls.className = 'pid-tuning-controls';

    const tuneBtn = document.createElement('button');
    tuneBtn.id = `tune${buttonPrefix}PidBtn`;
    tuneBtn.className = 'tune';
    tuneBtn.textContent = `Tune ${buttonPrefix}`;
    controls.appendChild(registerDynamicElement(tuneBtn.id, tuneBtn));

    const cancelBtn = document.createElement('button');
    cancelBtn.id = `cancel${buttonPrefix}PidTuningBtn`;
    cancelBtn.className = 'tune-cancel';
    cancelBtn.textContent = 'Cancel';
    controls.appendChild(registerDynamicElement(cancelBtn.id, cancelBtn));

    const saveBtn = document.createElement('button');
    saveBtn.id = `save${buttonPrefix}PidTuningBtn`;
    saveBtn.className = 'tune-save';
    saveBtn.textContent = 'Save Tuned PID';
    controls.appendChild(registerDynamicElement(saveBtn.id, saveBtn));

    const discardBtn = document.createElement('button');
    discardBtn.id = `discard${buttonPrefix}PidTuningBtn`;
    discardBtn.className = 'tune-discard';
    discardBtn.textContent = 'Discard';
    controls.appendChild(registerDynamicElement(discardBtn.id, discardBtn));

    panel.appendChild(controls);

    const stateItem = document.createElement('div');
    stateItem.className = 'status-item';
    stateItem.innerHTML = `<span class="status-label">Tuning:</span> <span id="${prefix}State" class="status-value">IDLE</span>`;
    panel.appendChild(stateItem);

    const phaseItem = document.createElement('div');
    phaseItem.className = 'status-item';
    phaseItem.innerHTML = `<span class="status-label">Phase:</span> <span id="${prefix}Phase" class="status-value">IDLE</span>`;
    panel.appendChild(phaseItem);

    const progress = document.createElement('div');
    progress.className = 'pid-tuning-progress';
    progress.setAttribute('aria-hidden', 'true');
    progress.innerHTML = `<div id="${prefix}ProgressBar" class="pid-tuning-progress-fill"></div>`;
    panel.appendChild(progress);

    const message = document.createElement('div');
    message.id = `${prefix}Message`;
    message.className = 'pid-tuning-message';
    message.textContent = 'Idle';
    panel.appendChild(registerDynamicElement(message.id, message));

    const gains = document.createElement('div');
    gains.className = 'pid-tuning-gains';
    gains.innerHTML = `Candidate: <span id="${prefix}Gains">N/A</span>`;
    panel.appendChild(gains);

    registerDynamicElement(`${prefix}State`, stateItem.querySelector(`#${prefix}State`));
    registerDynamicElement(`${prefix}Phase`, phaseItem.querySelector(`#${prefix}Phase`));
    registerDynamicElement(`${prefix}ProgressBar`, progress.querySelector(`#${prefix}ProgressBar`));
    registerDynamicElement(`${prefix}Gains`, gains.querySelector(`#${prefix}Gains`));

    return panel;
}

/**
 * Creates the form section for general configuration settings.
 * @param {HTMLElement} container - The parent element to append the form to.
 */
function createGeneralConfigForm(container) {
    const formDiv = document.createElement('div');
    // Field definitions including section and key for mapping
    const fields = [
        // --- mainLoop ---
        { label: 'Loop Int(ms)', id: `mainLoop_interval_ms`, step: '1', min: 1, max: 1000, section: 'mainLoop', key: 'interval_ms' },
        // --- imu ---
        { label: 'IMU Alpha', id: `imu_comp_filter_alpha`, step: '0.001', min: 0, max: 1, section: 'imu', key: 'comp_filter_alpha'},
        { label: 'IMU Cal Samp', id: `imu_calibration_samples`, step: '100', min: 10, max: 10000, section: 'imu', key: 'calibration_samples'},
        { label: 'IMU I2C Hz', id: `imu_i2c_freq_hz`, step: '1000', min: 10000, max: 400000, section: 'imu', key: 'i2c_freq_hz'},
        { label: 'IMU FIFO Thr', id: `imu_fifo_read_threshold`, step: '1', min: 1, max: 240, section: 'imu', key: 'fifo_read_threshold'},
        { label: 'IMU Gyro X Off', id: `imu_gyro_offset_x`, step: '0.001', section: 'imu', key: 'gyro_offset_x'}, // Readonly display? Needs specific handling if not editable
        { label: 'IMU Gyro Y Off', id: `imu_gyro_offset_y`, step: '0.001', section: 'imu', key: 'gyro_offset_y'},
        { label: 'IMU Gyro Z Off', id: `imu_gyro_offset_z`, step: '0.001', section: 'imu', key: 'gyro_offset_z'},
        // --- control ---
        { label: 'Joy Exp', id: `control_joystick_exponent`, step: '0.1', min: 0.1, max: 5.0, section: 'control', key: 'joystick_exponent'},
        { label: 'Max Tilt (deg)', id: `control_max_target_pitch_offset_deg`, step: '0.1', min: 0, max: 90, section: 'control', key: 'max_target_pitch_offset_deg'},
        { label: 'Yaw Control', id: `control_yaw_control_enabled`, type: 'checkbox', section: 'control', key: 'yaw_control_enabled'},
        // --- motor ---
        { label: 'M Deadzone', id: `motor_deadzone_duty`, step: '1', min: 0, section: 'motor', key: 'deadzone_duty'}, // Max depends on resolution, maybe validate on save
        // --- encoder ---
        { label: 'Enc Gear Ratio', id: `encoder_gear_ratio`, step: '0.1', min: 0.1, max: 1000, section: 'encoder', key: 'gear_ratio'},
        { label: 'Enc Whl Diam mm', id: `encoder_wheel_diameter_mm`, step: '0.1', min: 1, max: 1000, section: 'encoder', key: 'wheel_diameter_mm'},
        { label: 'Enc Filter Alpha', id: `encoder_speed_filter_alpha`, step: '0.01', min: 0, max: 1, section: 'encoder', key: 'speed_filter_alpha'},
        // --- pid_tuning ---
        { label: 'Tune Step Effort', id: `pid_tuning_step_effort`, step: '0.01', min: 0.01, max: 1, section: 'pid_tuning', key: 'step_effort'},
        { label: 'Tune Agg Effort', id: `pid_tuning_max_effort`, step: '0.01', min: 0.01, max: 1, section: 'pid_tuning', key: 'max_effort'},
        { label: 'Tune Step(ms)', id: `pid_tuning_step_duration_ms`, step: '100', min: 100, max: 10000, section: 'pid_tuning', key: 'step_duration_ms'},
        { label: 'Tune Rest(ms)', id: `pid_tuning_rest_duration_ms`, step: '100', min: 0, max: 10000, section: 'pid_tuning', key: 'rest_duration_ms'},
        { label: 'Tune Min Resp', id: `pid_tuning_min_response_dps`, step: '10', min: 1, max: 5000, section: 'pid_tuning', key: 'min_response_dps'},
        { label: 'Tune Max Speed', id: `pid_tuning_max_speed_dps`, step: '10', min: 1, max: 10000, section: 'pid_tuning', key: 'max_speed_dps'},
        { label: 'Tune Val Target', id: `pid_tuning_validation_target_dps`, step: '10', min: 1, max: 10000, section: 'pid_tuning', key: 'validation_target_dps'},
        { label: 'Tune Gain Scale', id: `pid_tuning_gain_scale`, step: '0.05', min: 0.05, max: 1, section: 'pid_tuning', key: 'gain_scale'},
        // --- battery ---
        { label: 'Batt Max V', id: `battery_voltage_max`, step: '0.01', min: 0.1, max: 100, section: 'battery', key: 'voltage_max'},
        { label: 'Batt Min V', id: `battery_voltage_min`, step: '0.01', min: 0.1, section: 'battery', key: 'voltage_min'},
        { label: 'Batt Ratio', id: `battery_voltage_divider_ratio`, step: '0.001', min: 0.1, max: 100, section: 'battery', key: 'voltage_divider_ratio'},
        { label: 'Crit Batt Shutdown', id: `battery_critical_battery_motor_shutdown_enabled`, type: 'checkbox', section: 'battery', key: 'critical_battery_motor_shutdown_enabled'},
        // --- dimensions ---
        { label: 'Wheelbase (m)', id: `dimensions_wheelbase_m`, step: '0.001', min: 0.01, max: 1.0, section: 'dimensions', key: 'wheelbase_m'},
        // --- behavior ---
        { label: 'Joy Deadzone', id: `behavior_joystick_deadzone`, step: '0.01', min: 0, max: 1, section: 'behavior', key: 'joystick_deadzone'},
        { label: 'Joy Timeout(ms)', id: `behavior_joystick_timeout_ms`, step: '10', min: 1, max: 10000, section: 'behavior', key: 'joystick_timeout_ms'},
        { label: 'Joy Max AngVel', id: `behavior_max_target_angular_velocity_dps`, step: '1', min: 1, max: 1000, section: 'behavior', key: 'max_target_angular_velocity_dps'},
        { label: 'Fall Thresh(deg)', id: `behavior_fall_pitch_threshold_deg`, step: '1', min: 10, max: 90, section: 'behavior', key: 'fall_pitch_threshold_deg'},
        { label: 'Fall Dura(ms)', id: `behavior_fall_threshold_duration_ms`, step: '10', min: 1, max: 10000, section: 'behavior', key: 'fall_threshold_duration_ms'},
        { label: 'AutoBal Thresh(deg)', id: `behavior_auto_balance_pitch_threshold_deg`, step: '0.1', min: 0, max: 30, section: 'behavior', key: 'auto_balance_pitch_threshold_deg'},
        { label: 'AutoBal Hold(ms)', id: `behavior_auto_balance_hold_duration_ms`, step: '100', min: 1, max: 60000, section: 'behavior', key: 'auto_balance_hold_duration_ms'},
        { label: 'Batt Samples', id: `behavior_battery_oversampling_count`, step: '1', min: 1, max: 1024, section: 'behavior', key: 'battery_oversampling_count'},
        { label: 'Batt Int(ms)', id: `behavior_battery_read_interval_ms`, step: '100', min: 100, max: 60000, section: 'behavior', key: 'battery_read_interval_ms'},
        { label: 'IMU I2C Fail Thr', id: `behavior_imu_health_i2c_fail_threshold`, step: '1', min: 1, max: 100, section: 'behavior', key: 'imu_health_i2c_fail_threshold'},
        { label: 'IMU NoData Thr', id: `behavior_imu_health_no_data_threshold`, step: '1', min: 1, max: 100, section: 'behavior', key: 'imu_health_no_data_threshold'},
        { label: 'IMU Timeout(ms)', id: `behavior_imu_health_data_timeout_ms`, step: '10', min: 1, max: 10000, section: 'behavior', key: 'imu_health_data_timeout_ms'},
        // --- web ---
        { label: 'Telem Buf Size', id: `web_telemetry_buffer_size`, step: '10', min: 1, max: 10000, section: 'web', key: 'telemetry_buffer_size'},
        { label: 'Max POST Size', id: `web_max_config_post_size`, step: '128', min: 512, max: 65536, section: 'web', key: 'max_config_post_size'},
    ];
    fields.forEach(field => addFormField(formDiv, field));
    const btn = document.createElement('button'); btn.textContent = 'Save General Settings'; btn.className = 'config-save-btn';
    btn.addEventListener('click', () => saveGeneralConfig(formDiv, fields)); // Pass fields mapping
    formDiv.appendChild(btn);
    container.appendChild(formDiv);
}

// --- Configuration Form Interaction ---

export function toggleConfigMenu() {
    const menu = uiElements.configMenu;
    if (!menu) return;
    const isHidden = menu.style.display === 'none' || menu.style.display === '';
    menu.style.display = isHidden ? 'block' : 'none';
    if (isHidden && !appState.configDataCache) {
        console.log("Config menu opened, fetching config data...");
        fetchConfigApi();
    }
    if (!isHidden) { hideAllConfigForms(); }
}

export function hideAllConfigForms() {
    [ uiElements.angleConfigFormContainer,
      uiElements.speedLeftConfigFormContainer,
      uiElements.speedRightConfigFormContainer,
      uiElements.yawRateConfigFormContainer,
      uiElements.generalConfigFormContainer
    ].forEach(el => { if (el) el.style.display = 'none'; });
}

export async function showConfigForm(sectionKey, container) {
    if (!container) { console.error(`Container for section ${sectionKey} not found.`); return; }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div'); // Assumes the form content is wrapped in a single div
    if (formDiv) { await loadPIDConfigSection(sectionKey, formDiv); }
    else { console.error(`Form div not found within container for section ${sectionKey}.`); }
}

export async function showGeneralConfigForm(container) {
    if (!container) { console.error(`General config form container not found.`); return; }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div'); // Assumes the form content is wrapped in a single div
    if (formDiv) { await loadGeneralConfig(formDiv); }
    else { console.error(`Form div not found within general config container.`); }
}

// --- Configuration Loading/Saving Logic ---

async function loadPIDConfigSection(sectionKey, formElement) {
    if (!formElement) { console.warn(`Form element for ${sectionKey} not found.`); return; }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig || !fullConfig[sectionKey]) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        alert(`Failed to load config for ${sectionKey}. Please refresh or check connection.`);
        return;
    }
    const configSection = fullConfig[sectionKey];
    Object.keys(configSection).forEach(configKey => {
        const inputId = `${sectionKey}_${configKey}`;
        const input = formElement.querySelector(`#${inputId}`);
        if (input) { input.value = configSection[configKey]; }
        else { console.warn(`Input field #${inputId} not found in form for ${sectionKey}.`); }
    });
}

async function loadGeneralConfig(formElement) {
    if (!formElement) { console.warn("General config form element not found."); return; }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig) {
        console.warn("Full config data not available.");
        alert("Failed to load general config. Please refresh or check connection.");
        return;
    }

    formElement.querySelectorAll('input').forEach(input => {
        const idParts = input.id.split('_');
        if (idParts.length < 2 && (!input.dataset.section || !input.dataset.key)) { console.warn(`Could not parse section/key from input ID: ${input.id}`); return; }
        const sectionKey = input.dataset.section || idParts[0];
        const valueKey = input.dataset.key || idParts.slice(1).join('_');

        // Traverse the config object using the keys
        let value = fullConfig;
        try {
             value = value[sectionKey][valueKey];
             if (value !== undefined) {
                 if (input.type === 'checkbox') { input.checked = !!value; }
                 else { input.value = value; }
             }
             else { console.warn(`Value for ${sectionKey}.${valueKey} (ID: ${input.id}) is undefined in config.`); }
        } catch (e) {
             console.warn(`Error accessing ${sectionKey}.${valueKey} (ID: ${input.id}) in config data:`, e);
        }
    });
}


async function savePIDConfigSection(sectionKey, formElement) {
    const newSectionData = {}; let isValid = true;
    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        if (!isValid) return;
        const key = input.id.substring(sectionKey.length + 1);
        const value = parseFloat(input.value);
        if (isNaN(value)) { alert(`Invalid number for: ${input.previousElementSibling?.textContent || input.id}`); input.focus(); isValid = false; }
        else { newSectionData[key] = value; }
    });
    if (!isValid) return;
    const currentConfig = await fetchConfigApi(); if (!currentConfig) { alert("Could not load current config. Save aborted."); return; }
    // Make sure to merge with the correct PID section name (already in sectionKey)
    const configToSend = { ...currentConfig, [sectionKey]: newSectionData };
    const saveResult = await postConfigApi(configToSend);
    if (saveResult) { alert(`${sectionKey.replace(/_/g, ' ')} config saved!`); }
    else { alert(`Failed to save ${sectionKey.replace(/_/g, ' ')} config.`); }
}


async function saveGeneralConfig(formElement, fieldMapping) {
    let isValid = true;
    const currentConfig = await fetchConfigApi(); if (!currentConfig) { alert("Could not load current config. Save aborted."); return; }
    const configToSend = JSON.parse(JSON.stringify(currentConfig)); // Deep copy

    fieldMapping.forEach(field => {
        if (!isValid) return;
        const input = formElement.querySelector(`#${field.id}`);
        if (input) {
            let value;
            if (input.type === 'checkbox') {
                value = input.checked;
            } else {
                value = parseFloat(input.value);
                if (isNaN(value)) { alert(`Invalid number for ${field.label}`); input.focus(); isValid = false; return; }
            }

            if (!configToSend[field.section]) { configToSend[field.section] = {}; }
            configToSend[field.section][field.key] = value;
        } else { console.warn(`Input field ID ${field.id} not found.`); }
    });
    if (!isValid) return;
    const saveResult = await postConfigApi(configToSend);
    if (saveResult) { alert('General config saved!'); }
    else { alert('Failed to save general config.'); }
}
