// js/configUI.js
import { uiElements } from './ui.js';
import { fetchConfigApi, postConfigApi } from './api.js';
import { appState } from './state.js'; // Needed for config cache access

// --- Configuration Form Creation ---

export function createConfigForms() {
    // Clear existing content if any
    uiElements.angleFormContainer.innerHTML = '<h3>Angle PID</h3>';
    uiElements.speedLeftFormContainer.innerHTML = '<h3>Speed PID Left</h3>';
    uiElements.speedRightFormContainer.innerHTML = '<h3>Speed PID Right</h3>';
    uiElements.generalFormContainer.innerHTML = '<h3>General Settings</h3>';

    // Create PID forms
    uiElements.angleFormContainer.appendChild(createPIDFormDiv('pid_angle', 'Angle PID'));
    uiElements.speedLeftFormContainer.appendChild(createPIDFormDiv('pid_speed_left', 'Speed L PID'));
    uiElements.speedRightFormContainer.appendChild(createPIDFormDiv('pid_speed_right', 'Speed R PID'));

    // Create General settings form
    createGeneralConfigForm(uiElements.generalFormContainer);
}

function addFormField(parent, fieldConfig) {
    const group = document.createElement('div'); group.className = 'form-group';
    const lbl = document.createElement('label'); lbl.htmlFor = fieldConfig.id; lbl.textContent = fieldConfig.label + ':';
    const inp = document.createElement('input'); inp.type = 'number'; inp.id = fieldConfig.id; inp.name = fieldConfig.id;
    if (fieldConfig.step) inp.step = fieldConfig.step;
    if (fieldConfig.min !== undefined) inp.min = fieldConfig.min;
    if (fieldConfig.max !== undefined) inp.max = fieldConfig.max;
    group.appendChild(lbl); group.appendChild(inp); parent.appendChild(group);
    return inp;
}

function createPIDFormDiv(idPrefix, saveButtonSuffix = 'PID') {
    const formDiv = document.createElement('div');
    const fields = [
        { label: 'Kp', id: `${idPrefix}_pid_kp`, step: '0.1' }, { label: 'Ki', id: `${idPrefix}_pid_ki`, step: '0.01' },
        { label: 'Kd', id: `${idPrefix}_pid_kd`, step: '0.001' }, { label: 'Out Min', id: `${idPrefix}_pid_output_min`, step: '1' },
        { label: 'Out Max', id: `${idPrefix}_pid_output_max`, step: '1' }, { label: 'ITerm Min', id: `${idPrefix}_pid_iterm_min`, step: '1' },
        { label: 'ITerm Max', id: `${idPrefix}_pid_iterm_max`, step: '1' }
    ];
    fields.forEach(field => addFormField(formDiv, field));
    const btn = document.createElement('button'); btn.textContent = `Save ${saveButtonSuffix}`; btn.className = 'config-save-btn';
    btn.addEventListener('click', () => savePIDConfigSection(idPrefix, formDiv));
    formDiv.appendChild(btn);
    return formDiv;
}

function createGeneralConfigForm(container) {
    const formDiv = document.createElement('div');
    // Field definitions now include section and key
    const fields = [
        { label: 'Loop (ms)', id: `mainLoop_interval_ms`, step: '1', min: 1, section: 'mainLoop', key: 'interval_ms' },
        { label: 'IMU Alpha', id: `imu_comp_filter_alpha`, step: '0.001', min: 0, max: 1, section: 'imu', key: 'comp_filter_alpha'},
        { label: 'IMU Cal Samp', id: `imu_calibration_samples`, step: '100', min: 100, section: 'imu', key: 'calibration_samples'},
        { label: 'M Deadzone', id: `motor_deadzone_duty`, step: '1', min: 0, section: 'motor', key: 'deadzone_duty'},
        { label: 'Enc Gear Ratio', id: `encoder_gear_ratio`, step: '0.1', min: 0.1, section: 'encoder', key: 'gear_ratio'},
        { label: 'Enc Whl Diam mm', id: `encoder_wheel_diameter_mm`, step: '0.1', min: 1, section: 'encoder', key: 'wheel_diameter_mm'},
        { label: 'Batt Max V', id: `battery_voltage_max`, step: '0.01', min: 0.1, section: 'battery', key: 'voltage_max'},
        { label: 'Batt Min V', id: `battery_voltage_min`, step: '0.01', min: 0.1, section: 'battery', key: 'voltage_min'}
    ];
    fields.forEach(field => addFormField(formDiv, field));
    const btn = document.createElement('button'); btn.textContent = 'Save General'; btn.className = 'config-save-btn';
    btn.addEventListener('click', () => saveGeneralConfig(formDiv, fields)); // Pass fields mapping
    formDiv.appendChild(btn);
    container.appendChild(formDiv);
}

// --- Configuration Form Interaction ---

export function toggleConfigMenu() {
    const menu = uiElements.configMenu;
    const isHidden = menu.style.display === 'none' || menu.style.display === '';
    menu.style.display = isHidden ? 'block' : 'none';
    if (isHidden && !appState.configDataCache) {
        fetchConfigApi(); // Load if opening and not cached
    }
    if (!isHidden) {
        hideAllConfigForms();
    }
}

export function hideAllConfigForms() {
    [uiElements.angleFormContainer, uiElements.speedLeftFormContainer,
     uiElements.speedRightFormContainer, uiElements.generalFormContainer]
     .forEach(el => { if (el) el.style.display = 'none'; });
}

export async function showConfigForm(sectionKey, container) {
    if (!container) return;
    hideAllConfigForms();
    container.style.display = 'block';
    await loadPIDConfigSection(sectionKey, container.querySelector('div')); // Load into the inner div
}

export async function showGeneralConfigForm(container) {
    if (!container) return;
    hideAllConfigForms();
    container.style.display = 'block';
    await loadGeneralConfig(container.querySelector('div')); // Load into the inner div
}

// --- Configuration Loading/Saving Logic ---

async function loadPIDConfigSection(sectionKey, formElement) {
    if (!formElement) { console.warn(`Form element for ${sectionKey} not found.`); return; }
    const fullConfig = await fetchConfigApi(); // Use API function
    if (!fullConfig || !fullConfig[sectionKey]) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        return;
    }
    const configSection = fullConfig[sectionKey];
    Object.keys(configSection).forEach(configKey => {
        const inputId = `${sectionKey}_${configKey}`;
        const input = formElement.querySelector(`#${inputId}`);
        if (input) input.value = configSection[configKey];
    });
}

async function loadGeneralConfig(formElement) {
    if (!formElement) { console.warn("General config form element not found."); return; }
    const fullConfig = await fetchConfigApi(); // Use API function
    if (!fullConfig) { console.warn("Full config data not available."); return; }

    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        const idParts = input.id.split('_');
        if (idParts.length < 2) return;
        const sectionKey = idParts[0]; const valueKey = idParts.slice(1).join('_');
        if (fullConfig[sectionKey]?.[valueKey] !== undefined) {
            input.value = fullConfig[sectionKey][valueKey];
        }
    });
}

async function savePIDConfigSection(sectionKey, formElement) {
    const newSectionData = {}; let isValid = true;
    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        if (!isValid) return;
        const key = input.id.substring(sectionKey.length + 1);
        const value = parseFloat(input.value);
        if (isNaN(value)) { alert(`Invalid number: ${input.id}`); input.focus(); isValid = false; }
        else { newSectionData[key] = value; }
    });
    if (!isValid) return;

    const currentConfig = await fetchConfigApi();
    if (!currentConfig) { alert("Could not load current config."); return; }
    const configToSend = { ...currentConfig, [sectionKey]: newSectionData };
    const saveResult = await postConfigApi(configToSend); // Use API function
    if (saveResult) alert(`${sectionKey.replace(/_/g, ' ')} config saved!`);
}

async function saveGeneralConfig(formElement, fieldMapping) {
    let isValid = true;
    const currentConfig = await fetchConfigApi();
    if (!currentConfig) { alert("Could not load current config."); return; }
    const configToSend = JSON.parse(JSON.stringify(currentConfig)); // Deep copy

    fieldMapping.forEach(field => {
        if (!isValid) return;
        const input = formElement.querySelector(`#${field.id}`);
        if (input) {
            const value = parseFloat(input.value);
            if (isNaN(value)) { alert(`Invalid number for ${field.label}`); input.focus(); isValid = false; }
            else if (configToSend[field.section]) { configToSend[field.section][field.key] = value; }
        }
    });
    if (!isValid) return;

    const saveResult = await postConfigApi(configToSend); // Use API function
    if (saveResult) alert('General config saved!');
}