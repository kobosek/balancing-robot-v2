import { uiElements } from './ui.js';
import { fetchConfigApi, postConfigApi } from './api.js';
import { appState } from './state.js'; // Needed for config cache access

// --- Configuration Form Creation ---

/**
 * Creates the HTML structure for all configuration forms (PIDs and General).
 */
export function createConfigForms() {
    // Clear existing content if any
    uiElements.angleConfigFormContainer.innerHTML = '<h3>Angle PID</h3>';
    uiElements.speedLeftConfigFormContainer.innerHTML = '<h3>Speed PID Left</h3>';
    uiElements.speedRightConfigFormContainer.innerHTML = '<h3>Speed PID Right</h3>';
    uiElements.yawRateConfigFormContainer.innerHTML = '<h3>Yaw Rate PID</h3>'; // <<< ADDED
    uiElements.generalConfigFormContainer.innerHTML = '<h3>General Settings</h3>';

    // Create PID forms
    uiElements.angleConfigFormContainer.appendChild(createPIDFormDiv('pid_angle', 'Angle PID'));
    uiElements.speedLeftConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_left', 'Speed L PID'));
    uiElements.speedRightConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_right', 'Speed R PID'));
    uiElements.yawRateConfigFormContainer.appendChild(createPIDFormDiv('pid_yaw_rate', 'Yaw Rate PID')); // <<< ADDED

    // Create General settings form
    createGeneralConfigForm(uiElements.generalConfigFormContainer);
    console.log("Config forms created.");
}

/**
 * Helper function to add a labeled number input field to a form group.
 * @param {HTMLElement} parent - The container element to append the field to.
 * @param {object} fieldConfig - Configuration object {id, label, step?, min?, max?}.
 * @returns {HTMLInputElement} - The created input element.
 */
function addFormField(parent, fieldConfig) {
    const group = document.createElement('div'); group.className = 'form-group';
    const lbl = document.createElement('label'); lbl.htmlFor = fieldConfig.id; lbl.textContent = fieldConfig.label + ':';
    const inp = document.createElement('input'); inp.type = 'number'; inp.id = fieldConfig.id; inp.name = fieldConfig.id;
    // Apply optional attributes
    if (fieldConfig.step) inp.step = fieldConfig.step;
    if (fieldConfig.min !== undefined) inp.min = fieldConfig.min;
    if (fieldConfig.max !== undefined) inp.max = fieldConfig.max;
    group.appendChild(lbl); group.appendChild(inp); parent.appendChild(group);
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

/**
 * Creates the form section for general configuration settings.
 * @param {HTMLElement} container - The parent element to append the form to.
 */
function createGeneralConfigForm(container) {
    const formDiv = document.createElement('div');
    // Field definitions including section and key for mapping
    const fields = [
        // Example fields - Add more as needed from ConfigData.hpp
        { label: 'Loop (ms)', id: `mainLoop_interval_ms`, step: '1', min: 1, section: 'mainLoop', key: 'interval_ms' },
        { label: 'IMU Alpha', id: `imu_comp_filter_alpha`, step: '0.001', min: 0, max: 1, section: 'imu', key: 'comp_filter_alpha'},
        { label: 'IMU Cal Samp', id: `imu_calibration_samples`, step: '100', min: 100, section: 'imu', key: 'calibration_samples'},
        { label: 'Joy Exp', id: `control_joystick_exponent`, step: '0.1', min: 0.1, section: 'control', key: 'joystick_exponent'},
        { label: 'Max Tilt (deg)', id: `control_max_target_pitch_offset_deg`, step: '0.1', min: 0, max: 20, section: 'control', key: 'max_target_pitch_offset_deg'},
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

/**
 * Toggles the visibility of the main configuration menu section.
 * Fetches config data if the menu is opened and data isn't cached.
 */
export function toggleConfigMenu() {
    const menu = uiElements.configMenu;
    if (!menu) return; // Guard against missing element

    const isHidden = menu.style.display === 'none' || menu.style.display === '';
    menu.style.display = isHidden ? 'block' : 'none';
    if (isHidden && !appState.configDataCache) {
        console.log("Config menu opened, fetching config data...");
        fetchConfigApi(); // Load if opening and not cached
    }
    // Hide all specific forms when closing the main menu
    if (!isHidden) {
        hideAllConfigForms();
    }
}

/**
 * Hides all individual configuration form containers.
 */
export function hideAllConfigForms() {
    [ uiElements.angleConfigFormContainer,
      uiElements.speedLeftConfigFormContainer,
      uiElements.speedRightConfigFormContainer,
      uiElements.yawRateConfigFormContainer, // <<< ADDED
      uiElements.generalConfigFormContainer
    ].forEach(el => { if (el) el.style.display = 'none'; });
}

/**
 * Shows a specific PID configuration form and loads its data.
 * @param {string} sectionKey - The key corresponding to the PID section in config (e.g., 'pid_angle').
 * @param {HTMLElement} container - The container element for the form.
 */
export async function showConfigForm(sectionKey, container) {
    if (!container) {
        console.error(`Container for section ${sectionKey} not found.`);
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    // Ensure the form div exists within the container (it should if createConfigForms ran)
    const formDiv = container.querySelector('div');
    if (formDiv) {
        await loadPIDConfigSection(sectionKey, formDiv);
    } else {
        console.error(`Form div not found within container for section ${sectionKey}.`);
    }
}

/**
 * Shows the general configuration form and loads its data.
 * @param {HTMLElement} container - The container element for the general config form.
 */
export async function showGeneralConfigForm(container) {
    if (!container) {
        console.error(`General config form container not found.`);
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div');
    if (formDiv) {
        await loadGeneralConfig(formDiv);
    } else {
        console.error(`Form div not found within general config container.`);
    }
}

// --- Configuration Loading/Saving Logic ---

/**
 * Loads data for a specific PID configuration section into its form fields.
 * @param {string} sectionKey - The config section key (e.g., 'pid_angle').
 * @param {HTMLElement} formElement - The div element containing the form inputs.
 */
async function loadPIDConfigSection(sectionKey, formElement) {
    if (!formElement) { console.warn(`Form element for ${sectionKey} not found.`); return; }
    const fullConfig = appState.configDataCache || await fetchConfigApi(); // Use cache or fetch
    if (!fullConfig || !fullConfig[sectionKey]) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        alert(`Failed to load config for ${sectionKey}. Please try again.`);
        return;
    }
    const configSection = fullConfig[sectionKey];
    Object.keys(configSection).forEach(configKey => {
        // Construct input ID based on convention: sectionKey + '_' + configKey
        const inputId = `${sectionKey}_${configKey}`;
        const input = formElement.querySelector(`#${inputId}`);
        if (input) {
            input.value = configSection[configKey];
        } else {
            console.warn(`Input field #${inputId} not found in form for ${sectionKey}.`);
        }
    });
}

/**
 * Loads data for the general configuration section into its form fields.
 * @param {HTMLElement} formElement - The div element containing the form inputs.
 */
async function loadGeneralConfig(formElement) {
    if (!formElement) { console.warn("General config form element not found."); return; }
    const fullConfig = appState.configDataCache || await fetchConfigApi(); // Use cache or fetch
    if (!fullConfig) {
        console.warn("Full config data not available.");
        alert("Failed to load general config. Please try again.");
        return;
    }

    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        // Extract section and key from input ID (e.g., 'mainLoop_interval_ms')
        const idParts = input.id.split('_');
        if (idParts.length < 2) {
             console.warn(`Could not parse section/key from input ID: ${input.id}`);
             return;
        }
        const sectionKey = idParts[0];
        const valueKey = idParts.slice(1).join('_'); // Handle keys with underscores

        // Check if the value exists in the fetched config
        if (fullConfig[sectionKey]?.[valueKey] !== undefined) {
            input.value = fullConfig[sectionKey][valueKey];
        } else {
            console.warn(`Value for ${sectionKey}.${valueKey} (ID: ${input.id}) not found in config data.`);
            // Optionally clear the input or leave it as is
            // input.value = '';
        }
    });
}

/**
 * Saves data from a specific PID configuration section form back to the server.
 * @param {string} sectionKey - The config section key (e.g., 'pid_angle').
 * @param {HTMLElement} formElement - The div element containing the form inputs.
 */
async function savePIDConfigSection(sectionKey, formElement) {
    const newSectionData = {};
    let isValid = true;
    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        if (!isValid) return;
        // Extract the key part from the ID (e.g., 'kp' from 'pid_angle_kp')
        const key = input.id.substring(sectionKey.length + 1);
        const value = parseFloat(input.value);
        if (isNaN(value)) {
            alert(`Invalid number entered for: ${input.previousElementSibling?.textContent || input.id}`);
            input.focus();
            isValid = false;
        } else {
            newSectionData[key] = value;
        }
    });
    if (!isValid) return;

    // Fetch the *current* full config to merge with
    const currentConfig = await fetchConfigApi(); // Ensure we have the latest
    if (!currentConfig) {
        alert("Could not load current config to merge changes. Save aborted.");
        return;
    }

    // Create the payload to send: merge new section data into current config
    const configToSend = { ...currentConfig, [sectionKey]: newSectionData };

    // Post the merged config
    const saveResult = await postConfigApi(configToSend);
    if (saveResult) {
        alert(`${sectionKey.replace(/_/g, ' ')} config saved successfully!`);
        // Optionally re-load the section to confirm? Or just rely on cache invalidation.
    } else {
        alert(`Failed to save ${sectionKey.replace(/_/g, ' ')} config.`);
    }
}

/**
 * Saves data from the general configuration form back to the server.
 * @param {HTMLElement} formElement - The div element containing the form inputs.
 * @param {Array<object>} fieldMapping - Array defining the mapping from input ID to config structure.
 */
async function saveGeneralConfig(formElement, fieldMapping) {
    let isValid = true;
    const currentConfig = await fetchConfigApi(); // Get latest config
    if (!currentConfig) {
        alert("Could not load current config to merge changes. Save aborted.");
        return;
    }
    // Create a deep copy to modify safely
    const configToSend = JSON.parse(JSON.stringify(currentConfig));

    // Iterate through the defined fields for this form
    fieldMapping.forEach(field => {
        if (!isValid) return;
        const input = formElement.querySelector(`#${field.id}`);
        if (input) {
            const value = parseFloat(input.value);
            if (isNaN(value)) {
                alert(`Invalid number entered for ${field.label}`);
                input.focus();
                isValid = false;
            } else {
                // Ensure the section exists before assigning
                if (!configToSend[field.section]) {
                     configToSend[field.section] = {}; // Create section if missing
                     console.warn(`Created missing section '${field.section}' while saving general config.`);
                }
                configToSend[field.section][field.key] = value;
            }
        } else {
            console.warn(`Input field with ID ${field.id} not found in general config form.`);
        }
    });
    if (!isValid) return;

    // Post the updated config
    const saveResult = await postConfigApi(configToSend);
    if (saveResult) {
        alert('General config saved successfully!');
    } else {
        alert('Failed to save general config.');
    }
}