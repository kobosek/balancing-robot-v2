import { fetchConfigApi, postConfigApi } from './api.js';

export async function loadPIDConfigSection(sectionKey, formElement) {
    if (!formElement) {
        console.warn(`Form element for ${sectionKey} not found.`);
        return;
    }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig || !fullConfig[sectionKey]) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        alert(`Failed to load config for ${sectionKey}. Please refresh or check connection.`);
        return;
    }

    Object.keys(fullConfig[sectionKey]).forEach(configKey => {
        const inputId = `${sectionKey}_${configKey}`;
        const input = formElement.querySelector(`#${inputId}`);
        if (input) {
            input.value = fullConfig[sectionKey][configKey];
        } else {
            console.warn(`Input field #${inputId} not found in form for ${sectionKey}.`);
        }
    });
}

export async function loadGeneralConfig(formElement) {
    if (!formElement) {
        console.warn('General config form element not found.');
        return;
    }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig) {
        console.warn('Full config data not available.');
        alert('Failed to load general config. Please refresh or check connection.');
        return;
    }

    formElement.querySelectorAll('input').forEach(input => {
        const idParts = input.id.split('_');
        if (idParts.length < 2 && (!input.dataset.section || !input.dataset.key)) {
            console.warn(`Could not parse section/key from input ID: ${input.id}`);
            return;
        }

        const sectionKey = input.dataset.section || idParts[0];
        const valueKey = input.dataset.key || idParts.slice(1).join('_');
        try {
            const value = fullConfig[sectionKey][valueKey];
            if (value !== undefined) {
                if (input.type === 'checkbox') {
                    input.checked = !!value;
                } else {
                    input.value = value;
                }
            } else {
                console.warn(`Value for ${sectionKey}.${valueKey} (ID: ${input.id}) is undefined in config.`);
            }
        } catch (error) {
            console.warn(`Error accessing ${sectionKey}.${valueKey} (ID: ${input.id}) in config data:`, error);
        }
    });
}

export async function savePIDConfigSection(sectionKey, formElement) {
    const newSectionData = {};
    let isValid = true;

    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        if (!isValid) {
            return;
        }
        const key = input.id.substring(sectionKey.length + 1);
        const value = parseFloat(input.value);
        if (isNaN(value)) {
            alert(`Invalid number for: ${input.previousElementSibling?.textContent || input.id}`);
            input.focus();
            isValid = false;
        } else {
            newSectionData[key] = value;
        }
    });

    if (!isValid) {
        return;
    }

    const currentConfig = await fetchConfigApi();
    if (!currentConfig) {
        alert('Could not load current config. Save aborted.');
        return;
    }

    const saveResult = await postConfigApi({ ...currentConfig, [sectionKey]: newSectionData });
    if (saveResult) {
        alert(`${sectionKey.replace(/_/g, ' ')} config saved!`);
    } else {
        alert(`Failed to save ${sectionKey.replace(/_/g, ' ')} config.`);
    }
}

export async function saveGeneralConfig(formElement, fieldMapping) {
    let isValid = true;
    const currentConfig = await fetchConfigApi();
    if (!currentConfig) {
        alert('Could not load current config. Save aborted.');
        return;
    }

    const configToSend = JSON.parse(JSON.stringify(currentConfig));
    fieldMapping.forEach(field => {
        if (!isValid) {
            return;
        }

        const input = formElement.querySelector(`#${field.id}`);
        if (!input) {
            console.warn(`Input field ID ${field.id} not found.`);
            return;
        }

        let value;
        if (input.type === 'checkbox') {
            value = input.checked;
        } else {
            value = parseFloat(input.value);
            if (isNaN(value)) {
                alert(`Invalid number for ${field.label}`);
                input.focus();
                isValid = false;
                return;
            }
        }

        if (!configToSend[field.section]) {
            configToSend[field.section] = {};
        }
        configToSend[field.section][field.key] = value;
    });

    if (!isValid) {
        return;
    }

    const saveResult = await postConfigApi(configToSend);
    if (saveResult) {
        alert('General config saved!');
    } else {
        alert('Failed to save general config.');
    }
}
