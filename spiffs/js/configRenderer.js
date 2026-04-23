import { uiElements } from './ui.js';
import { GENERAL_CONFIG_FIELDS, PID_FIELDS } from './configSchema.js';
import { saveGeneralConfig, savePIDConfigSection } from './configPersistence.js';

function registerDynamicElement(id, element) {
    uiElements[id] = element;
    return element;
}

function addFormField(parent, fieldConfig) {
    const isCheckbox = fieldConfig.type === 'checkbox';
    const group = document.createElement('div');
    group.className = isCheckbox ? 'form-group form-group-checkbox' : 'form-group';

    const label = document.createElement('label');
    label.htmlFor = fieldConfig.id;
    label.textContent = isCheckbox ? fieldConfig.label : `${fieldConfig.label}:`;

    const input = document.createElement('input');
    input.type = isCheckbox ? 'checkbox' : 'number';
    input.id = fieldConfig.id;
    input.name = fieldConfig.id;
    if (fieldConfig.section) {
        input.dataset.section = fieldConfig.section;
    }
    if (fieldConfig.key) {
        input.dataset.key = fieldConfig.key;
    }

    if (!isCheckbox) {
        if (fieldConfig.step) {
            input.step = fieldConfig.step;
        }
        if (fieldConfig.min !== undefined) {
            input.min = fieldConfig.min;
        }
        if (fieldConfig.max !== undefined) {
            input.max = fieldConfig.max;
        }
    }

    if (isCheckbox) {
        group.appendChild(input);
        group.appendChild(label);
    } else {
        group.appendChild(label);
        group.appendChild(input);
    }

    parent.appendChild(group);
}

function createPIDFormDiv(idPrefix, saveButtonSuffix = 'PID') {
    const formDiv = document.createElement('div');
    PID_FIELDS.forEach(field => {
        addFormField(formDiv, {
            label: field.label,
            id: `${idPrefix}_${field.suffix}`,
            step: field.step
        });
    });

    const button = document.createElement('button');
    button.textContent = `Save ${saveButtonSuffix}`;
    button.className = 'config-save-btn';
    button.addEventListener('click', () => savePIDConfigSection(idPrefix, formDiv));
    formDiv.appendChild(button);
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

function createGeneralConfigForm(container) {
    const formDiv = document.createElement('div');
    GENERAL_CONFIG_FIELDS.forEach(field => addFormField(formDiv, field));

    const button = document.createElement('button');
    button.textContent = 'Save General Settings';
    button.className = 'config-save-btn';
    button.addEventListener('click', () => saveGeneralConfig(formDiv, GENERAL_CONFIG_FIELDS));
    formDiv.appendChild(button);
    container.appendChild(formDiv);
}

export function createConfigForms() {
    if (!uiElements.angleConfigFormContainer || !uiElements.speedLeftConfigFormContainer ||
        !uiElements.speedRightConfigFormContainer || !uiElements.yawRateConfigFormContainer ||
        !uiElements.generalConfigFormContainer) {
        console.error('One or more config form containers are missing. Cannot create forms.');
        return;
    }

    uiElements.angleConfigFormContainer.innerHTML = '<h3>Angle PID</h3>';
    uiElements.speedLeftConfigFormContainer.innerHTML = '<h3>Speed PID Left</h3>';
    uiElements.speedRightConfigFormContainer.innerHTML = '<h3>Speed PID Right</h3>';
    uiElements.yawRateConfigFormContainer.innerHTML = '<h3>Yaw Rate PID</h3>';
    uiElements.generalConfigFormContainer.innerHTML = '<h3>General Settings</h3>';

    uiElements.angleConfigFormContainer.appendChild(createPIDFormDiv('pid_angle', 'Angle PID'));
    uiElements.speedLeftConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_left', 'Speed L PID'));
    uiElements.speedLeftConfigFormContainer.appendChild(createPidTuningPanel('left', 'Left Motor PID Tuning'));
    uiElements.speedRightConfigFormContainer.appendChild(createPIDFormDiv('pid_speed_right', 'Speed R PID'));
    uiElements.speedRightConfigFormContainer.appendChild(createPidTuningPanel('right', 'Right Motor PID Tuning'));
    uiElements.yawRateConfigFormContainer.appendChild(createPIDFormDiv('pid_yaw_rate', 'Yaw Rate PID'));
    createGeneralConfigForm(uiElements.generalConfigFormContainer);
    console.log('Config forms created.');
}
