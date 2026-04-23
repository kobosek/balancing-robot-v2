import { uiElements } from './ui.js';
import { fetchConfigApi } from './api.js';
import { createConfigForms as renderConfigForms } from './configRenderer.js';
import { loadGeneralConfig, loadPIDConfigSection } from './configPersistence.js';
import { appState } from './state.js';

export function createConfigForms() {
    renderConfigForms();
}

export function toggleConfigMenu() {
    const menu = uiElements.configMenu;
    if (!menu) {
        return;
    }
    const isHidden = menu.style.display === 'none' || menu.style.display === '';
    menu.style.display = isHidden ? 'block' : 'none';
    if (isHidden && !appState.configDataCache) {
        console.log('Config menu opened, fetching config data...');
        fetchConfigApi();
    }
    if (!isHidden) {
        hideAllConfigForms();
    }
}

export function hideAllConfigForms() {
    [
        uiElements.angleConfigFormContainer,
        uiElements.speedLeftConfigFormContainer,
        uiElements.speedRightConfigFormContainer,
        uiElements.yawRateConfigFormContainer,
        uiElements.generalConfigFormContainer
    ].forEach(el => {
        if (el) {
            el.style.display = 'none';
        }
    });
}

export async function showConfigForm(sectionKey, container) {
    if (!container) {
        console.error(`Container for section ${sectionKey} not found.`);
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div');
    if (formDiv) {
        await loadPIDConfigSection(sectionKey, formDiv);
    } else {
        console.error(`Form div not found within container for section ${sectionKey}.`);
    }
}

export async function showGeneralConfigForm(container) {
    if (!container) {
        console.error('General config form container not found.');
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div');
    if (formDiv) {
        await loadGeneralConfig(formDiv);
    } else {
        console.error('Form div not found within general config container.');
    }
}
