// js/joystick.js
/* global nipplejs */ // Inform linter that nipplejs is a global variable
import { appState } from './state.js';
import { uiElements } from './ui.js';
import { sendWebSocketMessage } from './websocket.js';
import { JOYSTICK_SEND_INTERVAL_MS } from './constants.js';

let sendIntervalTimer = null;

export function setupJoystick() {
    if (typeof nipplejs === 'undefined' || typeof nipplejs.create !== 'function') {
        console.error("NippleJS library not loaded!");
        return;
    }
    if (!uiElements.joystickZone) {
        console.error("joystickZone element not found!");
        return;
    }

    // Defer initialization slightly
    requestAnimationFrame(() => {
        console.log("Creating joystick...");
        const options = {
            zone: uiElements.joystickZone,
            mode: 'static', position: { left: '50%', top: '50%' },
            color: '#6c757d', size: 150, threshold: 0.1,
            restJoystick: true, restOpacity: 0.5, lockX: false, lockY: false, shape: 'circle'
        };
        try {
            if (appState.joystick.instance) {
                appState.joystick.instance.destroy();
            }
            appState.joystick.instance = nipplejs.create(options);
            appState.joystick.instance.on('start', handleJoystickStart);
            appState.joystick.instance.on('move', handleJoystickMove);
            appState.joystick.instance.on('end', handleJoystickEnd);
            console.log("Joystick initialized.");
        } catch (error) {
            console.error("Failed to create joystick:", error);
        }
    });
}

function handleJoystickStart(evt, nipple) {
    appState.joystick.isActive = true;
    startJoystickSendInterval();
}

function handleJoystickMove(evt, nipple) {
    if (!nipple || typeof nipple.force !== 'number' || typeof nipple.angle?.radian !== 'number' || !nipple.options?.size) {
        return;
    }
    const maxDistance = nipple.options.size / 2;
    const force = Math.min(nipple.force, maxDistance);
    const normalizedForce = maxDistance > 0 ? (force / maxDistance) : 0;
    const angleRad = nipple.angle.radian;

    appState.joystick.currentData.y = normalizedForce * Math.sin(angleRad);
    appState.joystick.currentData.x = normalizedForce * Math.cos(angleRad);
}

function handleJoystickEnd(evt, nipple) {
    appState.joystick.isActive = false;
    appState.joystick.currentData.x = 0;
    appState.joystick.currentData.y = 0;
    sendJoystickDataIfNeeded(); // Send final zero position
    stopJoystickSendInterval();
}

function startJoystickSendInterval() {
    if (!sendIntervalTimer) {
        sendJoystickDataIfNeeded(); // Send immediately
        sendIntervalTimer = setInterval(sendJoystickDataIfNeeded, JOYSTICK_SEND_INTERVAL_MS);
        appState.timers.joystickSend = sendIntervalTimer; // Store timer ID in central state if needed
    }
}

export function stopJoystickSendInterval() { // Export if needed by websocket.js
    if (sendIntervalTimer) {
        clearInterval(sendIntervalTimer);
        sendIntervalTimer = null;
        appState.timers.joystickSend = null;
    }
}

function sendJoystickDataIfNeeded() {
    const { currentData, lastSentData, isActive } = appState.joystick;
    const dx = currentData.x - lastSentData.x;
    const dy = currentData.y - lastSentData.y;
    const threshold = 0.01;

    if ((isActive && (Math.abs(dx) > threshold || Math.abs(dy) > threshold)) ||
        (!isActive && (lastSentData.x !== 0 || lastSentData.y !== 0)))
    {
        const payload = { type: "joystick", x: currentData.x, y: currentData.y };
        if (sendWebSocketMessage(payload)) {
            // Update last sent only on successful send
            appState.joystick.lastSentData.x = currentData.x;
            appState.joystick.lastSentData.y = currentData.y;
        } else {
             console.warn("Failed to send joystick data via WebSocket.");
        }
    }
}