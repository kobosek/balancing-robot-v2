import { appState } from './state.js';
import { uiElements } from './ui.js';
import { JOYSTICK_SEND_INTERVAL_MS } from './constants.js';

let sendIntervalTimer = null;
let sendJoystickPayload = null;

export function setJoystickTransport(sender) {
    sendJoystickPayload = typeof sender === 'function' ? sender : null;
}

export function setupJoystick() {
    if (typeof nipplejs === 'undefined' || typeof nipplejs.create !== 'function') {
        console.error("NippleJS library not loaded!");
        return;
    }
    // Ensure uiElements.joystickZone is available. It should be if assignElements ran.
    if (!uiElements.joystickZone) {
        console.error("setupJoystick called before uiElements.joystickZone was assigned!");
        // Attempt to find it again, but this indicates an initialization order issue
        uiElements.joystickZone = document.getElementById('joystickZone');
        if (!uiElements.joystickZone) {
             console.error("FATAL: joystickZone element not found even on second attempt!");
             return;
        }
    }

    // Defer initialization slightly
    requestAnimationFrame(() => {
        console.log("Creating joystick...");
        // --- MODIFIED: Restore some core options ---
        const options = {
            zone: uiElements.joystickZone,
            mode: 'static',                      // *** RESTORED ***
            position: { left: '50%', top: '50%' }, // *** RESTORED ***
            color: 'blue',                       // Keep blue for visibility
            size: 150, // Increased size slightly for better control area
            threshold: 0.05, // Lower threshold slightly for sensitivity check
            restJoystick: true, // *** RESTORED *** - Ensures nipple returns visually
            restOpacity: 0.5,   // *** RESTORED ***
             lockX: false,
             lockY: false,
             // shape: 'circle', // Default
        };
        console.log("Using semi-restored NippleJS options:", options);
        // --- END MODIFICATION ---
        try {
            if (appState.joystick.instance) {
                console.log("Destroying previous joystick instance.");
                appState.joystick.instance.destroy();
            }
            appState.joystick.instance = nipplejs.create(options);
            // Attach NippleJS specific handlers
            appState.joystick.instance.on('start', handleJoystickStart);
            appState.joystick.instance.on('move', handleJoystickMove);
            appState.joystick.instance.on('end', handleJoystickEnd);
            console.log("NippleJS Joystick initialized and handlers attached.");

        } catch (error) {
            console.error("Failed to create NippleJS joystick:", error);
        }
    });
}

// NippleJS Event Handlers
function handleJoystickStart(evt, nipple) {
    console.log("[Joystick] NippleJS Event: start"); // Log start event
    appState.joystick.isActive = true;
    startJoystickSendInterval();
}

function handleJoystickMove(evt, nipple) {
    // console.log("[Joystick] NippleJS Event: move detected"); // Keep suppressed if too noisy

    // --- REVISED CHECK ---
    if (!nipple ||
        typeof nipple.force !== 'number' ||
        typeof nipple.angle?.radian !== 'number' ||
        !nipple.instance?.options?.size)
    {
        console.warn("[Joystick] NippleJS Move event skipped: Invalid or incomplete nipple data structure.", nipple);
        return; // Exit if data is invalid
    }
    // --- END REVISED CHECK ---

    // Access size via instance.options
    const maxDistance = nipple.instance.options.size / 2;

    // Recalculate normalizedForce using distance and maxDistance
    // Ensure distance is also valid number before using
    const currentDistance = typeof nipple.distance === 'number' ? nipple.distance : 0;
    const normalizedForce = maxDistance > 0 ? Math.min(currentDistance / maxDistance, 1.0) : 0; // Normalize distance, cap at 1.0

    const angleRad = nipple.angle.radian;

    const rawY = normalizedForce * Math.sin(angleRad);
    const rawX = normalizedForce * Math.cos(angleRad);

    // Update the state
    appState.joystick.currentData.y = rawY;
    appState.joystick.currentData.x = rawX;
}

function handleJoystickEnd(evt, nipple) {
    console.log("[Joystick] NippleJS Event: end"); // Log end event
    appState.joystick.isActive = false;
    appState.joystick.currentData.x = 0;
    appState.joystick.currentData.y = 0;
    sendJoystickDataIfNeeded(); // Send final zero position IMMEDIATELY
    stopJoystickSendInterval(); // Stop the interval timer
}

// Interval timer logic for sending data
function startJoystickSendInterval() {
    if (!sendIntervalTimer) {
        sendJoystickDataIfNeeded(); // Send immediately on start
        sendIntervalTimer = setInterval(sendJoystickDataIfNeeded, JOYSTICK_SEND_INTERVAL_MS);
        appState.timers.joystickSend = sendIntervalTimer; // Track timer ID
         console.log("[Joystick] Started send interval:", sendIntervalTimer);
    }
}

export function stopJoystickSendInterval() { // Exported for use in websocket.js and main.js cleanup
    if (sendIntervalTimer) {
        clearInterval(sendIntervalTimer);
         console.log("[Joystick] Stopped send interval:", sendIntervalTimer);
        sendIntervalTimer = null;
        // Also clear from central state tracking
        if (appState.timers.joystickSend) {
             appState.timers.joystickSend = null;
        }
    }
}


// --- *** MODIFIED sendJoystickDataIfNeeded Logic *** ---
function sendJoystickDataIfNeeded() {
    // Ensure joystick state is valid before accessing
    if (!appState.joystick || !appState.joystick.currentData || !appState.joystick.lastSentData) {
        // console.warn("[Joystick] sendJoystickDataIfNeeded called before joystick state is ready.");
        return;
    }

    const { currentData, lastSentData, isActive } = appState.joystick;

    // Ensure currentData has valid numbers before sending
    const sendX = typeof currentData.x === 'number' ? currentData.x : 0;
    const sendY = typeof currentData.y === 'number' ? currentData.y : 0;
    const lastX = typeof lastSentData.x === 'number' ? lastSentData.x : 0; // Use 0 if invalid
    const lastY = typeof lastSentData.y === 'number' ? lastSentData.y : 0; // Use 0 if invalid

    let shouldSend = false;

    if (isActive) {
        // If joystick is active, *always* send the current position.
        // The backend (CommandProcessor) will handle filtering based on change.
        // We just need to keep the backend's timeout from triggering.
        shouldSend = true;
         // console.log(`[Joystick] Active: Sending X=${sendX.toFixed(3)}, Y=${sendY.toFixed(3)}`); // Debug log
    } else {
        // If joystick is inactive (released), only send if the last *sent* position wasn't zero.
        // This sends the final {0, 0} packet exactly once after release.
        if (Math.abs(lastX) > 1e-4 || Math.abs(lastY) > 1e-4) { // Check if last sent was non-zero
            shouldSend = true;
             // console.log(`[Joystick] Inactive: Sending final zero. Last sent was X=${lastX.toFixed(3)}, Y=${lastY.toFixed(3)}`); // Debug log
        } else {
             // console.log(`[Joystick] Inactive: Already sent zero. Doing nothing.`); // Debug log
        }
    }

    if (shouldSend) {
        const payload = { type: "joystick", x: sendX, y: sendY };
        // console.log("[Joystick] Sending WS:", payload); // Can be noisy
        if (sendJoystickPayload && sendJoystickPayload(payload)) {
            // Update last sent only on successful send
            appState.joystick.lastSentData.x = sendX;
            appState.joystick.lastSentData.y = sendY;
        } else {
             console.warn("Failed to send joystick data via configured transport.");
        }
    }
}
