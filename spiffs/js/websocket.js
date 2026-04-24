import { getWsUrl, WS_RECONNECT_DELAY_MS } from './constants.js';
import { appState } from './state.js';
import { updateWsStatusUI } from './ui.js';

let reconnectTimer = null;
let disconnectHandler = null;

export function setWebSocketDisconnectHandler(handler) {
    disconnectHandler = typeof handler === 'function' ? handler : null;
}

export function setupWebSocket() {
    const wsUrl = getWsUrl();
    console.log(`Connecting WebSocket: ${wsUrl}`);
    updateWsStatusUI('CONNECTING', '#ffc107');

    clearTimeout(reconnectTimer); // Clear any pending reconnect timer
    appState.timers.wsReconnect = null; // Clear tracked timer ID

    try {
        // Close existing connection if any before creating a new one
        if (appState.ws && appState.ws.readyState !== WebSocket.CLOSED) {
             console.log("Closing existing WebSocket connection before reconnecting.");
             // Remove listeners to prevent triggering onclose logic during manual closure
             appState.ws.onopen = null;
             appState.ws.onmessage = null;
             appState.ws.onerror = null;
             appState.ws.onclose = null;
             appState.ws.close();
             appState.ws = null; // Explicitly nullify
        }

        appState.ws = new WebSocket(wsUrl);

        appState.ws.onopen = () => {
            console.log("WebSocket Connected");
            updateWsStatusUI('CONNECTED', '#198754');
            clearTimeout(reconnectTimer); // Successfully connected, clear timer
            appState.timers.wsReconnect = null;
        };

        appState.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                console.log("WS RX:", data);
                // Handle incoming messages (e.g., pong)
                if (data.type === 'pong') {
                    console.log("Pong received.");
                }
                // Add handlers for other message types if needed
            } catch (e) {
                console.warn("Failed to parse WS message:", event.data, e);
            }
        };

        appState.ws.onerror = (error) => {
            console.error("WebSocket Error:", error);
            // UI update usually happens in onclose
        };

        appState.ws.onclose = (event) => {
            // Check if this socket instance is still the current one in appState
            // Prevents old socket onclose handlers firing after a reconnect attempt
            if (appState.ws !== event.target) {
                 console.log("Ignoring onclose event from outdated WebSocket instance.");
                 return;
            }

            console.log(`WebSocket Closed. Code: ${event.code}, Reason: ${event.reason}`);
            updateWsStatusUI('DISCONNECTED', '#dc3545');
            appState.ws = null; // Nullify the current WS instance
            if (disconnectHandler) {
                disconnectHandler();
            }

            // Schedule reconnection attempt
            clearTimeout(reconnectTimer); // Clear any existing timer just in case
            reconnectTimer = setTimeout(setupWebSocket, WS_RECONNECT_DELAY_MS);
            appState.timers.wsReconnect = reconnectTimer; // Track the new timer ID
             console.log(`WebSocket reconnect scheduled in ${WS_RECONNECT_DELAY_MS}ms. Timer ID: ${reconnectTimer}`);
        };
    } catch (error) {
        console.error("Failed to create WebSocket:", error);
        updateWsStatusUI('ERROR', '#dc3545');
        appState.ws = null; // Ensure ws state is null on creation error

        // Ensure reconnect attempt even if initial creation fails
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(setupWebSocket, WS_RECONNECT_DELAY_MS);
        appState.timers.wsReconnect = reconnectTimer;
        console.log(`WebSocket creation failed, reconnect scheduled. Timer ID: ${reconnectTimer}`);
    }
}

export function sendWebSocketMessage(payload) {
    if (appState.ws && appState.ws.readyState === WebSocket.OPEN) {
        try {
            appState.ws.send(JSON.stringify(payload));
            return true;
        } catch (e) {
            console.error("WS Send Error:", e);
            return false;
        }
    } else {
        // console.warn("WS not open, cannot send message.");
        return false;
    }
}
