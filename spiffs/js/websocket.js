// js/websocket.js
import { getWsUrl, WS_RECONNECT_DELAY_MS } from './constants.js';
import { appState } from './state.js';
import { updateWsStatusUI } from './ui.js';
import { stopJoystickSendInterval } from './joystick.js'; // Import function to stop joystick on disconnect

let reconnectTimer = null;

export function setupWebSocket() {
    const wsUrl = getWsUrl();
    console.log(`Connecting WebSocket: ${wsUrl}`);
    updateWsStatusUI('CONNECTING', '#ffc107');

    clearTimeout(reconnectTimer); // Clear any pending reconnect timer

    try {
        // Close existing connection if any before creating a new one
        if (appState.ws && appState.ws.readyState !== WebSocket.CLOSED) {
             console.log("Closing existing WebSocket connection before reconnecting.");
             appState.ws.close();
        }

        appState.ws = new WebSocket(wsUrl);

        appState.ws.onopen = () => {
            console.log("WebSocket Connected");
            updateWsStatusUI('CONNECTED', '#198754');
            clearTimeout(reconnectTimer); // Successfully connected, clear timer
        };

        appState.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                console.log("WS RX:", data);
                // Handle incoming messages (e.g., pong)
                if (data.type === 'pong') {
                    console.log("Pong received.");
                }
            } catch (e) {
                console.warn("Failed to parse WS message:", event.data, e);
            }
        };

        appState.ws.onerror = (error) => {
            console.error("WebSocket Error:", error);
            // updateWsStatusUI might be called in onclose
        };

        appState.ws.onclose = (event) => {
            console.log(`WebSocket Closed. Code: ${event.code}, Reason: ${event.reason}`);
            updateWsStatusUI('DISCONNECTED', '#dc3545');
            appState.ws = null;
            stopJoystickSendInterval(); // Stop sending joystick data

            // Schedule reconnection attempt only if not intentionally closed?
            // For simplicity, always try to reconnect for now.
            clearTimeout(reconnectTimer);
            reconnectTimer = setTimeout(setupWebSocket, WS_RECONNECT_DELAY_MS);
        };
    } catch (error) {
        console.error("Failed to create WebSocket:", error);
        updateWsStatusUI('ERROR', '#dc3545');
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(setupWebSocket, WS_RECONNECT_DELAY_MS);
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