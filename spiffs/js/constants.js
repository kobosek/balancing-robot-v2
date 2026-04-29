export const MAX_DATA_POINTS = 400;
export const DATA_FETCH_INTERVAL_MS = 150;
export const STATE_FETCH_INTERVAL_MS = 1000;
export const JOYSTICK_SEND_INTERVAL_MS = 75;
export const WS_RECONNECT_DELAY_MS = 3000;

export const API_COMMAND_URL = '/api/command';
export const API_CONFIG_URL = '/api/config';
export const API_STATE_URL = '/api/state';
export const API_DATA_URL = '/data';
export const API_OTA_URL = '/api/ota';
export const API_LOGS_URL = '/api/logs';

// Function to get WebSocket URL dynamically
export function getWsUrl() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${window.location.host}/ws`;
}

// Graphing constants
export const GRAPH_COLORS = ['#6f42c1', '#0d6efd', '#dc3545', '#198754', '#ffc107', '#fd7e14', '#20c997', '#0dcaf0']; // Added color for yaw
export const Y_SPEED_RANGE_DPS = 800;
export const Y_ANGLE_RANGE_DEG = 90;
export const Y_EFFORT_RANGE = 1.0;
export const Y_YAW_ANGLE_RANGE_DEG = 180;
export const Y_YAWRATE_RANGE_DPS = 180;
