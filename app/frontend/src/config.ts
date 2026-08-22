// Dynamic API & WebSocket configuration based on current browser window location

const hostname = typeof window !== 'undefined' && window.location.hostname ? window.location.hostname : 'localhost';
const protocol = typeof window !== 'undefined' && window.location.protocol === 'https:' ? 'https:' : 'http:';
const wsProtocol = typeof window !== 'undefined' && window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const backendPort = 8000;

export const API_BASE = `${protocol}//${hostname}:${backendPort}`;
export const WS_BASE = `${wsProtocol}//${hostname}:${backendPort}`;
