/**
 * GARUDA VISION — MAIN APPLICATION ORCHESTRATOR
 * Establishes real-time WebSocket connection to backend, dispatches incoming
 * telemetry streams to TelemetryUI, ChartsManager, and GarudaVisualization,
 * and handles all simulation controls, camera switches, and layer toggles.
 */

(function (window) {
  'use strict';

  // Application State
  const state = {
    ws: null,
    isConnected: false,
    reconnectAttempts: 0,
    maxReconnectDelay: 10000,
    isPaused: false,
    selectedRateHz: 20,
    selectedProvider: 'SYNTHETIC',
    noiseEnabled: true,
    lastPingSent: 0,
    latencyMs: 0,
    pingIntervalId: null,
    lastTelemetryTime: 0
  };

  // Determine WebSocket URL
  function getWsUrl() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    // If running in development without a host port, default to localhost:3000
    const host = location.host || 'localhost:3000';
    return `${protocol}//${host}/ws`;
  }

  // ===========================================================================
  // 1. WEBSOCKET CLIENT & AUTO-RECONNECTION
  // ===========================================================================
  function connectWebSocket() {
    const wsUrl = getWsUrl();
    console.log(`[GARUDA-APP] Connecting to WebSocket stream at ${wsUrl}...`);

    try {
      state.ws = new WebSocket(wsUrl);
    } catch (err) {
      console.warn('[GARUDA-APP] WebSocket instantiation error:', err);
      scheduleReconnect();
      return;
    }

    state.ws.onopen = function () {
      console.log('[GARUDA-APP] WebSocket connection established successfully.');
      state.isConnected = true;
      state.reconnectAttempts = 0;
      if (window.TelemetryUI) {
        window.TelemetryUI.updateConnectionStatus(true, state.latencyMs);
      }
      startPingMonitor();
    };

    state.ws.onmessage = function (event) {
      try {
        const payload = JSON.parse(event.data);
        handleIncomingMessage(payload);
      } catch (err) {
        console.error('[GARUDA-APP] Failed to parse WebSocket message payload:', err);
      }
    };

    state.ws.onclose = function (event) {
      console.warn(`[GARUDA-APP] WebSocket connection closed (code: ${event.code}).`);
      state.isConnected = false;
      stopPingMonitor();
      if (window.TelemetryUI) {
        window.TelemetryUI.updateConnectionStatus(false, null);
      }
      scheduleReconnect();
    };

    state.ws.onerror = function (error) {
      console.error('[GARUDA-APP] WebSocket error encountered:', error);
      state.ws.close();
    };
  }

  function scheduleReconnect() {
    state.reconnectAttempts++;
    // Exponential backoff with jitter: 1s, 2s, 4s, 8s, up to max 10s
    const delay = Math.min(1000 * Math.pow(1.5, state.reconnectAttempts), state.maxReconnectDelay);
    console.log(`[GARUDA-APP] Scheduling reconnect in ${Math.round(delay)}ms (attempt #${state.reconnectAttempts})...`);
    setTimeout(connectWebSocket, delay);
  }

  function sendCommand(msgObj) {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
      state.ws.send(JSON.stringify(msgObj));
    } else {
      console.warn('[GARUDA-APP] Cannot send command; WebSocket not open:', msgObj);
    }
  }

  // Latency & Ping Heartbeat
  function startPingMonitor() {
    stopPingMonitor();
    state.pingIntervalId = setInterval(() => {
      if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.lastPingSent = Date.now();
        sendCommand({ type: 'PING', timestamp: state.lastPingSent });
      }
    }, 3000);
  }

  function stopPingMonitor() {
    if (state.pingIntervalId) {
      clearInterval(state.pingIntervalId);
      state.pingIntervalId = null;
    }
  }

  // ===========================================================================
  // 2. INCOMING PACKET DISPATCHER
  // ===========================================================================
  function handleIncomingMessage(msg) {
    if (!msg) return;

    // Handle PONG response
    if (msg.type === 'PONG') {
      const now = Date.now();
      const originTime = msg.timestamp || state.lastPingSent;
      state.latencyMs = Math.max(now - originTime, 1);
      if (window.TelemetryUI) {
        window.TelemetryUI.updateConnectionStatus(true, state.latencyMs);
      }
      return;
    }

    // Extract telemetry payload (handles { type: 'TELEMETRY', data: ... } packet envelope)
    const telemetry = msg.type === 'TELEMETRY' ? msg.data : (msg.data || msg);

    // Telemetry Packet Check: standard schema must have header & vehicle
    if (telemetry && telemetry.header && telemetry.vehicle) {
      state.lastTelemetryTime = Date.now();

      // Dispatch 1: DOM Telemetry UI
      if (window.TelemetryUI && typeof window.TelemetryUI.update === 'function') {
        window.TelemetryUI.update(telemetry);
      }

      // Dispatch 2: Rolling Analytics Charts
      if (window.ChartsManager && typeof window.ChartsManager.update === 'function') {
        window.ChartsManager.update(telemetry);
      }

      // Dispatch 3: 3D Visualization Engine (Agent 2)
      if (window.GarudaVisualization && typeof window.GarudaVisualization.updateTelemetry === 'function') {
        window.GarudaVisualization.updateTelemetry(telemetry);
      }
    }
  }

  // ===========================================================================
  // 3. SIMULATION CONTROLS & EVENT BINDINGS
  // ===========================================================================
  function setupSimulationControls() {
    // Play / Pause Toggle
    const btnPlayPause = document.getElementById('btn-play-pause');
    const playPauseIcon = document.getElementById('play-pause-icon');
    const playPauseText = document.getElementById('play-pause-text');

    if (btnPlayPause) {
      btnPlayPause.addEventListener('click', () => {
        state.isPaused = !state.isPaused;
        const command = state.isPaused ? 'PAUSE' : 'START';
        sendCommand({ type: 'SIM_CONTROL', command: command });

        if (state.isPaused) {
          btnPlayPause.classList.remove('btn-primary');
          if (playPauseIcon) playPauseIcon.innerHTML = '&#9658;';
          if (playPauseText) playPauseText.textContent = 'RESUME';
        } else {
          btnPlayPause.classList.add('btn-primary');
          if (playPauseIcon) playPauseIcon.innerHTML = '&#10074;&#10074;';
          if (playPauseText) playPauseText.textContent = 'PAUSE';
        }
      });
    }

    // Step Forward 1 Frame
    const btnStep = document.getElementById('btn-step');
    if (btnStep) {
      btnStep.addEventListener('click', () => {
        sendCommand({ type: 'SIM_CONTROL', command: 'STEP' });
      });
    }

    // Reset Simulation
    const btnReset = document.getElementById('btn-reset');
    if (btnReset) {
      btnReset.addEventListener('click', () => {
        sendCommand({ type: 'SIM_CONTROL', command: 'RESET' });
        if (window.ChartsManager) {
          window.ChartsManager.reset();
        }
      });
    }

    // Spawn Hazard Ahead Buttons
    function spawnHazardAhead() {
      console.log('[GARUDA-APP] Requesting obstacle spawn ahead of ego vehicle...');
      sendCommand({ type: 'INJECT_OBSTACLE', distance: 18.0 });
    }

    const btnSpawnHazard = document.getElementById('btn-spawn-hazard');
    if (btnSpawnHazard) {
      btnSpawnHazard.addEventListener('click', spawnHazardAhead);
    }

    const btnQuickSpawn = document.getElementById('btn-quick-spawn');
    if (btnQuickSpawn) {
      btnQuickSpawn.addEventListener('click', spawnHazardAhead);
    }

    // Tick Rate Selector (10, 20, 30, 50 Hz)
    const rateButtons = document.querySelectorAll('#rate-selector .rate-btn');
    rateButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        rateButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const rateHz = parseInt(btn.getAttribute('data-rate'), 10);
        state.selectedRateHz = rateHz;
        sendCommand({ type: 'SET_RATE', rateHz: rateHz });
      });
    });

    // Data Provider Switcher (SYNTHETIC vs CARLA)
    const btnSynthetic = document.getElementById('btn-provider-synthetic');
    const btnCarla = document.getElementById('btn-provider-carla');

    function selectProvider(provider) {
      state.selectedProvider = provider;
      if (btnSynthetic) btnSynthetic.classList.toggle('active', provider === 'SYNTHETIC');
      if (btnCarla) btnCarla.classList.toggle('active', provider === 'CARLA');
      sendCommand({ type: 'SET_PROVIDER', provider: provider });
    }

    if (btnSynthetic) {
      btnSynthetic.addEventListener('click', () => selectProvider('SYNTHETIC'));
    }
    if (btnCarla) {
      btnCarla.addEventListener('click', () => selectProvider('CARLA'));
    }

    // Sensor Noise Toggle
    const toggleNoise = document.getElementById('toggle-noise');
    const noiseStatusText = document.getElementById('noise-status-text');
    if (toggleNoise) {
      toggleNoise.addEventListener('change', () => {
        state.noiseEnabled = toggleNoise.checked;
        if (noiseStatusText) {
          noiseStatusText.textContent = state.noiseEnabled ? 'ON' : 'OFF';
        }
        sendCommand({ type: 'TOGGLE_NOISE', enabled: state.noiseEnabled });
      });
    }

    // Camera Switcher Buttons
    const cameraButtons = document.querySelectorAll('#camera-controls .hud-btn');
    cameraButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        cameraButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const mode = btn.getAttribute('data-camera');
        if (window.GarudaVisualization) {
          if (typeof window.GarudaVisualization.setCameraMode === 'function') {
            window.GarudaVisualization.setCameraMode(mode);
          } else if (typeof window.GarudaVisualization.setCamera === 'function') {
            window.GarudaVisualization.setCamera(mode);
          }
        }
      });
    });

    // Sensor Layer Overlays
    const layerToggles = document.querySelectorAll('.layer-toggle-group .hud-toggle');
    layerToggles.forEach(btn => {
      btn.addEventListener('click', () => {
        btn.classList.toggle('active');
        const layer = btn.getAttribute('data-layer');
        const isActive = btn.classList.contains('active');

        if (window.GarudaVisualization) {
          if (layer === 'lidar' && typeof window.GarudaVisualization.toggleLidar === 'function') {
            window.GarudaVisualization.toggleLidar(isActive);
          } else if ((layer === 'bbox' || layer === 'obstacles') && typeof window.GarudaVisualization.toggleObstacles === 'function') {
            window.GarudaVisualization.toggleObstacles(isActive);
          } else if (layer === 'grid' && typeof window.GarudaVisualization.toggleGrid === 'function') {
            window.GarudaVisualization.toggleGrid(isActive);
          } else if ((layer === 'path' || layer === 'trajectory') && typeof window.GarudaVisualization.toggleTrajectory === 'function') {
            window.GarudaVisualization.toggleTrajectory(isActive);
          }
        }
      });
    });

    // Keyboard Shortcuts
    document.addEventListener('keydown', (e) => {
      // Ignore if typing inside input
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      if (e.code === 'Space') {
        e.preventDefault();
        btnPlayPause?.click();
      } else if (e.key === '1') {
        document.querySelector('button[data-camera="chase"]')?.click();
      } else if (e.key === '2') {
        document.querySelector('button[data-camera="god"]')?.click();
      } else if (e.key === '3') {
        document.querySelector('button[data-camera="cockpit"]')?.click();
      } else if (e.key === '4') {
        document.querySelector('button[data-camera="orbit"]')?.click();
      } else if (e.key === 'l' || e.key === 'L') {
        document.getElementById('toggle-lidar')?.click();
      } else if (e.key === 'b' || e.key === 'B') {
        document.getElementById('toggle-bbox')?.click();
      } else if (e.key === 'g' || e.key === 'G') {
        document.getElementById('toggle-grid')?.click();
      } else if (e.key === 't' || e.key === 'T') {
        document.getElementById('toggle-path')?.click();
      }
    });
  }

  // ===========================================================================
  // 4. BOOTSTRAP INITIALIZATION
  // ===========================================================================
  function init() {
    console.log('[GARUDA-APP] Initializing Garuda Vision Frontend...');
    
    // Initialize 3D Engine if present
    if (window.GarudaVisualization && typeof window.GarudaVisualization.init === 'function') {
      try {
        window.GarudaVisualization.init('#viewport-container');
      } catch (err) {
        console.error('[GARUDA-APP] Error initializing 3D Visualization engine:', err);
      }
    }

    setupSimulationControls();
    connectWebSocket();
  }

  document.addEventListener('DOMContentLoaded', init);

  // Expose state and dispatch helper globally
  window.GarudaApp = {
    state: state,
    sendCommand: sendCommand,
    connectWebSocket: connectWebSocket
  };

})(window);
