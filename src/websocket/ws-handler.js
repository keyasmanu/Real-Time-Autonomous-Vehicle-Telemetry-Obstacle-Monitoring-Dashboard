import { WebSocketServer, WebSocket } from 'ws';

/**
 * WebSocketHandler manages the real-time bi-directional WebSocket communication
 * between the GARUDA VISION backend and connected dashboard clients.
 */
export class WebSocketHandler {
  /**
   * @param {import('http').Server} [httpServer] - Optional HTTP server to bind WebSocket server
   * @param {object} [simulationManager] - SimulationManager instance emitting 'telemetry' events
   * @param {object} [options] - Additional configuration options
   */
  constructor(httpServer, simulationManager = null, options = {}) {
    this.options = {
      path: '/ws',
      heartbeatIntervalMs: 30000,
      ...options
    };

    this.clients = new Set();
    this.latestTelemetry = null;
    this.simulationManager = null;
    this.telemetryListener = null;

    // Initialize WebSocketServer in noServer mode for explicit upgrade handling
    this.wss = new WebSocketServer({ noServer: true });

    // Mount on HTTP server upgrade event if server is provided
    if (httpServer) {
      this.attachToServer(httpServer);
    }

    // Attach simulation manager if provided
    if (simulationManager) {
      this.setSimulationManager(simulationManager);
    }

    // Setup connection handler
    this.wss.on('connection', (ws, request) => {
      this.handleConnection(ws, request);
    });

    // Start periodic heartbeat
    this.startHeartbeat();
  }

  /**
   * Attach upgrade event listener to an existing HTTP server.
   * Handles upgrades on both '/ws' and '/' routes.
   * @param {import('http').Server} server 
   */
  attachToServer(server) {
    server.on('upgrade', (request, socket, head) => {
      try {
        const host = request.headers.host || 'localhost';
        const url = new URL(request.url, `http://${host}`);
        const pathname = url.pathname;

        // Support /ws, /ws/, and root /
        if (pathname === '/ws' || pathname === '/ws/' || pathname === '/') {
          this.wss.handleUpgrade(request, socket, head, (ws) => {
            this.wss.emit('connection', ws, request);
          });
        } else {
          socket.destroy();
        }
      } catch (err) {
        console.error('[WS] Upgrade handling error:', err.message);
        socket.destroy();
      }
    });
  }

  /**
   * Bind or replace the simulation manager instance.
   * Listens for 'telemetry' events to broadcast.
   * @param {object} simManager 
   */
  setSimulationManager(simManager) {
    if (this.simulationManager && this.telemetryListener) {
      if (typeof this.simulationManager.off === 'function') {
        this.simulationManager.off('telemetry', this.telemetryListener);
      } else if (typeof this.simulationManager.removeListener === 'function') {
        this.simulationManager.removeListener('telemetry', this.telemetryListener);
      }
    }

    this.simulationManager = simManager;

    if (this.simulationManager && typeof this.simulationManager.on === 'function') {
      this.telemetryListener = (payload) => {
        this.broadcastTelemetry(payload);
      };
      this.simulationManager.on('telemetry', this.telemetryListener);

      // Cache existing latest telemetry if present
      if (typeof this.simulationManager.getLatestTelemetry === 'function') {
        const latest = this.simulationManager.getLatestTelemetry();
        if (latest) this.latestTelemetry = latest;
      }
    }
  }

  /**
   * Start periodic ping/pong heartbeats to prune dead connections.
   */
  startHeartbeat() {
    if (this.heartbeatTimer) clearInterval(this.heartbeatTimer);
    this.heartbeatTimer = setInterval(() => {
      for (const client of this.clients) {
        if (client.isAlive === false) {
          console.log('[WS] Terminating unresponsive client (heartbeat missed)');
          this.clients.delete(client);
          client.terminate();
          continue;
        }
        client.isAlive = false;
        try {
          client.ping();
        } catch (e) {
          this.clients.delete(client);
        }
      }
    }, this.options.heartbeatIntervalMs);
  }

  /**
   * Handle incoming WebSocket client connection.
   * @param {WebSocket} ws 
   * @param {import('http').IncomingMessage} request 
   */
  handleConnection(ws, request) {
    ws.isAlive = true;
    this.clients.add(ws);

    const clientIp = request?.socket?.remoteAddress || 'unknown';
    console.log(`[WS] Client connected from ${clientIp}. Total active clients: ${this.clients.size}`);

    ws.on('pong', () => {
      ws.isAlive = true;
    });

    ws.on('message', (data) => {
      ws.isAlive = true;
      this.handleClientMessage(ws, data);
    });

    ws.on('close', (code, reason) => {
      this.clients.delete(ws);
      console.log(`[WS] Client disconnected (code: ${code}). Total active clients: ${this.clients.size}`);
    });

    ws.on('error', (err) => {
      console.error('[WS] Client connection error:', err.message);
      this.clients.delete(ws);
    });

    // 1. Send INIT message with current configuration and provider info
    const status = this.getSimulationStatus();
    const carlaStatus = this.getCarlaStatus();

    this.send(ws, {
      type: 'INIT',
      data: {
        provider: status.provider || 'SYNTHETIC',
        running: status.running !== undefined ? status.running : false,
        rateHz: status.rateHz || 20,
        noiseEnabled: status.noiseEnabled !== undefined ? status.noiseEnabled : true,
        carlaStatus,
        clientCount: this.clients.size,
        serverTime: new Date().toISOString()
      }
    });

    // 2. Immediately send latest telemetry if available
    const latest = this.getLatestTelemetry();
    if (latest) {
      this.send(ws, {
        type: 'TELEMETRY',
        data: latest
      });
    }

    // 3. Send current status
    this.send(ws, {
      type: 'STATUS',
      data: this.getStatus()
    });
  }

  /**
   * Handle incoming JSON message from a client.
   * @param {WebSocket} ws 
   * @param {Buffer|string} rawData 
   */
  handleClientMessage(ws, rawData) {
    let message;
    try {
      message = JSON.parse(rawData.toString());
    } catch (err) {
      this.send(ws, { type: 'ERROR', message: 'Malformed JSON payload' });
      return;
    }

    if (!message || typeof message !== 'object') {
      this.send(ws, { type: 'ERROR', message: 'Payload must be a JSON object' });
      return;
    }

    const { type } = message;

    try {
      switch (type) {
        case 'SIM_CONTROL': {
          const command = (message.command || '').toUpperCase();
          const validCommands = ['START', 'PAUSE', 'RESET', 'STEP'];
          if (!validCommands.includes(command)) {
            this.send(ws, { type: 'ERROR', message: `Invalid SIM_CONTROL command: ${message.command}` });
            return;
          }
          this.dispatchSimControl(command);
          this.broadcastStatus();
          break;
        }

        case 'SET_RATE': {
          const rateHz = Number(message.rateHz ?? message.rate);
          if (isNaN(rateHz) || rateHz <= 0 || rateHz > 200) {
            this.send(ws, { type: 'ERROR', message: `Invalid rateHz: ${message.rateHz}. Must be between 1 and 200.` });
            return;
          }
          this.setSimulationRate(rateHz);
          this.broadcastStatus();
          break;
        }

        case 'SET_PROVIDER': {
          const provider = (message.provider || '').toUpperCase();
          if (provider !== 'SYNTHETIC' && provider !== 'CARLA' && provider !== 'CARLA_BRIDGE') {
            this.send(ws, { type: 'ERROR', message: `Invalid provider: ${message.provider}. Supported: SYNTHETIC, CARLA` });
            return;
          }
          this.setSimulationProvider(provider);
          this.broadcastStatus();
          break;
        }

        case 'TOGGLE_NOISE': {
          const enabled = message.enabled !== undefined ? Boolean(message.enabled) : undefined;
          this.toggleSimulationNoise(enabled);
          this.broadcastStatus();
          break;
        }

        case 'INJECT_OBSTACLE': {
          if (this.simulationManager && typeof this.simulationManager.injectObstacle === 'function') {
            let obsData = message.obstacle || {};
            if (!obsData.position && this.latestTelemetry?.vehicle) {
              const veh = this.latestTelemetry.vehicle;
              const headingRad = veh.orientation.headingRad || 0;
              const dist = Number(message.distance) || 18.0;
              obsData.position = {
                x: veh.position.x + Math.cos(headingRad) * dist,
                y: veh.position.y + Math.sin(headingRad) * dist,
                z: 0.0
              };
            }
            const injected = this.simulationManager.injectObstacle(obsData);
            this.broadcast({
              type: 'SYSTEM_EVENT',
              event: 'OBSTACLE_INJECTED',
              payload: injected
            });
          }
          break;
        }

        case 'PING': {
          this.send(ws, { type: 'PONG', timestamp: Date.now() });
          break;
        }

        case 'GET_STATUS': {
          this.send(ws, { type: 'STATUS', data: this.getStatus() });
          break;
        }

        case 'GET_TELEMETRY': {
          const latest = this.getLatestTelemetry();
          if (latest) {
            this.send(ws, { type: 'TELEMETRY', data: latest });
          } else {
            this.send(ws, { type: 'WARN', message: 'No telemetry available yet' });
          }
          break;
        }

        default:
          console.warn(`[WS] Unrecognized message type: ${type}`);
          this.send(ws, { type: 'WARN', message: `Unrecognized message type: ${type}` });
      }
    } catch (err) {
      console.error('[WS] Error processing client message:', err);
      this.send(ws, { type: 'ERROR', message: 'Internal error processing message' });
    }
  }

  /**
   * Dispatch simulation control command (START, PAUSE, RESET, STEP).
   * @param {string} command 
   */
  dispatchSimControl(command) {
    if (!this.simulationManager) {
      console.warn('[WS] SimulationManager not available to dispatch command:', command);
      return;
    }

    const cmd = command.toUpperCase();

    if (typeof this.simulationManager.handleControl === 'function') {
      this.simulationManager.handleControl({ command: cmd });
      return;
    }

    switch (cmd) {
      case 'START':
        if (typeof this.simulationManager.start === 'function') this.simulationManager.start();
        else if (typeof this.simulationManager.startSimulation === 'function') this.simulationManager.startSimulation();
        break;
      case 'PAUSE':
        if (typeof this.simulationManager.pause === 'function') this.simulationManager.pause();
        else if (typeof this.simulationManager.pauseSimulation === 'function') this.simulationManager.pauseSimulation();
        break;
      case 'RESET':
        if (typeof this.simulationManager.reset === 'function') this.simulationManager.reset();
        else if (typeof this.simulationManager.resetSimulation === 'function') this.simulationManager.resetSimulation();
        break;
      case 'STEP':
        if (typeof this.simulationManager.step === 'function') this.simulationManager.step();
        else if (typeof this.simulationManager.stepSimulation === 'function') this.simulationManager.stepSimulation();
        break;
      default:
        console.warn(`[WS] Unsupported command: ${command}`);
    }
  }

  /**
   * Set simulation frequency rate (Hz).
   * @param {number} rateHz 
   */
  setSimulationRate(rateHz) {
    if (!this.simulationManager) return;
    if (typeof this.simulationManager.setRate === 'function') {
      this.simulationManager.setRate(rateHz);
    } else if (typeof this.simulationManager.setUpdateRate === 'function') {
      this.simulationManager.setUpdateRate(rateHz);
    } else if (typeof this.simulationManager.setFrequency === 'function') {
      this.simulationManager.setFrequency(rateHz);
    } else if (typeof this.simulationManager.handleControl === 'function') {
      this.simulationManager.handleControl({ rateHz });
    }
  }

  /**
   * Switch data provider ("SYNTHETIC" or "CARLA").
   * @param {string} provider 
   */
  setSimulationProvider(provider) {
    if (!this.simulationManager) return;
    if (typeof this.simulationManager.setProvider === 'function') {
      this.simulationManager.setProvider(provider);
    } else if (typeof this.simulationManager.switchProvider === 'function') {
      this.simulationManager.switchProvider(provider);
    } else if (typeof this.simulationManager.handleControl === 'function') {
      this.simulationManager.handleControl({ provider });
    }
  }

  /**
   * Enable or disable sensor noise.
   * @param {boolean} [enabled] 
   */
  toggleSimulationNoise(enabled) {
    if (!this.simulationManager) return;
    if (typeof this.simulationManager.toggleNoise === 'function') {
      this.simulationManager.toggleNoise(enabled);
    } else if (typeof this.simulationManager.setNoise === 'function') {
      this.simulationManager.setNoise(enabled);
    } else if (typeof this.simulationManager.setNoiseEnabled === 'function') {
      this.simulationManager.setNoiseEnabled(enabled);
    } else if (typeof this.simulationManager.handleControl === 'function') {
      this.simulationManager.handleControl({ noiseEnabled: enabled });
    }
  }

  /**
   * Broadcast arbitrary message to all connected clients.
   * @param {object} messageObj
   */
  broadcast(messageObj) {
    if (!messageObj) return;
    const payload = JSON.stringify(messageObj);
    for (const client of this.clients) {
      if (client.readyState === WebSocket.OPEN) {
        try {
          client.send(payload);
        } catch (err) {
          console.error('[WS] Failed to broadcast message:', err.message);
        }
      }
    }
  }

  /**
   * Broadcast telemetry packet to all connected active clients.
   * @param {object} telemetryPayload 
   */
  broadcastTelemetry(telemetryPayload) {
    if (!telemetryPayload) return;
    this.latestTelemetry = telemetryPayload;

    const messageStr = JSON.stringify({
      type: 'TELEMETRY',
      data: telemetryPayload
    });

    for (const client of this.clients) {
      if (client.readyState === WebSocket.OPEN) {
        try {
          client.send(messageStr);
        } catch (err) {
          console.error('[WS] Failed to send telemetry to client:', err.message);
        }
      }
    }
  }

  /**
   * Broadcast status update to all connected clients.
   * @param {object} [customStatus] 
   */
  broadcastStatus(customStatus = null) {
    const statusPayload = JSON.stringify({
      type: 'STATUS',
      data: customStatus || this.getStatus()
    });

    for (const client of this.clients) {
      if (client.readyState === WebSocket.OPEN) {
        try {
          client.send(statusPayload);
        } catch (err) {
          console.error('[WS] Failed to send status to client:', err.message);
        }
      }
    }
  }

  /**
   * Helper to send JSON payload to a specific client.
   * @param {WebSocket} ws 
   * @param {object|string} payload 
   */
  send(ws, payload) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      try {
        ws.send(typeof payload === 'string' ? payload : JSON.stringify(payload));
      } catch (err) {
        console.error('[WS] Send error:', err.message);
      }
    }
  }

  /**
   * Get latest telemetry packet.
   * @returns {object|null}
   */
  getLatestTelemetry() {
    if (this.simulationManager && typeof this.simulationManager.getLatestTelemetry === 'function') {
      const latest = this.simulationManager.getLatestTelemetry();
      if (latest) {
        this.latestTelemetry = latest;
        return latest;
      }
    }
    return this.latestTelemetry;
  }

  /**
   * Query status from simulation manager.
   * @returns {object}
   */
  getSimulationStatus() {
    if (!this.simulationManager) {
      return {
        running: false,
        simulationRunning: false,
        isPaused: false,
        provider: 'SYNTHETIC',
        activeProvider: 'SYNTHETIC',
        rateHz: 20,
        tickRateHz: 20,
        updateRateHz: 20,
        noiseEnabled: false
      };
    }

    if (typeof this.simulationManager.getStatus === 'function') {
      const s = this.simulationManager.getStatus();
      const isRunning = Boolean(s.isRunning && !s.isPaused);
      return {
        ...s,
        running: isRunning,
        simulationRunning: isRunning,
        isPaused: Boolean(s.isPaused),
        provider: s.activeProvider || s.provider || 'SYNTHETIC',
        activeProvider: s.activeProvider || s.provider || 'SYNTHETIC',
        rateHz: s.updateRateHz || s.rateHz || 20,
        tickRateHz: s.updateRateHz || s.rateHz || 20,
        updateRateHz: s.updateRateHz || s.rateHz || 20,
        noiseEnabled: s.noiseEnabled !== undefined ? Boolean(s.noiseEnabled) : false
      };
    }

    const isRunning = Boolean(this.simulationManager.isRunning && !this.simulationManager.isPaused);
    return {
      running: isRunning,
      simulationRunning: isRunning,
      isPaused: Boolean(this.simulationManager.isPaused),
      provider: this.simulationManager.providerType || this.simulationManager.provider || 'SYNTHETIC',
      activeProvider: this.simulationManager.providerType || this.simulationManager.provider || 'SYNTHETIC',
      rateHz: this.simulationManager.updateRateHz || this.simulationManager.rateHz || 20,
      tickRateHz: this.simulationManager.updateRateHz || this.simulationManager.rateHz || 20,
      noiseEnabled: Boolean(this.simulationManager.noiseEnabled)
    };
  }

  /**
   * Query CARLA status from simulation manager.
   * @returns {object}
   */
  getCarlaStatus() {
    if (this.simulationManager && typeof this.simulationManager.getCarlaStatus === 'function') {
      return this.simulationManager.getCarlaStatus();
    }
    return {
      connected: false,
      status: 'DISCONNECTED',
      host: 'localhost',
      port: 2000,
      activeProvider: this.getSimulationStatus().provider || 'SYNTHETIC',
      message: 'CARLA bridge not connected. Operating in SYNTHETIC simulation mode.'
    };
  }

  /**
   * Comprehensive system and WebSocket status.
   * @returns {object}
   */
  getStatus() {
    const simStatus = this.getSimulationStatus();
    const count = this.clients.size;
    return {
      systemStatus: 'OPTIMAL',
      activeProvider: simStatus.activeProvider || 'SYNTHETIC',
      provider: simStatus.provider || 'SYNTHETIC',
      simulationRunning: Boolean(simStatus.running),
      running: Boolean(simStatus.running),
      isPaused: Boolean(simStatus.isPaused),
      tickRateHz: simStatus.tickRateHz || 20,
      rateHz: simStatus.rateHz || 20,
      noiseEnabled: simStatus.noiseEnabled,
      connectedClients: count,
      connectedClientCount: count,
      timestamp: new Date().toISOString()
    };
  }

  /**
   * Connected clients count.
   * @returns {number}
   */
  getClientCount() {
    return this.clients.size;
  }

  get connectedClients() {
    return this.clients.size;
  }

  /**
   * Clean shutdown of WebSocket server and all client connections.
   */
  close() {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }

    for (const client of this.clients) {
      try {
        client.terminate();
      } catch (e) {}
    }
    this.clients.clear();

    if (this.wss) {
      this.wss.close();
    }
    console.log('[WS] WebSocketHandler shutdown complete');
  }
}

export default WebSocketHandler;
