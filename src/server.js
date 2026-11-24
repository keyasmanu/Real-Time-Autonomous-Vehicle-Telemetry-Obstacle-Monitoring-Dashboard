import express from 'express';
import http from 'http';
import path from 'path';
import { fileURLToPath } from 'url';
import { WebSocketHandler } from './websocket/ws-handler.js';
import { SimulationManager } from './simulation/index.js';

// Determine directory paths
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const publicPath = path.resolve(__dirname, '..', 'public');

// Create Express app and HTTP server
const app = express();
const server = http.createServer(app);

// Global middleware
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Permissive CORS middleware for dashboard & visualization integrations
app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', '*');
  res.header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization');
  if (req.method === 'OPTIONS') {
    return res.sendStatus(204);
  }
  next();
});

// Serve static frontend assets from public/
app.use(express.static(publicPath));

// Shared application state references
let simulationManager = null;
let wsHandler = null;

/**
 * Initialize simulation manager and websocket handler
 */
export function initModules(options = {}) {
  if (!simulationManager) {
    simulationManager = new SimulationManager({
      updateRateHz: options.updateRateHz ?? 20,
      provider: options.provider || 'SYNTHETIC',
      noiseEnabled: options.noiseEnabled ?? false,
      autoStart: options.autoStart ?? true
    });
  }

  if (!wsHandler) {
    wsHandler = new WebSocketHandler(server, simulationManager);
  } else {
    wsHandler.setSimulationManager(simulationManager);
  }

  // Store in app.locals for test inspection
  app.locals.simulationManager = simulationManager;
  app.locals.wsHandler = wsHandler;

  return { simulationManager, wsHandler };
}

// -----------------------------------------------------------------------------
// REST API Endpoints
// -----------------------------------------------------------------------------

/**
 * GET /api/status
 * Returns system status, active data provider, simulation running state,
 * tick rate (Hz), connected client count, and uptime.
 */
app.get('/api/status', (req, res) => {
  const sim = simulationManager;
  const simStatus = sim ? sim.getStatus() : {};
  const isRunning = Boolean(simStatus.isRunning && !simStatus.isPaused);
  const provider = simStatus.activeProvider || sim?.providerType || 'SYNTHETIC';
  const rateHz = simStatus.updateRateHz || sim?.updateRateHz || 20;
  const connectedClients = wsHandler ? wsHandler.getClientCount() : 0;
  const uptime = Math.floor(process.uptime() * 100) / 100;

  res.json({
    status: 'ONLINE',
    systemStatus: 'OPTIMAL',
    activeProvider: provider,
    provider: provider,
    simulationRunning: isRunning,
    running: isRunning,
    isPaused: Boolean(simStatus.isPaused),
    tickRateHz: rateHz,
    rateHz: rateHz,
    noiseEnabled: Boolean(simStatus.noiseEnabled),
    connectedClients,
    connectedClientCount: connectedClients,
    uptime,
    uptimeSeconds: uptime,
    timestamp: new Date().toISOString()
  });
});

/**
 * GET /api/telemetry/latest
 * Returns the most recent telemetry packet.
 */
app.get('/api/telemetry/latest', (req, res) => {
  let telemetry = null;
  if (simulationManager && typeof simulationManager.getLatestTelemetry === 'function') {
    telemetry = simulationManager.getLatestTelemetry();
  } else if (wsHandler) {
    telemetry = wsHandler.getLatestTelemetry();
  }

  if (!telemetry) {
    return res.status(204).json({ message: 'No telemetry frame generated yet' });
  }

  res.json(telemetry);
});

/**
 * POST /api/simulation/control
 * Accepts { command: "START" | "PAUSE" | "RESET" | "STEP", provider: "SYNTHETIC" | "CARLA", rateHz: number, noiseEnabled: boolean }
 */
app.post('/api/simulation/control', (req, res) => {
  const { command, provider, rateHz, noiseEnabled } = req.body || {};

  if (!command && !provider && rateHz === undefined && noiseEnabled === undefined) {
    return res.status(400).json({
      error: 'Bad Request',
      message: 'At least one control parameter ("command", "provider", "rateHz", "noiseEnabled") must be specified.'
    });
  }

  const applied = {};

  // 1. Process command
  if (command !== undefined) {
    const cmd = String(command).toUpperCase();
    const validCommands = ['START', 'PAUSE', 'RESET', 'STEP'];
    if (!validCommands.includes(cmd)) {
      return res.status(400).json({
        error: 'Bad Request',
        message: `Invalid command: "${command}". Allowed commands: ${validCommands.join(', ')}`
      });
    }

    if (wsHandler) {
      wsHandler.dispatchSimControl(cmd);
    } else if (simulationManager) {
      if (cmd === 'START') simulationManager.start();
      else if (cmd === 'PAUSE') simulationManager.pause();
      else if (cmd === 'RESET') simulationManager.reset();
      else if (cmd === 'STEP') simulationManager.step();
    }
    applied.command = cmd;
  }

  // 2. Process rateHz
  if (rateHz !== undefined) {
    const rate = Number(rateHz);
    if (isNaN(rate) || rate < 1 || rate > 100) {
      return res.status(400).json({
        error: 'Bad Request',
        message: `Invalid rateHz: "${rateHz}". Must be a number between 1 and 100.`
      });
    }

    if (wsHandler) {
      wsHandler.setSimulationRate(rate);
    } else if (simulationManager) {
      simulationManager.setUpdateRate(rate);
    }
    applied.rateHz = rate;
  }

  // 3. Process provider
  if (provider !== undefined) {
    const prov = String(provider).toUpperCase();
    const normalizedProvider = prov === 'CARLA_BRIDGE' ? 'CARLA' : prov;
    if (normalizedProvider !== 'SYNTHETIC' && normalizedProvider !== 'CARLA') {
      return res.status(400).json({
        error: 'Bad Request',
        message: `Invalid provider: "${provider}". Allowed providers: "SYNTHETIC", "CARLA"`
      });
    }

    try {
      if (wsHandler) {
        wsHandler.setSimulationProvider(normalizedProvider);
      } else if (simulationManager) {
        simulationManager.setProvider(normalizedProvider);
      }
      applied.provider = normalizedProvider;
    } catch (err) {
      return res.status(400).json({
        error: 'Bad Request',
        message: err.message
      });
    }
  }

  // 4. Process noiseEnabled
  if (noiseEnabled !== undefined) {
    const noise = Boolean(noiseEnabled);
    if (wsHandler) {
      wsHandler.toggleSimulationNoise(noise);
    } else if (simulationManager) {
      simulationManager.setNoiseEnabled(noise);
    }
    applied.noiseEnabled = noise;
  }

  // Broadcast updated status to all active WebSocket clients
  if (wsHandler) {
    wsHandler.broadcastStatus();
  }

  // Fetch updated status
  const currentStatus = wsHandler ? wsHandler.getStatus() : (simulationManager?.getStatus() || {});

  res.json({
    success: true,
    message: 'Simulation control applied successfully',
    applied,
    status: currentStatus
  });
});

/**
 * GET /api/carla/status
 * Returns CARLA bridge connection status and info.
 */
app.get('/api/carla/status', (req, res) => {
  let carlaInfo = null;
  if (simulationManager && typeof simulationManager.getCarlaStatus === 'function') {
    carlaInfo = simulationManager.getCarlaStatus();
  } else if (wsHandler) {
    carlaInfo = wsHandler.getCarlaStatus();
  }

  if (!carlaInfo) {
    carlaInfo = {
      name: 'CarlaDataProvider',
      type: 'CARLA_BRIDGE',
      isInitialized: false,
      isRunning: false,
      isPaused: false,
      status: 'DISCONNECTED',
      message: 'CARLA bridge not connected. Operating in SYNTHETIC simulation mode.'
    };
  }

  res.json({
    ...carlaInfo,
    timestamp: new Date().toISOString()
  });
});

/**
 * GET /api/health
 * Simple health check endpoint for monitoring probes.
 */
app.get('/api/health', (req, res) => {
  res.json({
    status: 'UP',
    uptime: Math.floor(process.uptime()),
    timestamp: new Date().toISOString()
  });
});

// Single-page application route fallback
app.get('/', (req, res) => {
  res.sendFile(path.join(publicPath, 'index.html'));
});

// -----------------------------------------------------------------------------
// Server Lifecycle & Startup Logging
// -----------------------------------------------------------------------------

/**
 * Prints clean startup banner with connection details.
 * @param {number} port 
 */
function logBanner(port) {
  const line = '━'.repeat(60);
  console.log(`
${line}
  GARUDA VISION — Real-Time Autonomous Vehicle Telemetry
  Digital Twin & Obstacle Monitoring Platform
${line}
  Local Web Dashboard: http://localhost:${port}
  WebSocket Endpoint:  ws://localhost:${port}/ws
  
  Active REST Endpoints:
    • GET  /api/status
    • GET  /api/telemetry/latest
    • POST /api/simulation/control
    • GET  /api/carla/status
    • GET  /api/health
${line}
`);
}

/**
 * Sets up graceful termination handlers for SIGINT and SIGTERM.
 */
function setupGracefulShutdown() {
  const shutdown = (signal) => {
    console.log(`\n[Server] Received ${signal}. Initiating graceful shutdown...`);

    // 1. Terminate WebSocket connections
    if (wsHandler) {
      console.log('[Server] Closing WebSocket connections...');
      wsHandler.close();
    }

    // 2. Stop simulation loop
    if (simulationManager) {
      console.log('[Server] Stopping simulation engine...');
      if (typeof simulationManager.pause === 'function') {
        simulationManager.pause();
      }
      if (typeof simulationManager.destroy === 'function') {
        simulationManager.destroy();
      }
    }

    // 3. Close HTTP server
    server.close(() => {
      console.log('[Server] HTTP server closed gracefully. Goodbye.');
      process.exit(0);
    });

    // Enforce shutdown if connections linger beyond 5s
    setTimeout(() => {
      console.error('[Server] Forceful shutdown triggered after 5s timeout.');
      process.exit(1);
    }, 5000).unref();
  };

  process.once('SIGINT', () => shutdown('SIGINT'));
  process.once('SIGTERM', () => shutdown('SIGTERM'));
}

/**
 * Starts the HTTP and WebSocket servers on the specified port.
 * @param {number} [port]
 * @returns {Promise<http.Server>}
 */
export function startServer(port = process.env.PORT || 3000) {
  return new Promise((resolve, reject) => {
    initModules();
    setupGracefulShutdown();

    server.listen(port, () => {
      logBanner(port);
      resolve(server);
    });

    server.on('error', (err) => {
      console.error('[Server] Listen error:', err.message);
      reject(err);
    });
  });
}

/**
 * Stops HTTP and WebSocket servers. Useful for test teardowns.
 * @returns {Promise<void>}
 */
export function stopServer() {
  return new Promise((resolve) => {
    if (wsHandler) {
      wsHandler.close();
      wsHandler = null;
    }
    if (simulationManager) {
      if (typeof simulationManager.destroy === 'function') {
        simulationManager.destroy();
      } else if (typeof simulationManager.pause === 'function') {
        simulationManager.pause();
      }
      simulationManager = null;
    }
    server.close(() => {
      resolve();
    });
  });
}

// Auto-start server if executed directly
const isDirectExecution = process.argv[1] && (
  path.resolve(process.argv[1]) === __filename ||
  path.resolve(process.argv[1]) === path.resolve(__dirname, 'server.js')
);

if (isDirectExecution) {
  const port = process.env.PORT || 3000;
  startServer(port).catch((err) => {
    console.error('[Server] Fatal error on start:', err);
    process.exit(1);
  });
}

export { app, server, simulationManager, wsHandler };
export default app;
