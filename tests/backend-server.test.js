import { test, describe, before, after } from 'node:test';
import assert from 'node:assert';
import http from 'http';
import { WebSocket } from 'ws';
import { startServer, stopServer, app } from '../src/server.js';

const TEST_PORT = 3188;
const BASE_URL = `http://127.0.0.1:${TEST_PORT}`;
const WS_URL = `ws://127.0.0.1:${TEST_PORT}/ws`;

describe('GARUDA VISION — Backend Server & WebSocket Layer Tests', () => {
  let serverInstance;

  before(async () => {
    serverInstance = await startServer(TEST_PORT);
  });

  after(async () => {
    await stopServer();
  });

  test('GET /api/status returns comprehensive status and metrics', async () => {
    const res = await fetch(`${BASE_URL}/api/status`);
    assert.strictEqual(res.status, 200);

    const data = await res.json();
    assert.strictEqual(data.status, 'ONLINE');
    assert.strictEqual(data.systemStatus, 'OPTIMAL');
    assert.ok(data.activeProvider === 'SYNTHETIC' || data.activeProvider === 'CARLA');
    assert.strictEqual(typeof data.simulationRunning, 'boolean');
    assert.strictEqual(typeof data.tickRateHz, 'number');
    assert.strictEqual(typeof data.connectedClients, 'number');
    assert.strictEqual(typeof data.uptime, 'number');
    assert.ok(data.timestamp);
  });

  test('GET /api/telemetry/latest returns schema-compliant telemetry packet', async () => {
    const res = await fetch(`${BASE_URL}/api/telemetry/latest`);
    assert.strictEqual(res.status, 200);

    const telemetry = await res.json();
    assert.ok(telemetry.header, 'Telemetry must contain header');
    assert.strictEqual(telemetry.header.isSimulated, true);
    assert.strictEqual(
      telemetry.header.disclaimer,
      'SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS'
    );
    assert.ok(telemetry.vehicle, 'Telemetry must contain vehicle block');
    assert.ok(telemetry.vehicle.position, 'Telemetry must contain vehicle position');
    assert.ok(telemetry.diagnostics, 'Telemetry must contain diagnostics');
    assert.ok(Array.isArray(telemetry.obstacles), 'Telemetry must contain obstacles array');
    assert.ok(telemetry.lidar, 'Telemetry must contain lidar block');
  });

  test('POST /api/simulation/control accepts and applies control commands', async () => {
    // 1. PAUSE
    const pauseRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: 'PAUSE' })
    });
    assert.strictEqual(pauseRes.status, 200);
    const pauseData = await pauseRes.json();
    assert.strictEqual(pauseData.success, true);
    assert.strictEqual(pauseData.status.isPaused, true);

    // 2. START / RESUME
    const startRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: 'START' })
    });
    assert.strictEqual(startRes.status, 200);
    const startData = await startRes.json();
    assert.strictEqual(startData.success, true);
    assert.strictEqual(startData.status.simulationRunning, true);

    // 3. SET RATE
    const rateRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ rateHz: 30 })
    });
    assert.strictEqual(rateRes.status, 200);
    const rateData = await rateRes.json();
    assert.strictEqual(rateData.status.tickRateHz, 30);

    // 4. TOGGLE NOISE
    const noiseRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ noiseEnabled: true })
    });
    assert.strictEqual(noiseRes.status, 200);
    const noiseData = await noiseRes.json();
    assert.strictEqual(noiseData.status.noiseEnabled, true);

    // Reset rate back to 20
    await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ rateHz: 20 })
    });
  });

  test('POST /api/simulation/control validates erroneous payloads', async () => {
    // Empty body
    const emptyRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({})
    });
    assert.strictEqual(emptyRes.status, 400);

    // Invalid command
    const invalidCmdRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: 'FLY_TO_MARS' })
    });
    assert.strictEqual(invalidCmdRes.status, 400);

    // Invalid provider
    const invalidProvRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ provider: 'UNSUPPORTED_PROVIDER' })
    });
    assert.strictEqual(invalidProvRes.status, 400);

    // Invalid rateHz
    const invalidRateRes = await fetch(`${BASE_URL}/api/simulation/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ rateHz: -10 })
    });
    assert.strictEqual(invalidRateRes.status, 400);
  });

  test('GET /api/carla/status returns bridge metadata and status', async () => {
    const res = await fetch(`${BASE_URL}/api/carla/status`);
    assert.strictEqual(res.status, 200);

    const carla = await res.json();
    assert.ok(carla.status !== undefined);
    assert.strictEqual(typeof carla.isInitialized, 'boolean');
    assert.ok(carla.timestamp);
  });

  test('GET / serves frontend index.html static file', async () => {
    const res = await fetch(`${BASE_URL}/`);
    assert.strictEqual(res.status, 200);
    assert.ok(res.headers.get('content-type')?.includes('text/html'));
    const text = await res.text();
    assert.ok(text.includes('GARUDA') || text.length > 0);
  });

  test('WebSocket: manages client connections, handles heartbeats and message exchanges', async () => {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(WS_URL);
      const receivedTypes = new Set();
      let hasSentControl = false;

      ws.on('open', () => {
        // Send a ping message
        ws.send(JSON.stringify({ type: 'PING' }));
      });

      ws.on('message', (raw) => {
        const msg = JSON.parse(raw.toString());
        receivedTypes.add(msg.type);

        if (msg.type === 'PONG') {
          assert.ok(msg.timestamp);
        }

        if (msg.type === 'INIT') {
          assert.ok(msg.data.provider);
          assert.strictEqual(typeof msg.data.rateHz, 'number');
        }

        if (msg.type === 'TELEMETRY') {
          assert.strictEqual(msg.data.header.isSimulated, true);
          assert.strictEqual(
            msg.data.header.disclaimer,
            'SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS'
          );
        }

        // Once we received initial packets, test sending SIM_CONTROL
        if (!hasSentControl && receivedTypes.has('INIT') && receivedTypes.has('TELEMETRY')) {
          hasSentControl = true;
          ws.send(JSON.stringify({ type: 'SIM_CONTROL', command: 'STEP' }));
          ws.send(JSON.stringify({ type: 'SET_RATE', rateHz: 25 }));
          ws.send(JSON.stringify({ type: 'TOGGLE_NOISE', enabled: true }));
          ws.send(JSON.stringify({ type: 'GET_STATUS' }));

          setTimeout(() => {
            ws.close();
          }, 300);
        }
      });

      ws.on('close', () => {
        try {
          assert.ok(receivedTypes.has('INIT'), 'Should have received INIT frame');
          assert.ok(receivedTypes.has('TELEMETRY'), 'Should have received TELEMETRY frame');
          assert.ok(receivedTypes.has('STATUS'), 'Should have received STATUS frame');
          assert.ok(receivedTypes.has('PONG'), 'Should have received PONG frame');
          resolve();
        } catch (err) {
          reject(err);
        }
      });

      ws.on('error', reject);
    });
  });
});
