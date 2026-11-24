/**
 * GARUDA VISION - Synthetic Data Provider Lifecycle Test Suite
 * 
 * Verifies the lifecycle state transitions and methods of SyntheticDataProvider:
 * 1. init(): initializes buffers, odometer, and states.
 * 2. start(): transitions isRunning to true.
 * 3. pause() and resume(): freezes and unfreezes simulation state.
 * 4. reset(): restores vehicle, tick, and sequence back to initial state.
 * 5. step(dt): deterministic incremental simulation advances.
 * 6. getNextTelemetry(): produces standardized telemetry frame.
 * 7. setNoiseEnabled(): toggles sensor noise model.
 * 8. setUpdateRate(): configures target update frequency.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { SyntheticDataProvider } from '../src/simulation/providers/synthetic-provider.js';

describe('SyntheticDataProvider Lifecycle Tests', () => {
  it('should initialize with default states and values', async () => {
    const provider = new SyntheticDataProvider({ updateRateHz: 20 });
    await provider.init();

    assert.equal(provider.isInitialized, true);
    assert.equal(provider.isRunning, false);
    assert.equal(provider.isPaused, false);
    assert.equal(provider.sequenceNumber, 0);
    assert.equal(provider.simulationTick, 0);
    assert.equal(provider.updateRateHz, 20);
    assert.equal(provider.isNoiseEnabled(), false);

    const info = provider.getProviderInfo();
    assert.equal(info.name, 'SyntheticDataProvider');
    assert.equal(info.type, 'SYNTHETIC');
    assert.equal(info.status, 'IDLE');
  });

  it('should correctly handle start(), pause(), and resume() lifecycle states', () => {
    const provider = new SyntheticDataProvider();

    provider.start();
    assert.equal(provider.isRunning, true);
    assert.equal(provider.isPaused, false);
    assert.equal(provider.getProviderInfo().status, 'STREAMING');

    provider.pause();
    assert.equal(provider.isRunning, true);
    assert.equal(provider.isPaused, true);
    assert.equal(provider.getProviderInfo().status, 'PAUSED');

    provider.resume();
    assert.equal(provider.isRunning, true);
    assert.equal(provider.isPaused, false);
    assert.equal(provider.getProviderInfo().status, 'STREAMING');
  });

  it('should advance simulationTick and sequenceNumber on step(dt)', () => {
    const provider = new SyntheticDataProvider();
    provider.start();

    const dt = 0.05; // 50ms (20 Hz)
    provider.step(dt);
    assert.equal(provider.simulationTick, 1);
    assert.equal(provider.sequenceNumber, 1);

    provider.step(dt);
    assert.equal(provider.simulationTick, 2);
    assert.equal(provider.sequenceNumber, 2);

    // Stepping with 0 or negative dt should be a no-op
    provider.step(0);
    assert.equal(provider.simulationTick, 2);
  });

  it('should update vehicle position and odometry during step(dt)', () => {
    const provider = new SyntheticDataProvider();
    provider.start();

    const initialTelemetry = provider.getNextTelemetry();
    const initPos = initialTelemetry.vehicle.position;
    const initDist = initialTelemetry.vehicle.odometry.totalDistanceMeters;

    // Advance 40 ticks (2.0s)
    for (let i = 0; i < 40; i++) {
      provider.step(0.05);
    }

    const updatedTelemetry = provider.getNextTelemetry();
    const updatedPos = updatedTelemetry.vehicle.position;
    const updatedDist = updatedTelemetry.vehicle.odometry.totalDistanceMeters;

    assert.ok(
      updatedDist > initDist,
      `Odometry must increase after steps (initial: ${initDist}, updated: ${updatedDist})`
    );
    assert.ok(
      updatedPos.x !== initPos.x || updatedPos.y !== initPos.y,
      'Vehicle position must change after simulation steps'
    );
  });

  it('should restore initial origin state on reset()', () => {
    const provider = new SyntheticDataProvider();
    provider.start();

    // Advance 30 steps
    for (let i = 0; i < 30; i++) {
      provider.step(0.05);
    }

    assert.ok(provider.simulationTick > 0);
    assert.ok(provider.sequenceNumber > 0);

    // Call reset
    provider.reset();

    assert.equal(provider.simulationTick, 0);
    assert.equal(provider.sequenceNumber, 0);
    assert.equal(provider.isPaused, false);

    const resetTelemetry = provider.getNextTelemetry();
    assert.equal(resetTelemetry.vehicle.odometry.totalDistanceMeters, 0.0);
    assert.equal(resetTelemetry.vehicle.odometry.tripDurationSeconds, 0.0);
  });

  it('should toggle realistic sensor noise on and off', () => {
    const provider = new SyntheticDataProvider();
    assert.equal(provider.isNoiseEnabled(), false);

    provider.setNoiseEnabled(true);
    assert.equal(provider.isNoiseEnabled(), true);

    provider.setNoiseEnabled(false);
    assert.equal(provider.isNoiseEnabled(), false);
  });

  it('should set valid update rates and reject out-of-range rates', () => {
    const provider = new SyntheticDataProvider();
    provider.setUpdateRate(30);
    assert.equal(provider.updateRateHz, 30);

    provider.setUpdateRate(50);
    assert.equal(provider.updateRateHz, 50);

    // Invalid rate should be ignored
    provider.setUpdateRate(-10);
    assert.equal(provider.updateRateHz, 50);

    provider.setUpdateRate(500);
    assert.equal(provider.updateRateHz, 50);
  });

  it('should generate complete, non-empty LiDAR point clouds on getNextTelemetry()', () => {
    const provider = new SyntheticDataProvider();
    provider.step(0.05);
    const telemetry = provider.getNextTelemetry();

    assert.ok(telemetry.lidar);
    assert.equal(telemetry.lidar.sensor, 'GARUDA-LIDAR-32');
    assert.ok(telemetry.lidar.pointCount > 0, 'LiDAR should produce points');
    assert.equal(telemetry.lidar.points.length, telemetry.lidar.pointCount);
  });
});
