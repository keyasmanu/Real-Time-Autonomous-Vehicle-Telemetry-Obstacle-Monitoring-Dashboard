/**
 * GARUDA VISION - Data Provider Contract & Polymorphism Test Suite
 * 
 * Verifies that both SyntheticDataProvider and CarlaDataProvider strictly adhere
 * to the DataProvider base interface and produce interchangeable, standardized
 * telemetry payloads conforming to docs/TELEMETRY_SCHEMA.md.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { DataProvider } from '../src/simulation/providers/data-provider.interface.js';
import { SyntheticDataProvider } from '../src/simulation/providers/synthetic-provider.js';
import { CarlaDataProvider } from '../src/simulation/providers/carla-provider.js';

describe('DataProvider Interface & Polymorphism Contract Tests', () => {
  it('should throw TypeError when attempting to instantiate abstract DataProvider directly', () => {
    assert.throws(
      () => new DataProvider(),
      {
        name: 'TypeError',
        message: /Cannot construct DataProvider instances directly/
      }
    );
  });

  const providers = [
    { name: 'SyntheticDataProvider', create: () => new SyntheticDataProvider() },
    { name: 'CarlaDataProvider', create: () => new CarlaDataProvider() }
  ];

  for (const { name, create } of providers) {
    describe(`${name} Contract Compliance`, () => {
      it('should inherit from DataProvider', () => {
        const instance = create();
        assert.ok(instance instanceof DataProvider, `${name} must inherit from DataProvider`);
      });

      it('should implement all mandatory lifecycle methods', () => {
        const instance = create();
        const requiredMethods = [
          'init',
          'start',
          'pause',
          'resume',
          'reset',
          'step',
          'getNextTelemetry',
          'setNoiseEnabled',
          'isNoiseEnabled',
          'setUpdateRate',
          'getProviderInfo'
        ];

        for (const method of requiredMethods) {
          assert.equal(
            typeof instance[method],
            'function',
            `${name} must implement method: ${method}()`
          );
        }
      });

      it('should emit standardized telemetry containing all top-level schema blocks', () => {
        const instance = create();
        instance.step(0.05);
        const payload = instance.getNextTelemetry();

        const requiredTopLevelKeys = [
          'header',
          'vehicle',
          'diagnostics',
          'obstacles',
          'lidar',
          'alerts',
          'navigation'
        ];

        for (const key of requiredTopLevelKeys) {
          assert.ok(
            payload[key] !== undefined,
            `${name} getNextTelemetry() must contain top-level block: ${key}`
          );
        }
      });

      it('should return well-formed provider metadata via getProviderInfo()', () => {
        const instance = create();
        const info = instance.getProviderInfo();

        assert.equal(typeof info.name, 'string');
        assert.equal(typeof info.type, 'string');
        assert.equal(typeof info.isInitialized, 'boolean');
        assert.equal(typeof info.isRunning, 'boolean');
        assert.equal(typeof info.isPaused, 'boolean');
        assert.equal(typeof info.updateRateHz, 'number');
        assert.equal(typeof info.noiseEnabled, 'boolean');
        assert.equal(typeof info.status, 'string');
      });

      it('should support polymorphism: interchangeable stepping and telemetry generation', () => {
        // Consumer function that knows nothing about the concrete provider
        function runSimulationTick(provider, dt) {
          provider.step(dt);
          return provider.getNextTelemetry();
        }

        const instance = create();
        instance.start();

        const frame1 = runSimulationTick(instance, 0.05);
        assert.ok(frame1);
        assert.equal(frame1.header.isSimulated, true);
        assert.ok(frame1.vehicle.position);
      });
    });
  }

  describe('CarlaDataProvider Bridge & Ingestion Tests', () => {
    it('should operate in FALLBACK_SIMULATION mode when no external bridge is connected', () => {
      const carla = new CarlaDataProvider();
      const info = carla.getProviderInfo();

      assert.equal(info.status, 'FALLBACK_SIMULATION');
      assert.equal(info.details.isFallbackMode, true);
    });

    it('should ingest and normalize raw CARLA packets when external bridge streams data', () => {
      const carla = new CarlaDataProvider();

      const rawCarlaPacket = {
        transform: {
          location: { x: 50.5, y: -25.2, z: 0.1 },
          rotation: { yaw: 30.0, pitch: 0.0, roll: 0.0 }
        },
        velocity: { x: 12.0, y: -5.0, z: 0.0 },
        control: { throttle: 0.45, brake: 0.0, steer: 0.05, gear: 1 },
        obstacles: [
          {
            id: 'carla-actor-99',
            type: 'vehicle',
            location: { x: 65.0, y: -25.2, z: 0.0 },
            velocity: { x: 8.0, y: -5.0 }
          }
        ],
        lidar: {
          sensor: 'CARLA-RAYCAST-32',
          pointCount: 1,
          points: [[1.0, 0.0, -0.5, 0.9]]
        }
      };

      carla.ingestCarlaPacket(rawCarlaPacket);
      const info = carla.getProviderInfo();
      assert.equal(info.status, 'CONNECTED');

      const normalized = carla.getNextTelemetry();
      assert.equal(normalized.header.dataSource, 'CARLA_BRIDGE');
      assert.equal(normalized.vehicle.id, 'CARLA-EGO-VEHICLE');
      assert.equal(normalized.vehicle.position.x, 50.5);
      // Verify CARLA Unreal Y (-25.2) is inverted to ISO 8855 Left (+25.2)
      assert.equal(normalized.vehicle.position.y, 25.2);
    });
  });
});
