/**
 * GARUDA VISION - Simulation Engine & Orchestration Manager
 * 
 * Central orchestrator managing telemetry data sources (SyntheticDataProvider,
 * CarlaDataProvider), timer tick loop (10-50 Hz, default 20 Hz), simulation
 * lifecycle controls (START, PAUSE, RESUME, RESET, STEP), and real-time telemetry dispatch.
 */

import { EventEmitter } from 'events';
import { SyntheticDataProvider } from './providers/synthetic-provider.js';
import { CarlaDataProvider } from './providers/carla-provider.js';
import { DataProvider } from './providers/data-provider.interface.js';
import { VehicleModel } from './models/vehicle-model.js';
import { ObstacleManager } from './models/obstacle-manager.js';
import { LidarSimulator } from './models/lidar-simulator.js';
import * as MathUtils from './utils/math-utils.js';
import * as RiskCalculator from './utils/risk-calculator.js';

export class SimulationManager extends EventEmitter {
  /**
   * @param {Object} [options]
   * @param {number} [options.updateRateHz=20] - Telemetry frequency (10-50 Hz)
   * @param {"SYNTHETIC" | "CARLA"} [options.provider="SYNTHETIC"] - Initial provider
   * @param {boolean} [options.noiseEnabled=false] - Initial sensor noise toggle
   * @param {boolean} [options.autoStart=false] - Whether to start loop immediately
   */
  constructor(options = {}) {
    super();

    this.updateRateHz = MathUtils.clamp(options.updateRateHz ?? 20, 10, 50);
    this.providerType = (options.provider || 'SYNTHETIC').toUpperCase();
    this.noiseEnabled = Boolean(options.noiseEnabled);

    // Initialize provider instances
    this.syntheticProvider = new SyntheticDataProvider({ updateRateHz: this.updateRateHz });
    this.carlaProvider = new CarlaDataProvider({ updateRateHz: this.updateRateHz });

    // Set initial active provider
    this.activeProvider = this.providerType === 'CARLA' ? this.carlaProvider : this.syntheticProvider;
    this.activeProvider.setNoiseEnabled(this.noiseEnabled);

    // Timer and loop state
    this.timerId = null;
    this.isRunning = false;
    this.isPaused = false;
    this.frameCount = 0;
    this.latestTelemetry = null;

    // Cache initial frame
    this.latestTelemetry = this.activeProvider.getNextTelemetry();

    if (options.autoStart) {
      this.start();
    }
  }

  /**
   * Starts the simulation tick loop at configured updateRateHz.
   */
  start() {
    if (this.isRunning && !this.isPaused) return;

    this.isRunning = true;
    this.isPaused = false;
    this.activeProvider.start();

    this._restartTimer();
    this.emit('start', { provider: this.providerType, updateRateHz: this.updateRateHz });
  }

  /**
   * Pauses the simulation tick loop.
   */
  pause() {
    if (this.isPaused) return;

    this.isPaused = true;
    this.activeProvider.pause();
    this._stopTimer();
    this.emit('pause', { frameCount: this.frameCount });
  }

  /**
   * Resumes a paused simulation.
   */
  resume() {
    if (!this.isPaused) return;

    this.isRunning = true;
    this.isPaused = false;
    this.activeProvider.resume();
    this._restartTimer();
    this.emit('resume', { frameCount: this.frameCount });
  }

  /**
   * Resets the simulation clock and active provider state.
   */
  reset() {
    this.frameCount = 0;
    this.activeProvider.reset();
    this.latestTelemetry = this.activeProvider.getNextTelemetry();
    this.emit('reset', { frameCount: 0 });
    this.emit('telemetry', this.latestTelemetry);
  }

  /**
   * Executes a single simulation step.
   * Useful when simulation is paused or running in lockstep.
   * 
   * @param {number} [customDt] - Time step in seconds (defaults to 1 / updateRateHz)
   * @returns {Object} Latest telemetry frame
   */
  step(customDt) {
    const dt = customDt ?? (1.0 / this.updateRateHz);
    this.activeProvider.step(dt);
    this.frameCount++;
    this.latestTelemetry = this.activeProvider.getNextTelemetry();

    this.emit('telemetry', this.latestTelemetry);
    this.emit('step', { frameCount: this.frameCount, dt });
    return this.latestTelemetry;
  }

  /**
   * Internal tick handler invoked by setInterval timer.
   * @private
   */
  _tick() {
    if (!this.isRunning || this.isPaused) return;

    const dt = 1.0 / this.updateRateHz;
    this.activeProvider.step(dt);
    this.frameCount++;
    this.latestTelemetry = this.activeProvider.getNextTelemetry();

    this.emit('telemetry', this.latestTelemetry);
  }

  /**
   * Injects an obstacle into the active simulation provider.
   * @param {Object} [obstacleData]
   * @returns {Object}
   */
  injectObstacle(obstacleData) {
    if (this.activeProvider && typeof this.activeProvider.injectObstacle === 'function') {
      const injected = this.activeProvider.injectObstacle(obstacleData);
      this.emit('obstacleInjected', injected);
      return injected;
    }
    return null;
  }

  /**
   * Configures target update frequency between 10 Hz and 50 Hz.
   * @param {number} rateHz 
   */
  setUpdateRate(rateHz) {
    const clampedRate = MathUtils.clamp(Number(rateHz) || 20, 10, 50);
    if (this.updateRateHz === clampedRate) return;

    this.updateRateHz = clampedRate;
    this.syntheticProvider.setUpdateRate(clampedRate);
    this.carlaProvider.setUpdateRate(clampedRate);

    if (this.isRunning && !this.isPaused) {
      this._restartTimer();
    }

    this.emit('rateChange', { updateRateHz: this.updateRateHz });
  }

  /**
   * Switches the active data provider ("SYNTHETIC" or "CARLA").
   * @param {"SYNTHETIC" | "CARLA"} providerType 
   */
  setProvider(providerType) {
    const targetType = (providerType || '').toUpperCase();
    if (targetType !== 'SYNTHETIC' && targetType !== 'CARLA') {
      throw new Error(`Unsupported provider type: ${providerType}. Must be "SYNTHETIC" or "CARLA".`);
    }

    if (this.providerType === targetType) return;

    const wasRunning = this.isRunning && !this.isPaused;

    // Switch provider reference
    this.providerType = targetType;
    this.activeProvider = targetType === 'CARLA' ? this.carlaProvider : this.syntheticProvider;
    this.activeProvider.setNoiseEnabled(this.noiseEnabled);
    this.activeProvider.setUpdateRate(this.updateRateHz);

    if (wasRunning) {
      this.activeProvider.start();
    }

    this.latestTelemetry = this.activeProvider.getNextTelemetry();
    this.emit('providerChange', { provider: this.providerType });
    this.emit('telemetry', this.latestTelemetry);
  }

  /**
   * Toggles realistic sensor noise.
   * @param {boolean} enabled 
   */
  setNoiseEnabled(enabled) {
    this.noiseEnabled = Boolean(enabled);
    this.activeProvider.setNoiseEnabled(this.noiseEnabled);
    this.emit('noiseChange', { noiseEnabled: this.noiseEnabled });
  }

  /**
   * Convenience alias for setNoiseEnabled.
   * @param {boolean} enabled 
   */
  toggleNoise(enabled) {
    this.setNoiseEnabled(enabled !== undefined ? enabled : !this.noiseEnabled);
  }

  /**
   * Returns latest cached telemetry frame.
   * @returns {Object} Standardized telemetry payload
   */
  getLatestTelemetry() {
    if (!this.latestTelemetry) {
      this.latestTelemetry = this.activeProvider.getNextTelemetry();
    }
    return this.latestTelemetry;
  }

  /**
   * Returns current status summary.
   * @returns {Object}
   */
  getStatus() {
    return {
      isRunning: this.isRunning,
      isPaused: this.isPaused,
      updateRateHz: this.updateRateHz,
      activeProvider: this.providerType,
      noiseEnabled: this.noiseEnabled,
      frameCount: this.frameCount,
      providerInfo: this.activeProvider.getProviderInfo()
    };
  }

  /**
   * Returns detailed CARLA bridge status.
   * @returns {Object}
   */
  getCarlaStatus() {
    return this.carlaProvider.getProviderInfo();
  }

  /**
   * Stops timer loop and cleans up resources.
   */
  destroy() {
    this._stopTimer();
    this.isRunning = false;
    this.isPaused = false;
    this.removeAllListeners();
  }

  /**
   * Restarts the internal timer loop.
   * @private
   */
  _restartTimer() {
    this._stopTimer();
    const intervalMs = Math.round(1000.0 / this.updateRateHz);
    this.timerId = setInterval(() => this._tick(), intervalMs);
  }

  /**
   * Stops the internal timer loop.
   * @private
   */
  _stopTimer() {
    if (this.timerId) {
      clearInterval(this.timerId);
      this.timerId = null;
    }
  }
}

// Re-export core modules and utilities
export {
  SyntheticDataProvider,
  CarlaDataProvider,
  DataProvider,
  VehicleModel,
  ObstacleManager,
  LidarSimulator,
  MathUtils,
  RiskCalculator
};

export default SimulationManager;
