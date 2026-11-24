/**
 * GARUDA VISION - Deterministic Synthetic Telemetry Provider
 * 
 * Generates mathematically consistent vehicle kinematics, obstacle coordinates,
 * 32-beam simulated LiDAR point clouds, risk metrics, and alerts.
 * Strictly adheres to docs/TELEMETRY_SCHEMA.md.
 */

import { DataProvider } from './data-provider.interface.js';
import { VehicleModel } from '../models/vehicle-model.js';
import { ObstacleManager } from '../models/obstacle-manager.js';
import { LidarSimulator } from '../models/lidar-simulator.js';
import { generateAlerts } from '../utils/risk-calculator.js';

export class SyntheticDataProvider extends DataProvider {
  constructor(options = {}) {
    super('SyntheticDataProvider');
    this.updateRateHz = options.updateRateHz ?? 20;

    this.vehicleModel = new VehicleModel(options.vehicleId ?? 'GARUDA-EGO-01');
    this.obstacleManager = new ObstacleManager();
    this.lidarSimulator = new LidarSimulator({
      sensor: options.lidarSensor ?? 'GARUDA-LIDAR-32'
    });

    this.init();
  }

  /**
   * Initializes or re-initializes the synthetic environment.
   */
  async init() {
    this.sequenceNumber = 0;
    this.simulationTick = 0;
    this.vehicleModel.reset();
    this.obstacleManager.reset();
    this.isInitialized = true;
    this.isRunning = false;
    this.isPaused = false;
  }

  /**
   * Starts telemetry generation.
   */
  start() {
    this.isRunning = true;
    this.isPaused = false;
  }

  /**
   * Pauses telemetry generation.
   */
  pause() {
    this.isPaused = true;
  }

  /**
   * Resumes paused telemetry generation.
   */
  resume() {
    if (this.isRunning) {
      this.isPaused = false;
    }
  }

  /**
   * Resets simulation clock and states.
   */
  reset() {
    this.sequenceNumber = 0;
    this.simulationTick = 0;
    this.vehicleModel.reset();
    this.obstacleManager.reset();
    this.isPaused = false;
  }

  /**
   * Advances simulation kinematics and models by dt seconds.
   * @param {number} dt 
   */
  step(dt) {
    if (dt <= 0) return;
    this.simulationTick++;
    this.sequenceNumber++;

    // 1. Advance vehicle kinematics along deterministic spline
    this.vehicleModel.step(dt);

    // 2. Advance dynamic obstacles
    this.obstacleManager.step(dt);
  }

  /**
   * Dynamically injects an obstacle into the simulation.
   * @param {Object} [customObs]
   * @returns {Object}
   */
  injectObstacle(customObs) {
    return this.obstacleManager.injectObstacle(customObs);
  }

  /**
   * Toggles realistic sensor noise.
   * @param {boolean} enabled 
   */
  setNoiseEnabled(enabled) {
    super.setNoiseEnabled(enabled);
    this.vehicleModel.setNoiseEnabled(this.noiseEnabled);
  }

  /**
   * Generates and returns the complete standardized telemetry payload.
   * @returns {Object} Strictly conforming to docs/TELEMETRY_SCHEMA.md
   */
  getNextTelemetry() {
    const now = new Date();
    const timestampIso = now.toISOString();
    const timestampMs = now.getTime();

    // 1. Ego vehicle telemetry & diagnostics
    const vehicleData = this.vehicleModel.getTelemetryData();

    // 2. Relative obstacle coordinates and risk assessment
    const obstacles = this.obstacleManager.getTrackedObstacles(vehicleData, this.noiseEnabled);

    // 3. 32-beam LiDAR point cloud
    const lidar = this.lidarSimulator.generatePointCloud(vehicleData, obstacles, this.noiseEnabled);

    // 4. Collision and subsystem alerts
    const alerts = generateAlerts(
      obstacles,
      vehicleData.diagnostics,
      vehicleData.vehicle.motion,
      timestampIso
    );

    // 5. Construct standardized telemetry frame
    return {
      header: {
        sequenceNumber: this.sequenceNumber,
        timestamp: timestampIso,
        timestampMs,
        dataSource: 'SYNTHETIC',
        isSimulated: true,
        simulationTick: this.simulationTick,
        updateRateHz: this.updateRateHz,
        disclaimer: 'SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS'
      },
      vehicle: vehicleData.vehicle,
      diagnostics: vehicleData.diagnostics,
      obstacles,
      lidar,
      alerts,
      navigation: vehicleData.navigation
    };
  }

  /**
   * Returns provider information and status.
   */
  getProviderInfo() {
    return {
      name: this.name,
      type: 'SYNTHETIC',
      isInitialized: this.isInitialized,
      isRunning: this.isRunning,
      isPaused: this.isPaused,
      updateRateHz: this.updateRateHz,
      noiseEnabled: this.noiseEnabled,
      status: this.isRunning ? (this.isPaused ? 'PAUSED' : 'STREAMING') : 'IDLE',
      details: {
        trajectoryLengthMeters: this.vehicleModel.totalPathLength,
        totalWaypoints: this.vehicleModel.totalWaypoints,
        totalTrackedObstacles: this.obstacleManager.obstacles.length,
        lidarBeams: this.lidarSimulator.beamCount
      }
    };
  }
}
