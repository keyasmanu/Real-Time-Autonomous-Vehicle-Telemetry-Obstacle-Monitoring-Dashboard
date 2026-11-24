/**
 * GARUDA VISION - Modular CARLA Data Provider & Bridge Adapter
 * 
 * Provides connectivity and normalization for the CARLA Autonomous Driving
 * Simulator Python bridge. When a live CARLA bridge is connected, incoming
 * actor and sensor packets are normalized to the standardized schema.
 * When offline, it operates in a high-fidelity CARLA Fallback Simulation mode
 * simulating CARLA Town03 autopilot kinematics and ray-cast LiDAR.
 * 
 * Sets header.dataSource = "CARLA_BRIDGE" and isSimulated = true.
 */

import { DataProvider } from './data-provider.interface.js';
import { VehicleModel } from '../models/vehicle-model.js';
import { ObstacleManager } from '../models/obstacle-manager.js';
import { LidarSimulator } from '../models/lidar-simulator.js';
import { generateAlerts } from '../utils/risk-calculator.js';
import { roundTo } from '../utils/math-utils.js';

export class CarlaDataProvider extends DataProvider {
  /**
   * @param {Object} [options]
   * @param {string} [options.host='127.0.0.1']
   * @param {number} [options.port=2000]
   * @param {number} [options.bridgePort=8081]
   * @param {string} [options.town='Town03']
   * @param {string} [options.vehicleBlueprint='vehicle.tesla.model3']
   */
  constructor(options = {}) {
    super('CarlaDataProvider');
    this.updateRateHz = options.updateRateHz ?? 20;

    // Bridge connectivity config
    this.bridgeHost = options.host ?? '127.0.0.1';
    this.bridgePort = options.bridgePort ?? 8081;
    this.carlaServerPort = options.port ?? 2000;
    this.carlaTown = options.town ?? 'Town03';
    this.vehicleBlueprint = options.vehicleBlueprint ?? 'vehicle.tesla.model3';
    this.lidarBlueprint = 'sensor.lidar.ray_cast';

    // Bridge status
    this.connectionStatus = 'FALLBACK_SIMULATION'; // 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'FALLBACK_SIMULATION'
    this.bridgeHealth = 'STANDBY'; // 'ONLINE' | 'STANDBY' | 'DEGRADED' | 'OFFLINE'
    this.lastLatencyMs = 2.4;
    this.lastPacketTimestamp = null;
    this.latestExternalPacket = null;

    // Internal fallback simulation models
    this.fallbackVehicle = new VehicleModel('CARLA-HERO-TESLA-M3');
    this.fallbackObstacles = new ObstacleManager();
    this.fallbackLidar = new LidarSimulator({ sensor: 'CARLA-RAYCAST-32' });

    this.init();
  }

  /**
   * Initializes the CARLA provider.
   */
  async init() {
    this.sequenceNumber = 0;
    this.simulationTick = 0;
    this.fallbackVehicle.reset();
    this.fallbackObstacles.reset();
    this.isInitialized = true;
    this.isRunning = false;
    this.isPaused = false;
  }

  start() {
    this.isRunning = true;
    this.isPaused = false;
  }

  pause() {
    this.isPaused = true;
  }

  resume() {
    if (this.isRunning) {
      this.isPaused = false;
    }
  }

  reset() {
    this.sequenceNumber = 0;
    this.simulationTick = 0;
    this.fallbackVehicle.reset();
    this.fallbackObstacles.reset();
    this.latestExternalPacket = null;
    this.isPaused = false;
  }

  /**
   * Dynamically injects an obstacle into the simulation.
   * @param {Object} [customObs]
   * @returns {Object}
   */
  injectObstacle(customObs) {
    return this.fallbackObstacles.injectObstacle(customObs);
  }

  /**
   * Advances simulation kinematics.
   * @param {number} dt 
   */
  step(dt) {
    if (dt <= 0) return;
    this.simulationTick++;
    this.sequenceNumber++;

    if (this.connectionStatus === 'FALLBACK_SIMULATION') {
      this.fallbackVehicle.step(dt);
      this.fallbackObstacles.step(dt);
    }
  }

  /**
   * Ingests a raw or semi-processed packet from an external CARLA Python bridge.
   * Switches provider into 'CONNECTED' state.
   * 
   * @param {Object} rawPacket 
   */
  ingestCarlaPacket(rawPacket) {
    if (!rawPacket) return;
    this.latestExternalPacket = rawPacket;
    this.connectionStatus = 'CONNECTED';
    this.bridgeHealth = 'ONLINE';
    this.lastPacketTimestamp = Date.now();
    this.lastLatencyMs = roundTo(Math.random() * 2.0 + 1.2, 1);
  }

  /**
   * Generates next standardized telemetry frame.
   * If live external CARLA packet is available, normalizes it.
   * Otherwise uses fallback CARLA simulation model.
   * 
   * @returns {Object} Conforming to docs/TELEMETRY_SCHEMA.md
   */
  getNextTelemetry() {
    const now = new Date();
    const timestampIso = now.toISOString();
    const timestampMs = now.getTime();

    // Check if live connection went stale (> 2000ms without packet)
    if (this.connectionStatus === 'CONNECTED' && this.lastPacketTimestamp) {
      if (Date.now() - this.lastPacketTimestamp > 2000) {
        this.connectionStatus = 'FALLBACK_SIMULATION';
        this.bridgeHealth = 'STANDBY';
      }
    }

    if (this.latestExternalPacket && this.connectionStatus === 'CONNECTED') {
      return this.normalizeCarlaPacket(this.latestExternalPacket, timestampIso, timestampMs);
    }

    // High-fidelity CARLA Fallback Mode
    const vehicleData = this.fallbackVehicle.getTelemetryData();
    vehicleData.vehicle.id = 'CARLA-EGO-TOWN03';
    vehicleData.vehicle.status = 'AUTONOMOUS_ENGAGED';
    vehicleData.diagnostics.subsystems.carlaBridge = 'ONLINE';

    const obstacles = this.fallbackObstacles.getTrackedObstacles(vehicleData, this.noiseEnabled);
    const lidar = this.fallbackLidar.generatePointCloud(vehicleData, obstacles, this.noiseEnabled);
    const alerts = generateAlerts(
      obstacles,
      vehicleData.diagnostics,
      vehicleData.vehicle.motion,
      timestampIso
    );

    return {
      header: {
        sequenceNumber: this.sequenceNumber,
        timestamp: timestampIso,
        timestampMs,
        dataSource: 'CARLA_BRIDGE',
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
   * Normalizes incoming raw CARLA Python bridge packet into the standardized schema.
   * 
   * @param {Object} raw - Raw CARLA packet
   * @param {string} timestampIso 
   * @param {number} timestampMs 
   * @returns {Object}
   */
  normalizeCarlaPacket(raw, timestampIso, timestampMs) {
    const egoTransform = raw.transform || raw.vehicle?.transform || {};
    const egoVel = raw.velocity || raw.vehicle?.velocity || {};
    const egoControl = raw.control || raw.vehicle?.control || {};

    // CARLA to ISO 8855 coordinate transformation
    // CARLA Unreal: +X Forward, +Y Right, +Z Up.
    // ISO 8855: +X Forward, +Y Left (Invert Y from Unreal), +Z Up.
    const x = roundTo(egoTransform.location?.x ?? raw.vehicle?.position?.x ?? 0.0, 2);
    const y = roundTo(
      egoTransform.location?.y !== undefined ? -egoTransform.location.y : (raw.vehicle?.position?.y ?? 0.0),
      2
    );
    const z = roundTo(egoTransform.location?.z ?? raw.vehicle?.position?.z ?? 0.0, 2);

    const yaw = roundTo(
      egoTransform.rotation?.yaw !== undefined ? -egoTransform.rotation.yaw : (raw.vehicle?.orientation?.yaw ?? 0.0),
      1
    );

    const speedMps = roundTo(
      Math.hypot(egoVel.x || 0, egoVel.y || 0, egoVel.z || 0) || raw.vehicle?.motion?.speedMps || 0.0,
      2
    );
    const speedKph = roundTo(speedMps * 3.6, 1);

    return {
      header: {
        sequenceNumber: raw.header?.sequenceNumber ?? this.sequenceNumber,
        timestamp: timestampIso,
        timestampMs,
        dataSource: 'CARLA_BRIDGE',
        isSimulated: true,
        simulationTick: this.simulationTick,
        updateRateHz: this.updateRateHz,
        disclaimer: 'SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS'
      },
      vehicle: {
        id: raw.vehicle?.id ?? raw.vehicleId ?? 'CARLA-EGO-VEHICLE',
        status: raw.vehicle?.status ?? 'AUTONOMOUS_ENGAGED',
        position: { x, y, z },
        orientation: {
          yaw,
          pitch: roundTo(egoTransform.rotation?.pitch ?? 0.0, 1),
          roll: roundTo(egoTransform.rotation?.roll ?? 0.0, 1),
          headingDeg: yaw,
          headingRad: roundTo((yaw * Math.PI) / 180.0, 4)
        },
        motion: {
          speedKph,
          speedMps,
          acceleration: roundTo(raw.vehicle?.motion?.acceleration ?? 0.0, 2),
          jerk: roundTo(raw.vehicle?.motion?.jerk ?? 0.0, 2),
          steeringAngleDeg: roundTo((egoControl.steer ?? 0.0) * 35.0, 1),
          angularVelocityDeg: roundTo(raw.vehicle?.motion?.angularVelocityDeg ?? 0.0, 1),
          throttlePct: roundTo((egoControl.throttle ?? 0.0) * 100.0, 1),
          brakePct: roundTo((egoControl.brake ?? 0.0) * 100.0, 1),
          gear: egoControl.reverse ? 'R' : (egoControl.gear === 0 ? 'N' : 'D')
        },
        odometry: {
          totalDistanceMeters: roundTo(raw.vehicle?.odometry?.totalDistanceMeters ?? 0.0, 1),
          tripDurationSeconds: roundTo(raw.vehicle?.odometry?.tripDurationSeconds ?? 0.0, 1)
        }
      },
      diagnostics: raw.diagnostics || {
        systemHealth: 'OPTIMAL',
        cpuUsagePct: 24.5,
        batteryVoltage: 400.0,
        batteryLevelPct: 88.0,
        motorTempC: 45.0,
        computeLatencyMs: this.lastLatencyMs,
        subsystems: {
          perception: 'HEALTHY',
          localization: 'HEALTHY',
          planning: 'HEALTHY',
          control: 'HEALTHY',
          lidarSensor: 'ONLINE',
          radarSensor: 'ONLINE',
          cameraArray: 'ONLINE'
        }
      },
      obstacles: Array.isArray(raw.obstacles) ? raw.obstacles : [],
      lidar: raw.lidar || {
        sensor: 'CARLA-LIDAR-32',
        pointCount: 0,
        rangeMeters: 60.0,
        fieldOfViewDeg: 360,
        points: []
      },
      alerts: Array.isArray(raw.alerts) ? raw.alerts : [],
      navigation: raw.navigation || {
        currentWaypointIndex: 0,
        totalWaypoints: 40,
        currentRoadSegment: 'CARLA Town03 Expressway',
        lanePosition: 'CENTER',
        distanceToNextTurnMeters: 100.0,
        targetSpeedKph: 50.0
      }
    };
  }

  /**
   * Returns provider and bridge status.
   */
  getProviderInfo() {
    return {
      name: this.name,
      type: 'CARLA',
      isInitialized: this.isInitialized,
      isRunning: this.isRunning,
      isPaused: this.isPaused,
      updateRateHz: this.updateRateHz,
      noiseEnabled: this.noiseEnabled,
      status: this.connectionStatus,
      details: {
        bridgeHost: this.bridgeHost,
        bridgePort: this.bridgePort,
        carlaServerPort: this.carlaServerPort,
        carlaTown: this.carlaTown,
        vehicleBlueprint: this.vehicleBlueprint,
        lidarBlueprint: this.lidarBlueprint,
        bridgeHealth: this.bridgeHealth,
        lastLatencyMs: this.lastLatencyMs,
        isFallbackMode: this.connectionStatus === 'FALLBACK_SIMULATION'
      }
    };
  }
}
