/**
 * GARUDA VISION - Obstacle Manager
 * 
 * Manages predefined static and dynamic obstacles with 3D bounding boxes.
 * Updates dynamic obstacles deterministically per tick, evaluates spatial
 * relative metrics (dx, dy, range, bearingDeg, relativeSpeedMps, TTC),
 * and classifies risk levels conforming to the standardized telemetry schema.
 */

import { calculateObstacleMetrics } from '../utils/risk-calculator.js';
import { roundTo, gaussianNoise, clamp } from '../utils/math-utils.js';

export class ObstacleManager {
  constructor() {
    this.obstacles = [];
    this.init();
  }

  /**
   * Initializes predefined static and dynamic obstacles along the urban loop.
   */
  init() {
    this.obstacles = [
      // 1. Static parked vehicle on shoulder (Sector 1)
      {
        id: 'obs-001',
        type: 'VEHICLE',
        isStatic: true,
        initialPosition: { x: 55.0, y: 4.8, z: 0.0 },
        position: { x: 55.0, y: 4.8, z: 0.0 },
        dimensions: { length: 4.6, width: 1.9, height: 1.55 },
        velocity: { vx: 0.0, vy: 0.0, speedKph: 0.0 },
        classificationConfidence: 0.98
      },

      // 2. Dynamic Pedestrian crossing at Sector 4 Crosswalk
      {
        id: 'obs-002',
        type: 'PEDESTRIAN',
        isStatic: false,
        initialPosition: { x: -15.0, y: 278.0, z: 0.0 },
        position: { x: -15.0, y: 278.0, z: 0.0 },
        dimensions: { length: 0.6, width: 0.5, height: 1.75 },
        velocity: { vx: 0.0, vy: 1.3, speedKph: 4.68 },
        crossMinY: 277.0,
        crossMaxY: 293.0,
        crossSpeed: 1.3, // m/s
        direction: 1,
        classificationConfidence: 0.95
      },

      // 3. Dynamic Lead Vehicle on Sector 5 Expressway
      {
        id: 'obs-003',
        type: 'VEHICLE',
        isStatic: false,
        initialPosition: { x: -180.0, y: 225.0, z: 0.0 },
        position: { x: -180.0, y: 225.0, z: 0.0 },
        dimensions: { length: 4.8, width: 2.0, height: 1.6 },
        velocity: { vx: -8.5, vy: -7.2, speedKph: 39.5 },
        pathProgress: 0.0,
        speedMps: 11.0, // ~40 km/h
        classificationConfidence: 0.97
      },

      // 4. Dynamic Cyclist along Sector 3 Tech Avenue
      {
        id: 'obs-004',
        type: 'CYCLIST',
        isStatic: false,
        initialPosition: { x: 145.0, y: 216.0, z: 0.0 },
        position: { x: 145.0, y: 216.0, z: 0.0 },
        dimensions: { length: 1.8, width: 0.7, height: 1.65 },
        velocity: { vx: -3.2, vy: 2.8, speedKph: 15.3 },
        pathProgress: 0.0,
        speedMps: 4.25, // ~15.3 km/h
        classificationConfidence: 0.93
      },

      // 5. Static Construction Barrier near Sector 6 Chicane
      {
        id: 'obs-005',
        type: 'STATIC_BARRIER',
        isStatic: true,
        initialPosition: { x: -245.0, y: 5.0, z: 0.0 },
        position: { x: -245.0, y: 5.0, z: 0.0 },
        dimensions: { length: 2.8, width: 0.8, height: 1.1 },
        velocity: { vx: 0.0, vy: 0.0, speedKph: 0.0 },
        classificationConfidence: 0.99
      },

      // 6. Static Road Debris in Sector 7 Return Loop
      {
        id: 'obs-006',
        type: 'DEBRIS',
        isStatic: true,
        initialPosition: { x: -65.0, y: -18.0, z: 0.0 },
        position: { x: -65.0, y: -18.0, z: 0.0 },
        dimensions: { length: 0.85, width: 0.85, height: 0.35 },
        velocity: { vx: 0.0, vy: 0.0, speedKph: 0.0 },
        classificationConfidence: 0.91
      }
    ];
  }

  /**
   * Resets all obstacle positions to initial state.
   */
  reset() {
    this.init();
  }

  /**
   * Dynamically injects an obstacle (e.g. emergency hazard ahead of vehicle).
   * @param {Object} [customObs]
   * @returns {Object}
   */
  injectObstacle(customObs = {}) {
    const id = customObs.id || `haz-${Date.now().toString().slice(-4)}`;
    const obs = {
      id,
      type: customObs.type || 'VEHICLE',
      isStatic: customObs.isStatic !== undefined ? customObs.isStatic : true,
      initialPosition: { ...(customObs.position || { x: 0, y: 0, z: 0 }) },
      position: { ...(customObs.position || { x: 0, y: 0, z: 0 }) },
      dimensions: {
        length: customObs.dimensions?.length || 4.4,
        width: customObs.dimensions?.width || 1.9,
        height: customObs.dimensions?.height || 1.5
      },
      velocity: {
        vx: customObs.velocity?.vx || 0.0,
        vy: customObs.velocity?.vy || 0.0,
        speedKph: customObs.velocity?.speedKph || 0.0
      },
      classificationConfidence: customObs.classificationConfidence || 0.99
    };
    this.obstacles.push(obs);
    return obs;
  }

  /**
   * Advances dynamic obstacles by time delta dt.
   * @param {number} dt 
   */
  step(dt) {
    if (dt <= 0) return;

    for (const obs of this.obstacles) {
      if (obs.isStatic) continue;

      if (obs.id === 'obs-002') {
        // Pedestrian walking back and forth across crosswalk
        obs.position.y += obs.direction * obs.crossSpeed * dt;
        obs.velocity.vy = obs.direction * obs.crossSpeed;
        obs.velocity.speedKph = roundTo(Math.abs(obs.velocity.vy) * 3.6, 1);

        if (obs.position.y >= obs.crossMaxY) {
          obs.position.y = obs.crossMaxY;
          obs.direction = -1;
        } else if (obs.position.y <= obs.crossMinY) {
          obs.position.y = obs.crossMinY;
          obs.direction = 1;
        }
      } else if (obs.id === 'obs-003') {
        // Lead Vehicle on expressway: travels along curved arc
        obs.pathProgress += dt * 0.035; // cycle progress
        const angle = obs.pathProgress * Math.PI;
        // Motion along Sector 5 curved bypass
        const baseSpeed = obs.speedMps;
        obs.position.x = -200.0 + Math.cos(angle) * 45.0;
        obs.position.y = 190.0 + Math.sin(angle) * 35.0;
        obs.velocity.vx = roundTo(-Math.sin(angle) * 45.0 * 0.035 * Math.PI, 2);
        obs.velocity.vy = roundTo(Math.cos(angle) * 35.0 * 0.035 * Math.PI, 2);
        obs.velocity.speedKph = roundTo(Math.hypot(obs.velocity.vx, obs.velocity.vy) * 3.6, 1);
      } else if (obs.id === 'obs-004') {
        // Cyclist along Sector 3 Avenue
        obs.pathProgress = (obs.pathProgress + dt * 0.025) % 1.0;
        // Linear path from (165, 195) to (50, 275)
        const t = obs.pathProgress;
        obs.position.x = 165.0 - t * 115.0;
        obs.position.y = 195.0 + t * 80.0;
        obs.velocity.vx = roundTo(-115.0 * 0.025, 2);
        obs.velocity.vy = roundTo(80.0 * 0.025, 2);
        obs.velocity.speedKph = roundTo(Math.hypot(obs.velocity.vx, obs.velocity.vy) * 3.6, 1);
      }
    }
  }

  /**
   * Evaluates and formats obstacle list with relative metrics against ego vehicle.
   * 
   * @param {Object} egoTelemetry - Current vehicle telemetry
   * @param {boolean} [noiseEnabled=false]
   * @param {number} [maxRangeMeters=70.0] - Perception filter horizon
   * @returns {Array<Object>} Conforming to docs/TELEMETRY_SCHEMA.md
   */
  getTrackedObstacles(egoTelemetry, noiseEnabled = false, maxRangeMeters = 70.0) {
    const ego = {
      position: egoTelemetry.vehicle.position,
      orientation: egoTelemetry.vehicle.orientation,
      motion: egoTelemetry.vehicle.motion
    };

    const tracked = [];

    for (const obs of this.obstacles) {
      let posX = obs.position.x;
      let posY = obs.position.y;
      let posZ = obs.position.z;

      if (noiseEnabled) {
        posX += gaussianNoise(0, 0.04);
        posY += gaussianNoise(0, 0.04);
      }

      const simObs = {
        position: { x: posX, y: posY, z: posZ },
        velocity: obs.velocity
      };

      const metrics = calculateObstacleMetrics(ego, simObs);

      // Perception filter: include if within range horizon
      if (metrics.relativePosition.range <= maxRangeMeters) {
        tracked.push({
          id: obs.id,
          type: obs.type,
          position: {
            x: roundTo(posX, 2),
            y: roundTo(posY, 2),
            z: roundTo(posZ, 2)
          },
          dimensions: { ...obs.dimensions },
          relativePosition: metrics.relativePosition,
          velocity: {
            vx: roundTo(obs.velocity.vx, 1),
            vy: roundTo(obs.velocity.vy, 1),
            speedKph: roundTo(obs.velocity.speedKph, 1)
          },
          relativeSpeedMps: metrics.relativeSpeedMps,
          timeToCollisionSec: metrics.timeToCollisionSec,
          riskLevel: metrics.riskLevel,
          classificationConfidence: roundTo(obs.classificationConfidence, 2)
        });
      }
    }

    // Sort by range ascending (closest obstacles first)
    tracked.sort((a, b) => a.relativePosition.range - b.relativePosition.range);

    return tracked;
  }
}
