/**
 * GARUDA VISION - Risk Assessment & Collision Warning Test Suite
 * 
 * Verifies:
 * 1. 2D and 3D Euclidean distance calculations.
 * 2. World-to-Vehicle frame transformations (ISO 8855: +X forward, +Y left).
 * 3. Relative closing speed calculation along line-of-sight vector.
 * 4. Time-To-Collision (TTC) formula (TTC = range / |relativeSpeed|).
 * 5. Risk classification thresholds (SAFE, ATTENTION, WARNING, CRITICAL).
 * 6. Alert generation and severity ordering.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  calculateObstacleMetrics,
  classifyRisk,
  generateAlerts
} from '../src/simulation/utils/risk-calculator.js';
import {
  vec2Distance,
  vec3Distance,
  worldToVehicle2D,
  degToRad
} from '../src/simulation/utils/math-utils.js';

describe('Collision Risk & TTC Engine Tests', () => {
  it('should calculate 2D and 3D Euclidean distances correctly', () => {
    const p1 = { x: 0, y: 0, z: 0 };
    const p2 = { x: 3, y: 4, z: 12 };

    const dist2D = vec2Distance(p1, p2);
    assert.equal(dist2D, 5); // 3-4-5 triangle

    const dist3D = vec3Distance(p1, p2);
    assert.equal(dist3D, 13); // sqrt(9 + 16 + 144) = sqrt(169) = 13
  });

  it('should transform coordinates from World to Ego frame under ISO 8855 (+X Forward, +Y Left)', () => {
    const egoPos = { x: 10, y: 20 };

    // Case 1: Ego facing East (+X axis, heading = 0 rad)
    // Obstacle at (25, 20): exactly 15m ahead
    const rel1 = worldToVehicle2D({ x: 25, y: 20 }, egoPos, 0);
    assert.equal(Math.round(rel1.dx), 15);
    assert.equal(Math.round(rel1.dy), 0);
    assert.equal(Math.round(rel1.range), 15);
    assert.equal(Math.round(rel1.bearingDeg), 0);

    // Obstacle at (10, 28): exactly 8m to the left (+Y)
    const rel2 = worldToVehicle2D({ x: 10, y: 28 }, egoPos, 0);
    assert.equal(Math.round(rel2.dx), 0);
    assert.equal(Math.round(rel2.dy), 8);
    assert.equal(Math.round(rel2.bearingDeg), 90);

    // Case 2: Ego facing North (+Y axis, heading = +90 deg / pi/2 rad)
    // Obstacle at (10, 35): exactly 15m ahead along ego's heading
    const rel3 = worldToVehicle2D({ x: 10, y: 35 }, egoPos, Math.PI / 2);
    assert.equal(Math.round(rel3.dx), 15);
    assert.equal(Math.round(rel3.dy), 0);
    assert.equal(Math.round(rel3.bearingDeg), 0);
  });

  it('should compute closing relative speed (negative when closing in)', () => {
    const ego = {
      position: { x: 0, y: 0, z: 0 },
      orientation: { headingRad: 0 },
      motion: { speedMps: 15.0 } // Ego traveling forward at 15 m/s
    };

    // Obstacle directly ahead at x = 30m, moving forward slower at 5 m/s
    // Closing rate should be (5 - 15) = -10 m/s
    const obstacleAhead = {
      position: { x: 30, y: 0, z: 0 },
      velocity: { vx: 5.0, vy: 0 }
    };

    const metrics = calculateObstacleMetrics(ego, obstacleAhead);

    assert.equal(metrics.relativePosition.dx, 30.0);
    assert.equal(metrics.relativeSpeedMps, -10.0);
    assert.equal(metrics.timeToCollisionSec, 3.0); // 30m / 10 m/s = 3.0s
  });

  it('should set TTC to null when objects are opening distance (diverging)', () => {
    const ego = {
      position: { x: 0, y: 0, z: 0 },
      orientation: { headingRad: 0 },
      motion: { speedMps: 10.0 }
    };

    // Obstacle ahead moving faster away (vx = 20 m/s)
    const obstacleAhead = {
      position: { x: 40, y: 0, z: 0 },
      velocity: { vx: 20.0, vy: 0 }
    };

    const metrics = calculateObstacleMetrics(ego, obstacleAhead);

    assert.ok(metrics.relativeSpeedMps > 0, 'Relative speed must be positive when opening distance');
    assert.equal(metrics.timeToCollisionSec, null, 'TTC must be null when objects are diverging');
  });

  it('should categorize risk as CRITICAL when TTC <= 2.5s in travel lane', () => {
    // dx = 15m, in lane (dy = 0.5m), closing at -10 m/s -> TTC = 1.5s
    const risk = classifyRisk(15.0, 0.5, 15.0, 1.9, -10.0, 1.5);
    assert.equal(risk, 'CRITICAL');
  });

  it('should categorize risk as CRITICAL when range <= 5.0m regardless of speed', () => {
    const risk = classifyRisk(4.0, 0.0, 4.0, 0.0, 0.0, null);
    assert.equal(risk, 'CRITICAL');
  });

  it('should categorize risk as WARNING when TTC <= 4.5s in travel lane', () => {
    // dx = 25m, dy = 1.0m, TTC = 3.5s
    const risk = classifyRisk(25.0, 1.0, 25.02, 2.3, -7.1, 3.5);
    assert.equal(risk, 'WARNING');
  });

  it('should categorize risk as ATTENTION when approaching within 35m with moderate TTC', () => {
    // Obstacle in corridor at 30m, TTC = 6.0s
    const risk = classifyRisk(28.0, 3.0, 28.16, 6.1, -4.7, 6.0);
    assert.equal(risk, 'ATTENTION');
  });

  it('should categorize risk as SAFE when obstacle is outside corridor or moving away', () => {
    // Obstacle far lateral (dy = 12m), divergent
    const risk = classifyRisk(40.0, 12.0, 41.76, 16.7, 2.0, null);
    assert.equal(risk, 'SAFE');
  });

  it('should generate prioritized alerts sorted by severity (CRITICAL > WARNING > INFO)', () => {
    const obstacles = [
      {
        id: 'obs-warn',
        type: 'VEHICLE',
        riskLevel: 'WARNING',
        relativePosition: { range: 18.0 },
        timeToCollisionSec: 3.8
      },
      {
        id: 'obs-crit',
        type: 'PEDESTRIAN',
        riskLevel: 'CRITICAL',
        relativePosition: { range: 4.5 },
        timeToCollisionSec: 1.2
      }
    ];

    const diagnostics = {
      systemHealth: 'DEGRADED',
      motorTempC: 84.0,
      batteryLevelPct: 15.0
    };

    const alerts = generateAlerts(obstacles, diagnostics, { speedMps: 10 }, new Date().toISOString());

    assert.ok(alerts.length >= 2, 'Should generate multiple alerts');
    assert.equal(alerts[0].level, 'CRITICAL', 'First alert must be highest severity (CRITICAL)');
    assert.equal(alerts[0].source, 'FORWARD_COLLISION_WARNING');
    assert.ok(alerts.some(a => a.source === 'BATTERY_THERMAL'));
  });
});
