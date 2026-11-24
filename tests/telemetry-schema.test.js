/**
 * GARUDA VISION - Telemetry Schema Validation Test Suite
 * 
 * Validates that telemetry packets emitted by data providers conform strictly
 * to docs/TELEMETRY_SCHEMA.md v1.0.0.
 * 
 * Checks all required objects, types, ranges, and mandatory synthetic disclaimer.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { SyntheticDataProvider } from '../src/simulation/providers/synthetic-provider.js';
import { CarlaDataProvider } from '../src/simulation/providers/carla-provider.js';

/**
 * Validates a complete telemetry payload against TELEMETRY_SCHEMA v1.0.0.
 * @param {Object} payload 
 * @param {string} expectedSource - 'SYNTHETIC' or 'CARLA_BRIDGE'
 */
function assertTelemetrySchema(payload, expectedSource = 'SYNTHETIC') {
  assert.ok(payload, 'Payload must not be null or undefined');
  assert.equal(typeof payload, 'object', 'Payload must be an object');

  // ==========================================
  // 1. Header Validation
  // ==========================================
  assert.ok(payload.header, 'payload.header is required');
  const h = payload.header;
  assert.equal(typeof h.sequenceNumber, 'number', 'header.sequenceNumber must be a number');
  assert.ok(Number.isInteger(h.sequenceNumber) && h.sequenceNumber >= 0, 'header.sequenceNumber must be a non-negative integer');
  assert.equal(typeof h.timestamp, 'string', 'header.timestamp must be an ISO string');
  assert.ok(!isNaN(Date.parse(h.timestamp)), 'header.timestamp must be valid ISO-8601');
  assert.equal(typeof h.timestampMs, 'number', 'header.timestampMs must be a number');
  assert.ok(h.timestampMs > 0, 'header.timestampMs must be positive');
  assert.equal(h.dataSource, expectedSource, `header.dataSource must match expected ${expectedSource}`);
  assert.equal(h.isSimulated, true, 'header.isSimulated must strictly equal true');
  assert.equal(typeof h.simulationTick, 'number', 'header.simulationTick must be a number');
  assert.ok(h.simulationTick >= 0, 'header.simulationTick must be non-negative');
  assert.equal(typeof h.updateRateHz, 'number', 'header.updateRateHz must be a number');
  assert.ok(h.updateRateHz > 0, 'header.updateRateHz must be greater than 0');
  assert.equal(
    h.disclaimer,
    'SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS',
    'header.disclaimer must contain mandatory synthetic warning'
  );

  // ==========================================
  // 2. Vehicle Validation
  // ==========================================
  assert.ok(payload.vehicle, 'payload.vehicle is required');
  const v = payload.vehicle;
  assert.equal(typeof v.id, 'string', 'vehicle.id must be a string');
  assert.ok(v.id.length > 0, 'vehicle.id must not be empty');
  assert.equal(typeof v.status, 'string', 'vehicle.status must be a string');
  assert.ok(
    ['MANUAL', 'AUTONOMOUS_ENGAGED', 'EMERGENCY_STOP', 'CALIBRATING'].includes(v.status),
    `vehicle.status (${v.status}) must be a valid autonomous state`
  );

  // Position
  assert.ok(v.position, 'vehicle.position is required');
  assert.equal(typeof v.position.x, 'number', 'position.x must be a number');
  assert.equal(typeof v.position.y, 'number', 'position.y must be a number');
  assert.equal(typeof v.position.z, 'number', 'position.z must be a number');

  // Orientation
  assert.ok(v.orientation, 'vehicle.orientation is required');
  assert.equal(typeof v.orientation.yaw, 'number', 'orientation.yaw must be a number');
  assert.equal(typeof v.orientation.pitch, 'number', 'orientation.pitch must be a number');
  assert.equal(typeof v.orientation.roll, 'number', 'orientation.roll must be a number');
  assert.equal(typeof v.orientation.headingDeg, 'number', 'orientation.headingDeg must be a number');
  assert.equal(typeof v.orientation.headingRad, 'number', 'orientation.headingRad must be a number');

  // Motion
  assert.ok(v.motion, 'vehicle.motion is required');
  const m = v.motion;
  assert.equal(typeof m.speedKph, 'number', 'motion.speedKph must be a number');
  assert.ok(m.speedKph >= 0, 'motion.speedKph must be non-negative');
  assert.equal(typeof m.speedMps, 'number', 'motion.speedMps must be a number');
  assert.ok(m.speedMps >= 0, 'motion.speedMps must be non-negative');
  assert.equal(typeof m.acceleration, 'number', 'motion.acceleration must be a number');
  assert.equal(typeof m.jerk, 'number', 'motion.jerk must be a number');
  assert.equal(typeof m.steeringAngleDeg, 'number', 'motion.steeringAngleDeg must be a number');
  assert.ok(m.steeringAngleDeg >= -45 && m.steeringAngleDeg <= 45, 'steering angle must be within realistic bounds');
  assert.equal(typeof m.angularVelocityDeg, 'number', 'motion.angularVelocityDeg must be a number');
  assert.equal(typeof m.throttlePct, 'number', 'motion.throttlePct must be a number');
  assert.ok(m.throttlePct >= 0 && m.throttlePct <= 100, 'motion.throttlePct must be in [0, 100]');
  assert.equal(typeof m.brakePct, 'number', 'motion.brakePct must be a number');
  assert.ok(m.brakePct >= 0 && m.brakePct <= 100, 'motion.brakePct must be in [0, 100]');
  assert.ok(['P', 'R', 'N', 'D'].includes(m.gear), 'motion.gear must be valid gear letter');

  // Odometry
  assert.ok(v.odometry, 'vehicle.odometry is required');
  assert.equal(typeof v.odometry.totalDistanceMeters, 'number', 'odometry.totalDistanceMeters must be a number');
  assert.ok(v.odometry.totalDistanceMeters >= 0, 'odometry.totalDistanceMeters must be >= 0');
  assert.equal(typeof v.odometry.tripDurationSeconds, 'number', 'odometry.tripDurationSeconds must be a number');
  assert.ok(v.odometry.tripDurationSeconds >= 0, 'odometry.tripDurationSeconds must be >= 0');

  // ==========================================
  // 3. Diagnostics Validation
  // ==========================================
  assert.ok(payload.diagnostics, 'payload.diagnostics is required');
  const d = payload.diagnostics;
  assert.ok(['OPTIMAL', 'NOMINAL', 'DEGRADED', 'CRITICAL'].includes(d.systemHealth), 'diagnostics.systemHealth must be valid');
  assert.equal(typeof d.cpuUsagePct, 'number', 'diagnostics.cpuUsagePct must be a number');
  assert.ok(d.cpuUsagePct >= 0 && d.cpuUsagePct <= 100, 'cpuUsagePct must be in [0, 100]');
  assert.equal(typeof d.batteryVoltage, 'number', 'diagnostics.batteryVoltage must be a number');
  assert.ok(d.batteryVoltage > 0, 'diagnostics.batteryVoltage must be > 0');
  assert.equal(typeof d.batteryLevelPct, 'number', 'diagnostics.batteryLevelPct must be a number');
  assert.ok(d.batteryLevelPct >= 0 && d.batteryLevelPct <= 100, 'batteryLevelPct must be in [0, 100]');
  assert.equal(typeof d.motorTempC, 'number', 'diagnostics.motorTempC must be a number');
  assert.equal(typeof d.computeLatencyMs, 'number', 'diagnostics.computeLatencyMs must be a number');
  assert.ok(d.computeLatencyMs >= 0, 'computeLatencyMs must be non-negative');

  // Subsystems
  assert.ok(d.subsystems, 'diagnostics.subsystems is required');
  assert.equal(typeof d.subsystems.perception, 'string', 'perception subsystem required');
  assert.equal(typeof d.subsystems.localization, 'string', 'localization subsystem required');
  assert.equal(typeof d.subsystems.planning, 'string', 'planning subsystem required');
  assert.equal(typeof d.subsystems.control, 'string', 'control subsystem required');
  assert.equal(typeof d.subsystems.lidarSensor, 'string', 'lidarSensor subsystem required');
  assert.equal(typeof d.subsystems.radarSensor, 'string', 'radarSensor subsystem required');
  assert.equal(typeof d.subsystems.cameraArray, 'string', 'cameraArray subsystem required');

  // ==========================================
  // 4. Obstacles Validation
  // ==========================================
  assert.ok(Array.isArray(payload.obstacles), 'payload.obstacles must be an array');
  for (const obs of payload.obstacles) {
    assert.equal(typeof obs.id, 'string', 'obstacle.id must be a string');
    assert.ok(
      ['VEHICLE', 'PEDESTRIAN', 'CYCLIST', 'STATIC_BARRIER', 'DEBRIS'].includes(obs.type),
      `obstacle.type (${obs.type}) must be valid`
    );
    assert.ok(obs.position && typeof obs.position.x === 'number' && typeof obs.position.y === 'number', 'obstacle.position required');
    assert.ok(obs.dimensions && typeof obs.dimensions.length === 'number', 'obstacle.dimensions required');
    assert.ok(obs.relativePosition, 'obstacle.relativePosition required');
    assert.equal(typeof obs.relativePosition.dx, 'number', 'relativePosition.dx must be a number');
    assert.equal(typeof obs.relativePosition.dy, 'number', 'relativePosition.dy must be a number');
    assert.equal(typeof obs.relativePosition.range, 'number', 'relativePosition.range must be a number');
    assert.equal(typeof obs.relativePosition.bearingDeg, 'number', 'relativePosition.bearingDeg must be a number');
    assert.ok(obs.velocity && typeof obs.velocity.speedKph === 'number', 'obstacle.velocity required');
    assert.equal(typeof obs.relativeSpeedMps, 'number', 'obstacle.relativeSpeedMps must be a number');
    assert.ok(
      obs.timeToCollisionSec === null || (typeof obs.timeToCollisionSec === 'number' && obs.timeToCollisionSec >= 0),
      'obstacle.timeToCollisionSec must be null or positive number'
    );
    assert.ok(['SAFE', 'ATTENTION', 'WARNING', 'CRITICAL'].includes(obs.riskLevel), 'obstacle.riskLevel must be valid');
    assert.equal(typeof obs.classificationConfidence, 'number', 'obstacle.classificationConfidence must be a number');
    assert.ok(obs.classificationConfidence >= 0 && obs.classificationConfidence <= 1.0, 'confidence must be in [0, 1]');
  }

  // ==========================================
  // 5. LiDAR Validation
  // ==========================================
  assert.ok(payload.lidar, 'payload.lidar is required');
  const l = payload.lidar;
  assert.equal(typeof l.sensor, 'string', 'lidar.sensor must be a string');
  assert.equal(typeof l.pointCount, 'number', 'lidar.pointCount must be a number');
  assert.ok(l.pointCount >= 0, 'lidar.pointCount must be non-negative');
  assert.equal(typeof l.rangeMeters, 'number', 'lidar.rangeMeters must be a number');
  assert.ok(l.rangeMeters > 0, 'lidar.rangeMeters must be positive');
  assert.equal(typeof l.fieldOfViewDeg, 'number', 'lidar.fieldOfViewDeg must be a number');
  assert.ok(Array.isArray(l.points), 'lidar.points must be an array');
  assert.equal(l.points.length, l.pointCount, 'lidar.points.length must match lidar.pointCount');
  if (l.points.length > 0) {
    const p0 = l.points[0];
    assert.ok(Array.isArray(p0) && p0.length >= 3, 'lidar point must be [x, y, z, (intensity)]');
    assert.equal(typeof p0[0], 'number', 'point[0] (x) must be a number');
    assert.equal(typeof p0[1], 'number', 'point[1] (y) must be a number');
    assert.equal(typeof p0[2], 'number', 'point[2] (z) must be a number');
  }

  // ==========================================
  // 6. Alerts Validation
  // ==========================================
  assert.ok(Array.isArray(payload.alerts), 'payload.alerts must be an array');
  for (const alt of payload.alerts) {
    assert.equal(typeof alt.id, 'string', 'alert.id must be a string');
    assert.ok(['INFO', 'WARNING', 'CRITICAL'].includes(alt.level), 'alert.level must be INFO, WARNING, or CRITICAL');
    assert.equal(typeof alt.source, 'string', 'alert.source must be a string');
    assert.equal(typeof alt.message, 'string', 'alert.message must be a string');
    assert.equal(typeof alt.timestamp, 'string', 'alert.timestamp must be an ISO string');
  }

  // ==========================================
  // 7. Navigation Validation
  // ==========================================
  assert.ok(payload.navigation, 'payload.navigation is required');
  const n = payload.navigation;
  assert.equal(typeof n.currentWaypointIndex, 'number', 'navigation.currentWaypointIndex must be a number');
  assert.equal(typeof n.totalWaypoints, 'number', 'navigation.totalWaypoints must be a number');
  assert.equal(typeof n.currentRoadSegment, 'string', 'navigation.currentRoadSegment must be a string');
  assert.ok(['LEFT', 'CENTER', 'RIGHT'].includes(n.lanePosition), 'navigation.lanePosition must be valid');
  assert.equal(typeof n.distanceToNextTurnMeters, 'number', 'navigation.distanceToNextTurnMeters must be a number');
  assert.equal(typeof n.targetSpeedKph, 'number', 'navigation.targetSpeedKph must be a number');
}

describe('Telemetry Schema Specification Tests (TELEMETRY_SCHEMA v1.0.0)', () => {
  it('should validate complete SyntheticDataProvider telemetry payload', () => {
    const provider = new SyntheticDataProvider();
    provider.step(0.05); // step 1 tick (50ms)
    const payload = provider.getNextTelemetry();

    assertTelemetrySchema(payload, 'SYNTHETIC');
  });

  it('should validate complete CarlaDataProvider telemetry payload (Fallback mode)', () => {
    const provider = new CarlaDataProvider();
    provider.step(0.05);
    const payload = provider.getNextTelemetry();

    assertTelemetrySchema(payload, 'CARLA_BRIDGE');
  });

  it('should strictly contain synthetic disclaimer on all payloads', () => {
    const synthProvider = new SyntheticDataProvider();
    const carlaProvider = new CarlaDataProvider();

    const p1 = synthProvider.getNextTelemetry();
    const p2 = carlaProvider.getNextTelemetry();

    assert.equal(p1.header.isSimulated, true);
    assert.equal(p2.header.isSimulated, true);
    assert.ok(p1.header.disclaimer.includes('SYNTHETIC / SIMULATED DATA'));
    assert.ok(p2.header.disclaimer.includes('SYNTHETIC / SIMULATED DATA'));
  });

  it('should ensure speedKph and speedMps are mathematically consistent (v_kph = v_mps * 3.6)', () => {
    const provider = new SyntheticDataProvider();
    provider.step(1.0);
    const payload = provider.getNextTelemetry();

    const speedMps = payload.vehicle.motion.speedMps;
    const speedKph = payload.vehicle.motion.speedKph;

    const expectedKph = speedMps * 3.6;
    assert.ok(
      Math.abs(speedKph - expectedKph) < 0.2,
      `speedKph (${speedKph}) must match speedMps * 3.6 (${expectedKph}) within rounding tolerance`
    );
  });
});
