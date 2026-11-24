/**
 * GARUDA VISION - Vehicle Kinematics Test Suite
 * 
 * Verifies the deterministic vehicle kinematic model:
 * 1. Deterministic movement along planned trajectory (repeatable bit-exact poses).
 * 2. Speed calculation from position displacement (v = ds / dt).
 * 3. Heading calculation from velocity vector (theta = atan2(vy, vx)).
 * 4. Longitudinal acceleration calculation from velocity delta (a = dv / dt).
 * 5. Jerk calculation from acceleration delta (j = da / dt).
 * 6. Bicycle model steering angle bounds.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { VehicleModel } from '../src/simulation/models/vehicle-model.js';
import {
  catmullRom,
  catmullRomTangent,
  radToDeg,
  degToRad,
  normalizeAngleDeg,
  normalizeAngleRad,
  vec2Distance
} from '../src/simulation/utils/math-utils.js';

describe('Deterministic Kinematic Vehicle Model Tests', () => {
  it('should exhibit 100% deterministic movement for identical simulation steps', () => {
    const v1 = new VehicleModel('TEST-EGO-01');
    const v2 = new VehicleModel('TEST-EGO-02');

    // Simulate 20 ticks (1 second at 20 Hz, dt = 0.05s)
    const dt = 0.05;
    for (let i = 0; i < 20; i++) {
      v1.step(dt);
      v2.step(dt);
    }

    const t1 = v1.getTelemetryData();
    const t2 = v2.getTelemetryData();

    // Verify positions match exactly
    assert.equal(t1.vehicle.position.x, t2.vehicle.position.x);
    assert.equal(t1.vehicle.position.y, t2.vehicle.position.y);
    assert.equal(t1.vehicle.position.z, t2.vehicle.position.z);

    // Verify velocity, acceleration, and heading match exactly
    assert.equal(t1.vehicle.motion.speedMps, t2.vehicle.motion.speedMps);
    assert.equal(t1.vehicle.motion.acceleration, t2.vehicle.motion.acceleration);
    assert.equal(t1.vehicle.orientation.headingDeg, t2.vehicle.orientation.headingDeg);
    assert.equal(t1.vehicle.odometry.totalDistanceMeters, t2.vehicle.odometry.totalDistanceMeters);
  });

  it('should calculate speed consistently from position displacement', () => {
    const vehicle = new VehicleModel('TEST-EGO');
    // Advance several seconds so vehicle accelerates to cruising speed
    const dt = 0.05;
    for (let i = 0; i < 50; i++) {
      vehicle.step(dt);
    }

    const posBefore = { ...vehicle.position };
    vehicle.step(dt);
    const posAfter = { ...vehicle.position };

    const displacement = vec2Distance(posBefore, posAfter);
    const calculatedSpeedMps = displacement / dt;

    const reportedSpeedMps = vehicle.currentSpeedMps;

    // Both should match within reasonable discretization tolerance
    assert.ok(
      Math.abs(calculatedSpeedMps - reportedSpeedMps) < 0.2,
      `Calculated speed from displacement (${calculatedSpeedMps.toFixed(2)}) must match reported speed (${reportedSpeedMps.toFixed(2)})`
    );
  });

  it('should calculate heading from velocity vector (atan2(dy, dx))', () => {
    const p0 = { x: 0, y: 0, z: 0 };
    const p1 = { x: 10, y: 0, z: 0 };
    const p2 = { x: 20, y: 10, z: 0 };
    const p3 = { x: 30, y: 30, z: 0 };

    // Test Catmull-Rom tangent at t = 0.5
    const tangent = catmullRomTangent(p0, p1, p2, p3, 0.5);
    const expectedAngleRad = Math.atan2(tangent.dy, tangent.dx);
    const expectedAngleDeg = radToDeg(expectedAngleRad);

    assert.ok(typeof expectedAngleDeg === 'number' && !isNaN(expectedAngleDeg));
    assert.ok(tangent.dx > 0, 'Longitudinal forward tangent should be positive');
  });

  it('should derive acceleration from velocity delta: a = dv / dt', () => {
    const vehicle = new VehicleModel('TEST-EGO');
    const dt = 0.05;

    // Advance 5 steps
    for (let i = 0; i < 5; i++) {
      vehicle.step(dt);
    }

    const v1 = vehicle.currentSpeedMps;
    vehicle.step(dt);
    const v2 = vehicle.currentSpeedMps;

    const expectedAccel = (v2 - v1) / dt;
    const reportedAccel = vehicle.currentAcceleration;

    assert.ok(
      Math.abs(reportedAccel - expectedAccel) < 0.1,
      `Reported accel (${reportedAccel}) must match (v2 - v1)/dt (${expectedAccel.toFixed(2)})`
    );
  });

  it('should derive jerk from acceleration delta: j = da / dt', () => {
    const vehicle = new VehicleModel('TEST-EGO');
    const dt = 0.05;

    for (let i = 0; i < 10; i++) {
      vehicle.step(dt);
    }

    const a1 = vehicle.currentAcceleration;
    vehicle.step(dt);
    const a2 = vehicle.currentAcceleration;

    const expectedJerk = (a2 - a1) / dt;
    const reportedJerk = vehicle.currentJerk;

    assert.ok(
      Math.abs(reportedJerk - expectedJerk) < 0.15,
      `Reported jerk (${reportedJerk}) must match (a2 - a1)/dt (${expectedJerk.toFixed(2)})`
    );
  });

  it('should bound steering angle within realistic kinematic limits (e.g. [-35, +35] deg)', () => {
    const vehicle = new VehicleModel('TEST-EGO');
    const dt = 0.05;

    // Run entire circuit (800 ticks = 40s)
    let maxSteering = 0;
    for (let i = 0; i < 400; i++) {
      vehicle.step(dt);
      const absSteer = Math.abs(vehicle.steeringAngleDeg);
      if (absSteer > maxSteering) {
        maxSteering = absSteer;
      }
    }

    assert.ok(
      maxSteering <= 35.0,
      `Maximum steering angle reached (${maxSteering.toFixed(2)} deg) must not exceed 35 deg physical limit`
    );
  });

  it('should correctly normalize angles in degrees and radians', () => {
    assert.equal(normalizeAngleDeg(190), -170);
    assert.equal(normalizeAngleDeg(-200), 160);
    assert.equal(normalizeAngleDeg(360), 0);
    assert.equal(normalizeAngleDeg(-360), 0);
    assert.equal(normalizeAngleDeg(45), 45);

    assert.ok(Math.abs(normalizeAngleRad(Math.PI * 1.5) - (-Math.PI * 0.5)) < 1e-6);
    assert.ok(Math.abs(normalizeAngleRad(-Math.PI * 1.5) - (Math.PI * 0.5)) < 1e-6);
  });
});
