/**
 * GARUDA VISION - Kinematic Vehicle Model
 * 
 * Simulates a continuous urban loop trajectory with realistic kinematic constraints.
 * Positions, velocities, accelerations, jerks, headings, and steering angles are
 * mathematically derived using a kinematic bicycle model along a smooth path.
 */

import {
  catmullRom,
  catmullRomTangent,
  radToDeg,
  degToRad,
  normalizeAngleDeg,
  normalizeAngleRad,
  clamp,
  roundTo,
  gaussianNoise,
  lerp
} from '../utils/math-utils.js';

// Predefined 40-waypoint urban circuit with sectors and target speeds
const WAYPOINTS = [
  // Sector 1: Starting Straightaway (Waypoints 0 - 6)
  { x: 0.0, y: 0.0, z: 0.0, speedKph: 20.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 20.0, y: 0.0, z: 0.0, speedKph: 28.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 45.0, y: 0.0, z: 0.0, speedKph: 40.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 75.0, y: 0.0, z: 0.0, speedKph: 48.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 105.0, y: 0.0, z: 0.0, speedKph: 50.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 135.0, y: 2.0, z: 0.0, speedKph: 45.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },
  { x: 160.0, y: 8.0, z: 0.0, speedKph: 35.0, sector: 'Sector-1 Departure Boulevard', lane: 'CENTER' },

  // Sector 2: High Street Turn (Waypoints 7 - 12)
  { x: 185.0, y: 20.0, z: 0.0, speedKph: 28.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },
  { x: 205.0, y: 40.0, z: 0.0, speedKph: 25.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },
  { x: 215.0, y: 65.0, z: 0.0, speedKph: 25.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },
  { x: 218.0, y: 95.0, z: 0.0, speedKph: 30.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },
  { x: 210.0, y: 125.0, z: 0.0, speedKph: 38.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },
  { x: 195.0, y: 155.0, z: 0.0, speedKph: 45.0, sector: 'Sector-2 High Street Turn', lane: 'CENTER' },

  // Sector 3: Avenue Straight (Waypoints 13 - 18)
  { x: 175.0, y: 185.0, z: 0.0, speedKph: 48.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },
  { x: 150.0, y: 210.0, z: 0.0, speedKph: 50.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },
  { x: 120.0, y: 235.0, z: 0.0, speedKph: 50.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },
  { x: 90.0, y: 255.0, z: 0.0, speedKph: 45.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },
  { x: 60.0, y: 270.0, z: 0.0, speedKph: 35.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },
  { x: 30.0, y: 280.0, z: 0.0, speedKph: 20.0, sector: 'Sector-3 Tech Avenue', lane: 'CENTER' },

  // Sector 4: Crosswalk & Intersection Zone (Waypoints 19 - 24)
  { x: 5.0, y: 285.0, z: 0.0, speedKph: 10.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' },
  { x: -15.0, y: 285.0, z: 0.0, speedKph: 0.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' }, // Pedestrian stop zone
  { x: -35.0, y: 285.0, z: 0.0, speedKph: 15.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' },
  { x: -60.0, y: 282.0, z: 0.0, speedKph: 30.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' },
  { x: -90.0, y: 275.0, z: 0.0, speedKph: 42.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' },
  { x: -125.0, y: 260.0, z: 0.0, speedKph: 52.0, sector: 'Sector-4 Crosswalk & Intersection Zone', lane: 'CENTER' },

  // Sector 5: Sector-5 Northbound Expressway (Waypoints 25 - 30)
  { x: -160.0, y: 240.0, z: 0.0, speedKph: 58.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },
  { x: -200.0, y: 210.0, z: 0.0, speedKph: 60.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },
  { x: -235.0, y: 175.0, z: 0.0, speedKph: 60.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },
  { x: -260.0, y: 135.0, z: 0.0, speedKph: 55.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },
  { x: -275.0, y: 95.0, z: 0.0, speedKph: 48.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },
  { x: -275.0, y: 55.0, z: 0.0, speedKph: 40.0, sector: 'Sector-5 Northbound Expressway', lane: 'CENTER' },

  // Sector 6: Chicane Zone (Waypoints 31 - 35)
  { x: -260.0, y: 20.0, z: 0.0, speedKph: 32.0, sector: 'Sector-6 Chicane S-Curve', lane: 'LEFT' },
  { x: -235.0, y: -5.0, z: 0.0, speedKph: 30.0, sector: 'Sector-6 Chicane S-Curve', lane: 'RIGHT' },
  { x: -200.0, y: -20.0, z: 0.0, speedKph: 32.0, sector: 'Sector-6 Chicane S-Curve', lane: 'LEFT' },
  { x: -160.0, y: -25.0, z: 0.0, speedKph: 38.0, sector: 'Sector-6 Chicane S-Curve', lane: 'CENTER' },
  { x: -120.0, y: -25.0, z: 0.0, speedKph: 42.0, sector: 'Sector-6 Chicane S-Curve', lane: 'CENTER' },

  // Sector 7: Return Loop to Terminal (Waypoints 36 - 39)
  { x: -80.0, y: -20.0, z: 0.0, speedKph: 45.0, sector: 'Sector-7 Return Terminal Approach', lane: 'CENTER' },
  { x: -45.0, y: -12.0, z: 0.0, speedKph: 40.0, sector: 'Sector-7 Return Terminal Approach', lane: 'CENTER' },
  { x: -20.0, y: -5.0, z: 0.0, speedKph: 30.0, sector: 'Sector-7 Return Terminal Approach', lane: 'CENTER' },
  { x: -5.0, y: -1.0, z: 0.0, speedKph: 15.0, sector: 'Sector-7 Return Terminal Approach', lane: 'CENTER' }
];

export class VehicleModel {
  constructor(vehicleId = 'GARUDA-EGO-01') {
    this.id = vehicleId;
    this.status = 'AUTONOMOUS_ENGAGED';
    this.wheelbaseMeters = 2.85; // Wheelbase for bicycle model
    this.noiseEnabled = false;

    // Track path geometry
    this.waypoints = WAYPOINTS;
    this.totalWaypoints = WAYPOINTS.length;
    this._computePathMetrics();

    this.reset();
  }

  /**
   * Resets vehicle state to the beginning of the trajectory.
   */
  reset() {
    this.currentDistance = 0.0;
    this.currentSpeedMps = 0.0;
    this.nominalSpeedMps = 0.0;
    this.commandedAccel = 0.0;
    this.previousSpeedMps = 0.0;
    this.currentAcceleration = 0.0;
    this.previousAcceleration = 0.0;
    this.currentJerk = 0.0;
    this.steeringAngleDeg = 0.0;
    this.yawRateDeg = 0.0;
    this.headingRad = 0.0;
    this.headingDeg = 0.0;
    this.position = { x: this.waypoints[0].x, y: this.waypoints[0].y, z: 0.0 };

    this.totalDistanceMeters = 0.0;
    this.tripDurationSeconds = 0.0;
    this.throttlePct = 0.0;
    this.brakePct = 0.0;
    this.gear = 'D';

    // Pedestrian crosswalk wait timer
    this.crosswalkWaitTime = 0.0;
    this.isWaitingAtCrosswalk = false;

    // Diagnostic state
    this.batteryLevelPct = 94.5;
    this.batteryVoltage = 399.2;
    this.motorTempC = 42.5;
    this.cpuUsagePct = 18.5;
    this.computeLatencyMs = 14.0;
    this.systemHealth = 'OPTIMAL';

    // Navigation state
    this.currentWaypointIndex = 0;
    this.currentRoadSegment = this.waypoints[0].sector;
    this.lanePosition = this.waypoints[0].lane;
    this.distanceToNextTurnMeters = 160.0;
    this.targetSpeedKph = 25.0;

    // Initialize initial pose
    this._updatePoseFromDistance(0.0);
  }

  /**
   * Toggles realistic sensor noise.
   * @param {boolean} enabled 
   */
  setNoiseEnabled(enabled) {
    this.noiseEnabled = !!enabled;
  }

  /**
   * Advances the vehicle kinematics by time delta dt (seconds).
   * @param {number} dt 
   */
  step(dt) {
    if (dt <= 0) return;
    this.tripDurationSeconds += dt;

    // 1. Determine active segment and target speed
    const wpInfo = this._findSegmentAtDistance(this.currentDistance);
    this.currentWaypointIndex = wpInfo.index;
    const targetWp = this.waypoints[wpInfo.index];
    const nextWp = this.waypoints[(wpInfo.index + 1) % this.waypoints.length];
    const blendedTargetKph = lerp(targetWp.speedKph, nextWp.speedKph, wpInfo.t);

    this.currentRoadSegment = targetWp.sector;
    this.lanePosition = targetWp.lane;
    this.targetSpeedKph = blendedTargetKph;

    // Check crosswalk stop condition in Sector 4 (waypoint 20)
    let targetSpeedMps = blendedTargetKph / 3.6;
    if (wpInfo.index === 20 && !this.isWaitingAtCrosswalk && this.crosswalkWaitTime < 3.0) {
      targetSpeedMps = 0.0;
      this.targetSpeedKph = 0.0;
      if (this.currentSpeedMps < 0.5) {
        this.crosswalkWaitTime += dt;
        if (this.crosswalkWaitTime >= 3.0) {
          this.isWaitingAtCrosswalk = true; // Wait completed, proceed
        }
      }
    } else if (wpInfo.index !== 20) {
      // Reset crosswalk wait when past sector
      this.crosswalkWaitTime = 0.0;
      this.isWaitingAtCrosswalk = false;
    }

    // 2. Longitudinal acceleration calculation (smooth proportional cruise control)
    const prevX = this.position.x;
    const prevY = this.position.y;
    const prevSpeed = this.currentSpeedMps;
    const prevAccel = this.currentAcceleration;

    const speedDiff = targetSpeedMps - this.nominalSpeedMps;
    let desiredAccel = 0.0;
    if (speedDiff > 0) {
      // Accelerating (smooth ramp up, max 2.2 m/s^2)
      desiredAccel = Math.min(2.2, speedDiff * 0.8);
    } else {
      // Braking (smooth deceleration, max -3.8 m/s^2)
      desiredAccel = Math.max(-3.8, speedDiff * 1.2);
    }

    // Smooth acceleration transition
    const accelSlewRate = 3.5 * dt;
    this.commandedAccel = clamp(
      this.commandedAccel + clamp(desiredAccel - this.commandedAccel, -accelSlewRate, accelSlewRate),
      -4.5,
      2.5
    );

    // 3. Integrate distance along path
    this.nominalSpeedMps = Math.max(0.0, this.nominalSpeedMps + this.commandedAccel * dt);
    const avgSpeed = (prevSpeed + this.nominalSpeedMps) / 2.0;
    const distanceDelta = avgSpeed * dt;
    this.currentDistance = (this.currentDistance + distanceDelta) % this.totalPathLength;
    this.totalDistanceMeters += distanceDelta;

    // 4. Update vehicle pose (position, heading, steering) from path geometry
    this._updatePoseFromDistance(this.currentDistance);

    // 5. Calculate vehicle speed and acceleration directly from actual spatial displacement (movement)
    const displacement = Math.hypot(this.position.x - prevX, this.position.y - prevY);
    if (dt > 0) {
      this.currentSpeedMps = displacement / dt;
      this.currentAcceleration = (this.currentSpeedMps - prevSpeed) / dt;
      this.currentJerk = (this.currentAcceleration - prevAccel) / dt;
    } else {
      this.currentSpeedMps = 0.0;
      this.currentAcceleration = 0.0;
      this.currentJerk = 0.0;
    }
    this.previousSpeedMps = prevSpeed;
    this.previousAcceleration = prevAccel;

    // 6. Compute throttle & brake percentage inputs
    if (this.currentAcceleration > 0.05) {
      this.throttlePct = clamp((this.currentAcceleration / 2.2) * 100, 0, 100);
      this.brakePct = 0.0;
      this.gear = 'D';
    } else if (this.currentAcceleration < -0.05) {
      this.throttlePct = 0.0;
      this.brakePct = clamp((-this.currentAcceleration / 3.8) * 100, 0, 100);
      this.gear = 'D';
    } else {
      this.throttlePct = 12.0; // Steady cruise throttle
      this.brakePct = 0.0;
      this.gear = 'D';
    }
    if (this.currentSpeedMps < 0.1 && targetSpeedMps === 0) {
      this.brakePct = 85.0;
      this.throttlePct = 0.0;
    }

    // 7. Distance to next turn calculation
    this._computeDistanceToNextTurn();

    // 8. Update diagnostics state realistically
    this._updateDiagnostics(dt);
  }

  /**
   * Pre-calculates segment lengths and total cumulative distance along path.
   * @private
   */
  _computePathMetrics() {
    this.segmentLengths = [];
    this.cumulativeDistances = [0.0];
    let total = 0.0;

    for (let i = 0; i < this.waypoints.length; i++) {
      const p1 = this.waypoints[i];
      const p2 = this.waypoints[(i + 1) % this.waypoints.length];
      const dist = Math.hypot(p2.x - p1.x, p2.y - p1.y);
      this.segmentLengths.push(dist);
      total += dist;
      this.cumulativeDistances.push(total);
    }

    this.totalPathLength = total;
  }

  /**
   * Finds the segment index and local interpolation factor t for a given arc length.
   * @private
   */
  _findSegmentAtDistance(distance) {
    const d = distance % this.totalPathLength;
    for (let i = 0; i < this.segmentLengths.length; i++) {
      const segStart = this.cumulativeDistances[i];
      const segEnd = this.cumulativeDistances[i + 1];
      if (d >= segStart && d <= segEnd) {
        const segLen = this.segmentLengths[i];
        const t = segLen > 0 ? (d - segStart) / segLen : 0;
        return { index: i, t: clamp(t, 0, 1) };
      }
    }
    return { index: 0, t: 0 };
  }

  /**
   * Evaluates position, heading, and curvature along the spline at distance s.
   * @private
   */
  _updatePoseFromDistance(distance) {
    const { index, t } = this._findSegmentAtDistance(distance);
    const n = this.waypoints.length;

    // Catmull-Rom four control points
    const p0 = this.waypoints[(index - 1 + n) % n];
    const p1 = this.waypoints[index];
    const p2 = this.waypoints[(index + 1) % n];
    const p3 = this.waypoints[(index + 2) % n];

    // Position on spline
    const pos = catmullRom(p0, p1, p2, p3, t);
    this.position = {
      x: pos.x,
      y: pos.y,
      z: 0.0
    };

    // First derivative (tangent) for heading
    const tangent = catmullRomTangent(p0, p1, p2, p3, t);
    const rawHeadingRad = Math.atan2(tangent.dy, tangent.dx);
    const prevHeadingRad = this.headingRad;
    this.headingRad = normalizeAngleRad(rawHeadingRad);
    this.headingDeg = roundTo(radToDeg(this.headingRad), 1);

    // Curvature estimation: evaluate tangent slightly ahead
    const dtEps = 0.05;
    const tangentAhead = catmullRomTangent(p0, p1, p2, p3, clamp(t + dtEps, 0, 1));
    const headingAhead = Math.atan2(tangentAhead.dy, tangentAhead.dx);
    const headingDiff = normalizeAngleRad(headingAhead - rawHeadingRad);
    const approxCurvature = (dtEps * this.segmentLengths[index]) > 0.01
      ? headingDiff / (dtEps * this.segmentLengths[index])
      : 0.0;

    // Kinematic bicycle model: tan(delta) = L * curvature
    const rawSteeringRad = Math.atan(this.wheelbaseMeters * approxCurvature);
    const rawSteeringDeg = clamp(radToDeg(rawSteeringRad), -35.0, 35.0);
    this.steeringAngleDeg = roundTo(rawSteeringDeg, 1);

    // Yaw rate: omega = d(theta)/dt
    const rawYawRateDeg = this.currentSpeedMps > 0.1
      ? radToDeg(approxCurvature * this.currentSpeedMps)
      : 0.0;
    this.yawRateDeg = roundTo(clamp(rawYawRateDeg, -45.0, 45.0), 1);
  }

  /**
   * Finds distance to upcoming major curved turn in trajectory.
   * @private
   */
  _computeDistanceToNextTurn() {
    const turns = [7, 18, 31, 36];
    let minDistance = 999.0;

    for (const turnIdx of turns) {
      const turnDist = this.cumulativeDistances[turnIdx];
      let diff = turnDist - this.currentDistance;
      if (diff < 0) diff += this.totalPathLength;
      if (diff < minDistance) {
        minDistance = diff;
      }
    }

    this.distanceToNextTurnMeters = roundTo(minDistance, 1);
  }

  /**
   * Updates battery, motor temperature, CPU load, and subsystem health.
   * @private
   */
  _updateDiagnostics(dt) {
    // Battery slowly drains (e.g., 1% every 300 seconds)
    this.batteryLevelPct = Math.max(15.0, this.batteryLevelPct - (dt / 300.0));
    // Voltage corresponds to 350V - 410V
    this.batteryVoltage = 350.0 + (this.batteryLevelPct / 100.0) * 60.0;

    // Motor temperature rises under heavy throttle
    const targetTemp = 40.0 + (this.throttlePct / 100.0) * 22.0 + (this.currentSpeedMps / 16.0) * 10.0;
    this.motorTempC += (targetTemp - this.motorTempC) * 0.05 * dt;

    // Embedded CPU usage oscillates smoothly around 18-24%
    const cpuBase = 18.0 + (this.currentSpeedMps / 16.0) * 4.0;
    this.cpuUsagePct = roundTo(cpuBase + Math.sin(this.tripDurationSeconds * 0.5) * 2.5, 1);

    // Compute latency ~12 - 16 ms
    this.computeLatencyMs = roundTo(13.5 + Math.cos(this.tripDurationSeconds * 0.8) * 1.8, 1);
  }

  /**
   * Returns vehicle telemetry payload conforming to docs/TELEMETRY_SCHEMA.md
   * @returns {{
   *   vehicle: Object,
   *   diagnostics: Object,
   *   navigation: Object
   * }}
   */
  getTelemetryData() {
    let x = this.position.x;
    let y = this.position.y;
    let speedMps = this.currentSpeedMps;
    let accel = this.currentAcceleration;
    let headingDeg = this.headingDeg;
    let headingRad = this.headingRad;

    // Apply optional Gaussian noise
    if (this.noiseEnabled) {
      x += gaussianNoise(0, 0.05);
      y += gaussianNoise(0, 0.05);
      speedMps = Math.max(0, speedMps + gaussianNoise(0, 0.08));
      accel += gaussianNoise(0, 0.05);
      headingDeg = normalizeAngleDeg(headingDeg + gaussianNoise(0, 0.2));
      headingRad = degToRad(headingDeg);
    }

    const speedKph = roundTo(speedMps * 3.6, 1);

    return {
      vehicle: {
        id: this.id,
        status: this.status,
        position: {
          x: roundTo(x, 2),
          y: roundTo(y, 2),
          z: 0.0
        },
        orientation: {
          yaw: roundTo(headingDeg, 1),
          pitch: 0.0,
          roll: 0.0,
          headingDeg: roundTo(headingDeg, 1),
          headingRad: roundTo(headingRad, 4)
        },
        motion: {
          speedKph,
          speedMps: roundTo(speedMps, 2),
          acceleration: roundTo(accel, 2),
          jerk: roundTo(this.currentJerk, 2),
          steeringAngleDeg: roundTo(this.steeringAngleDeg, 1),
          angularVelocityDeg: roundTo(this.yawRateDeg, 1),
          throttlePct: roundTo(this.throttlePct, 1),
          brakePct: roundTo(this.brakePct, 1),
          gear: this.gear
        },
        odometry: {
          totalDistanceMeters: roundTo(this.totalDistanceMeters, 1),
          tripDurationSeconds: roundTo(this.tripDurationSeconds, 1)
        }
      },
      diagnostics: {
        systemHealth: this.systemHealth,
        cpuUsagePct: roundTo(this.cpuUsagePct, 1),
        batteryVoltage: roundTo(this.batteryVoltage, 1),
        batteryLevelPct: roundTo(this.batteryLevelPct, 1),
        motorTempC: roundTo(this.motorTempC, 1),
        computeLatencyMs: roundTo(this.computeLatencyMs, 1),
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
      navigation: {
        currentWaypointIndex: this.currentWaypointIndex,
        totalWaypoints: this.totalWaypoints,
        currentRoadSegment: this.currentRoadSegment,
        lanePosition: this.lanePosition,
        distanceToNextTurnMeters: roundTo(this.distanceToNextTurnMeters, 1),
        targetSpeedKph: roundTo(this.targetSpeedKph, 1)
      }
    };
  }
}
