/**
 * GARUDA VISION - Risk Assessment & Collision Warning Engine
 * 
 * Computes obstacle relative spatial metrics (dx, dy, range, bearingDeg),
 * closing rate (relativeSpeedMps), Time-To-Collision (TTC), risk level classification
 * (SAFE, ATTENTION, WARNING, CRITICAL), and active collision alert generation.
 */

import { worldToVehicle2D, roundTo } from './math-utils.js';

/**
 * Calculates relative kinematics for a given obstacle with respect to the ego vehicle.
 * 
 * @param {Object} ego
 * @param {{x: number, y: number, z: number}} ego.position
 * @param {{headingRad: number}} ego.orientation
 * @param {{speedMps: number}} ego.motion
 * @param {Object} obstacle
 * @param {{x: number, y: number, z: number}} obstacle.position
 * @param {{vx: number, vy: number}} obstacle.velocity
 * @returns {{
 *   relativePosition: {dx: number, dy: number, range: number, bearingDeg: number},
 *   relativeSpeedMps: number,
 *   timeToCollisionSec: number | null,
 *   riskLevel: "SAFE" | "ATTENTION" | "WARNING" | "CRITICAL"
 * }}
 */
export function calculateObstacleMetrics(ego, obstacle) {
  // 1. Transform obstacle world position to ego vehicle frame
  const rel = worldToVehicle2D(obstacle.position, ego.position, ego.orientation.headingRad);
  const range = roundTo(rel.range, 2);
  const dx = roundTo(rel.dx, 2);
  const dy = roundTo(rel.dy, 2);
  const bearingDeg = roundTo(rel.bearingDeg, 1);

  // 2. Compute ego velocity vector in world coordinates
  const egoVx = ego.motion.speedMps * Math.cos(ego.orientation.headingRad);
  const egoVy = ego.motion.speedMps * Math.sin(ego.orientation.headingRad);

  // 3. Relative velocity vector (obstacle - ego)
  const obsVx = obstacle.velocity.vx;
  const obsVy = obstacle.velocity.vy;
  const vRelX = obsVx - egoVx;
  const vRelY = obsVy - egoVy;

  // 4. Rate of change of range (projection of relative velocity onto line-of-sight)
  // Displacement vector from ego to obstacle
  const dispX = obstacle.position.x - ego.position.x;
  const dispY = obstacle.position.y - ego.position.y;
  const dist = Math.hypot(dispX, dispY);

  let relativeSpeedMps = 0;
  if (dist > 1e-4) {
    // Negative means closing in (distance decreasing)
    relativeSpeedMps = (vRelX * dispX + vRelY * dispY) / dist;
  }
  relativeSpeedMps = roundTo(relativeSpeedMps, 2);

  // 5. Time-To-Collision (TTC)
  // Closing in if relativeSpeedMps < -0.1 m/s
  let timeToCollisionSec = null;
  if (relativeSpeedMps < -0.1 && range > 0.1) {
    const rawTtc = range / Math.abs(relativeSpeedMps);
    // Only cap realistic TTC up to 30s
    if (rawTtc <= 30.0) {
      timeToCollisionSec = roundTo(rawTtc, 2);
    }
  }

  // 6. Classify risk level
  const riskLevel = classifyRisk(dx, dy, range, bearingDeg, relativeSpeedMps, timeToCollisionSec);

  return {
    relativePosition: { dx, dy, range, bearingDeg },
    relativeSpeedMps,
    timeToCollisionSec,
    riskLevel
  };
}

/**
 * Classifies the risk level of an obstacle.
 * 
 * @param {number} dx - Forward distance relative to ego
 * @param {number} dy - Lateral distance relative to ego (+left, -right)
 * @param {number} range - Total Euclidean distance
 * @param {number} bearingDeg - Relative bearing angle
 * @param {number} relativeSpeedMps - Approach rate (negative = closing)
 * @param {number | null} ttc - Time to collision in seconds
 * @returns {"SAFE" | "ATTENTION" | "WARNING" | "CRITICAL"}
 */
export function classifyRisk(dx, dy, range, bearingDeg, relativeSpeedMps, ttc) {
  const absDy = Math.abs(dy);
  const inTravelLane = absDy <= 2.2 && dx >= -1.0;
  const inCorridor = absDy <= 3.8 && dx >= -2.0;

  // CRITICAL condition:
  // - Direct immediate proximity (< 6m)
  // - In path with TTC < 2.5s
  // - In path with range < 12m while closing in
  if (range <= 5.0) {
    return 'CRITICAL';
  }
  if (inTravelLane) {
    if (ttc !== null && ttc <= 2.5) {
      return 'CRITICAL';
    }
    if (range <= 10.0 && relativeSpeedMps < -1.0) {
      return 'CRITICAL';
    }
  }

  // WARNING condition:
  // - In travel lane with TTC <= 4.5s or range <= 20m
  // - In corridor with TTC <= 3.2s or range <= 14m
  if (inTravelLane) {
    if (ttc !== null && ttc <= 4.5) {
      return 'WARNING';
    }
    if (range <= 22.0 && relativeSpeedMps < -0.5) {
      return 'WARNING';
    }
  } else if (inCorridor) {
    if (ttc !== null && ttc <= 3.2) {
      return 'WARNING';
    }
    if (range <= 14.0) {
      return 'WARNING';
    }
  }

  // ATTENTION condition:
  // - Approaching within 45m or in corridor within 35m
  if (ttc !== null && ttc <= 8.0) {
    return 'ATTENTION';
  }
  if (range <= 35.0 && Math.abs(bearingDeg) < 70) {
    return 'ATTENTION';
  }

  return 'SAFE';
}

/**
 * Generates an array of alerts from active obstacles and vehicle diagnostics,
 * sorted by severity (CRITICAL > WARNING > INFO).
 * 
 * @param {Array<Object>} obstacles - Processed obstacles with riskLevel
 * @param {Object} diagnostics - Vehicle diagnostics
 * @param {Object} motion - Ego motion
 * @param {string} timestampIso - ISO timestamp for alert
 * @returns {Array<{id: string, level: "INFO" | "WARNING" | "CRITICAL", source: string, message: string, timestamp: string}>}
 */
export function generateAlerts(obstacles, diagnostics, motion, timestampIso) {
  const alerts = [];
  let alertSeq = 1;

  // 1. Process obstacle alerts
  for (const obs of obstacles) {
    if (obs.riskLevel === 'CRITICAL') {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'CRITICAL',
        source: 'FORWARD_COLLISION_WARNING',
        message: `CRITICAL: ${obs.type} ${obs.id} at ${obs.relativePosition.range}m (TTC: ${obs.timeToCollisionSec ?? 'N/A'}s)`,
        timestamp: timestampIso
      });
    } else if (obs.riskLevel === 'WARNING') {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'WARNING',
        source: 'FORWARD_COLLISION_WARNING',
        message: `Obstacle ${obs.id} detected at ${obs.relativePosition.range}m (TTC: ${obs.timeToCollisionSec ?? 'N/A'}s)`,
        timestamp: timestampIso
      });
    }
  }

  // 2. Process vehicle diagnostic alerts
  if (diagnostics) {
    if (diagnostics.systemHealth === 'CRITICAL') {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'CRITICAL',
        source: 'SYSTEM_HEALTH_MONITOR',
        message: 'CRITICAL: Autonomous compute subsystem degraded',
        timestamp: timestampIso
      });
    } else if (diagnostics.systemHealth === 'DEGRADED') {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'WARNING',
        source: 'SYSTEM_HEALTH_MONITOR',
        message: 'WARNING: Subsystem redundancy operating in degraded mode',
        timestamp: timestampIso
      });
    }

    if (diagnostics.motorTempC && diagnostics.motorTempC > 80.0) {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'WARNING',
        source: 'BATTERY_THERMAL',
        message: `High motor inverter temperature: ${diagnostics.motorTempC.toFixed(1)}°C`,
        timestamp: timestampIso
      });
    }

    if (diagnostics.batteryLevelPct !== undefined && diagnostics.batteryLevelPct < 20.0) {
      alerts.push({
        id: `alt-${String(alertSeq++).padStart(2, '0')}`,
        level: 'WARNING',
        source: 'BATTERY_MONITOR',
        message: `Low traction battery level: ${diagnostics.batteryLevelPct.toFixed(1)}%`,
        timestamp: timestampIso
      });
    }
  }

  // 3. Sort by severity: CRITICAL first, then WARNING, then INFO
  const severityScore = { CRITICAL: 3, WARNING: 2, INFO: 1 };
  alerts.sort((a, b) => (severityScore[b.level] || 0) - (severityScore[a.level] || 0));

  return alerts;
}
