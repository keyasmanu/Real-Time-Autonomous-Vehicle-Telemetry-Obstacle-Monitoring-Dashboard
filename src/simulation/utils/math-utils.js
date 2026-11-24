/**
 * GARUDA VISION - Math & Kinematic Utilities
 * 
 * Provides vector math, angle normalization, coordinate transformations
 * between World and Ego frames (ISO 8855 convention), Catmull-Rom spline
 * interpolation, and Box-Muller Gaussian noise generation.
 * 
 * Coordinate System (ISO 8855):
 *  +X: Forward (Longitudinal)
 *  +Y: Left (Lateral)
 *  +Z: Up (Vertical)
 *  Yaw / Heading: 0 deg aligned with +X, counter-clockwise positive (+Y = 90 deg).
 */

/**
 * Converts degrees to radians.
 * @param {number} deg 
 * @returns {number}
 */
export function degToRad(deg) {
  return (deg * Math.PI) / 180.0;
}

/**
 * Converts radians to degrees.
 * @param {number} rad 
 * @returns {number}
 */
export function radToDeg(rad) {
  return (rad * 180.0) / Math.PI;
}

/**
 * Normalizes an angle in radians to the range [-pi, +pi].
 * @param {number} rad 
 * @returns {number}
 */
export function normalizeAngleRad(rad) {
  let angle = rad % (2 * Math.PI);
  if (angle > Math.PI) angle -= 2 * Math.PI;
  if (angle < -Math.PI) angle += 2 * Math.PI;
  return angle === 0 ? 0 : angle;
}

/**
 * Normalizes an angle in degrees to the range [-180, +180].
 * @param {number} deg 
 * @returns {number}
 */
export function normalizeAngleDeg(deg) {
  let angle = deg % 360;
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle === 0 ? 0 : angle;
}

/**
 * Calculates 2D Euclidean distance between two points.
 * @param {{x: number, y: number}} p1 
 * @param {{x: number, y: number}} p2 
 * @returns {number}
 */
export function vec2Distance(p1, p2) {
  return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

/**
 * Calculates 3D Euclidean distance between two points.
 * @param {{x: number, y: number, z?: number}} p1 
 * @param {{x: number, y: number, z?: number}} p2 
 * @returns {number}
 */
export function vec3Distance(p1, p2) {
  const z1 = p1.z ?? 0;
  const z2 = p2.z ?? 0;
  return Math.hypot(p2.x - p1.x, p2.y - p1.y, z2 - z1);
}

/**
 * Calculates the dot product of two 2D vectors.
 * @param {{x: number, y: number}} v1 
 * @param {{x: number, y: number}} v2 
 * @returns {number}
 */
export function vec2Dot(v1, v2) {
  return v1.x * v2.x + v1.y * v2.y;
}

/**
 * Returns a normalized 2D vector.
 * @param {{x: number, y: number}} v 
 * @returns {{x: number, y: number}}
 */
export function vec2Normalize(v) {
  const len = Math.hypot(v.x, v.y);
  if (len < 1e-9) return { x: 0, y: 0 };
  return { x: v.x / len, y: v.y / len };
}

/**
 * Rotates a 2D vector by an angle in radians (counter-clockwise).
 * @param {{x: number, y: number}} v 
 * @param {number} angleRad 
 * @returns {{x: number, y: number}}
 */
export function vec2Rotate(v, angleRad) {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return {
    x: v.x * cos - v.y * sin,
    y: v.x * sin + v.y * cos
  };
}

/**
 * Linear interpolation between a and b by factor t [0, 1].
 * @param {number} a 
 * @param {number} b 
 * @param {number} t 
 * @returns {number}
 */
export function lerp(a, b, t) {
  return a + (b - a) * t;
}

/**
 * Clamps a value between min and max.
 * @param {number} val 
 * @param {number} min 
 * @param {number} max 
 * @returns {number}
 */
export function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

/**
 * Transforms a world-coordinate point into the ego vehicle coordinate frame.
 * ISO 8855: +X is Forward, +Y is Left.
 * 
 * @param {{x: number, y: number}} worldPoint 
 * @param {{x: number, y: number}} vehiclePos 
 * @param {number} headingRad - Vehicle yaw in radians (0 = +X, pi/2 = +Y)
 * @returns {{dx: number, dy: number, range: number, bearingDeg: number}}
 */
export function worldToVehicle2D(worldPoint, vehiclePos, headingRad) {
  const dxWorld = worldPoint.x - vehiclePos.x;
  const dyWorld = worldPoint.y - vehiclePos.y;
  const range = Math.hypot(dxWorld, dyWorld);

  const cos = Math.cos(headingRad);
  const sin = Math.sin(headingRad);

  // Longitudinal (forward +X) and Lateral (left +Y)
  const dx = dxWorld * cos + dyWorld * sin;
  const dy = -dxWorld * sin + dyWorld * cos;

  // Relative bearing in vehicle frame (0 = forward, + = left, - = right)
  const bearingRad = Math.atan2(dy, dx);
  const bearingDeg = radToDeg(bearingRad);

  return { dx, dy, range, bearingDeg };
}

/**
 * Transforms a point in ego vehicle coordinate frame into world coordinates.
 * 
 * @param {{dx: number, dy: number}} vehiclePoint 
 * @param {{x: number, y: number}} vehiclePos 
 * @param {number} headingRad 
 * @returns {{x: number, y: number}}
 */
export function vehicleToWorld2D(vehiclePoint, vehiclePos, headingRad) {
  const cos = Math.cos(headingRad);
  const sin = Math.sin(headingRad);

  return {
    x: vehiclePos.x + vehiclePoint.dx * cos - vehiclePoint.dy * sin,
    y: vehiclePos.y + vehiclePoint.dx * sin + vehiclePoint.dy * cos
  };
}

/**
 * Calculates the relative bearing (in degrees, [-180, +180]) from the vehicle
 * to a target point.
 * 
 * @param {{x: number, y: number}} vehiclePos 
 * @param {number} vehicleHeadingDeg 
 * @param {{x: number, y: number}} targetPos 
 * @returns {number}
 */
export function calcRelativeBearingDeg(vehiclePos, vehicleHeadingDeg, targetPos) {
  const worldAngleRad = Math.atan2(targetPos.y - vehiclePos.y, targetPos.x - vehiclePos.x);
  const worldAngleDeg = radToDeg(worldAngleRad);
  return normalizeAngleDeg(worldAngleDeg - vehicleHeadingDeg);
}

/**
 * Generates normally distributed random numbers (mean, stdDev)
 * using the Box-Muller transform.
 * 
 * @param {number} [mean=0] 
 * @param {number} [stdDev=1] 
 * @returns {number}
 */
export function gaussianNoise(mean = 0, stdDev = 1) {
  let u1 = 0;
  let u2 = 0;
  // Ensure non-zero u1 for logarithm
  while (u1 < 1e-12) u1 = Math.random();
  while (u2 < 1e-12) u2 = Math.random();

  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  return mean + z0 * stdDev;
}

/**
 * Evaluates a 2D Catmull-Rom spline at parameter t [0, 1].
 * 
 * @param {{x: number, y: number, z?: number}} p0 
 * @param {{x: number, y: number, z?: number}} p1 
 * @param {{x: number, y: number, z?: number}} p2 
 * @param {{x: number, y: number, z?: number}} p3 
 * @param {number} t 
 * @returns {{x: number, y: number, z: number}}
 */
export function catmullRom(p0, p1, p2, p3, t) {
  const t2 = t * t;
  const t3 = t2 * t;

  const f1 = -0.5 * t3 + t2 - 0.5 * t;
  const f2 = 1.5 * t3 - 2.5 * t2 + 1.0;
  const f3 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
  const f4 = 0.5 * t3 - 0.5 * t2;

  const z0 = p0.z ?? 0;
  const z1 = p1.z ?? 0;
  const z2 = p2.z ?? 0;
  const z3 = p3.z ?? 0;

  return {
    x: p0.x * f1 + p1.x * f2 + p2.x * f3 + p3.x * f4,
    y: p0.y * f1 + p1.y * f2 + p2.y * f3 + p3.y * f4,
    z: z0 * f1 + z1 * f2 + z2 * f3 + z3 * f4
  };
}

/**
 * Evaluates the tangent vector (derivative with respect to t) of a Catmull-Rom spline.
 * 
 * @param {{x: number, y: number, z?: number}} p0 
 * @param {{x: number, y: number, z?: number}} p1 
 * @param {{x: number, y: number, z?: number}} p2 
 * @param {{x: number, y: number, z?: number}} p3 
 * @param {number} t 
 * @returns {{dx: number, dy: number, dz: number}}
 */
export function catmullRomTangent(p0, p1, p2, p3, t) {
  const t2 = t * t;

  const df1 = -1.5 * t2 + 2.0 * t - 0.5;
  const df2 = 4.5 * t2 - 5.0 * t;
  const df3 = -4.5 * t2 + 4.0 * t + 0.5;
  const df4 = 1.5 * t2 - 1.0 * t;

  const z0 = p0.z ?? 0;
  const z1 = p1.z ?? 0;
  const z2 = p2.z ?? 0;
  const z3 = p3.z ?? 0;

  return {
    dx: p0.x * df1 + p1.x * df2 + p2.x * df3 + p3.x * df4,
    dy: p0.y * df1 + p1.y * df2 + p2.y * df3 + p3.y * df4,
    dz: z0 * df1 + z1 * df2 + z2 * df3 + z3 * df4
  };
}

/**
 * Rounds a number to a specified number of decimal places.
 * @param {number} val 
 * @param {number} decimals 
 * @returns {number}
 */
export function roundTo(val, decimals = 2) {
  const factor = 10 ** decimals;
  return Math.round(val * factor) / factor;
}
