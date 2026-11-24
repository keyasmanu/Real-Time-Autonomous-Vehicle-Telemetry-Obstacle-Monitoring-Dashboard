# GARUDA VISION — Standardized Telemetry Schema Specification

**Version**: 1.0.0  
**Scope**: All Data Providers (`SyntheticDataProvider`, `CarlaDataProvider`) and Consumer Clients (Dashboard, 3D Engine, Analytics).

## Core Principles
1. **Source Independence**: Consumers must never distinguish between data sources except via `header.dataSource`.
2. **Explicit Synthetic Identification**: All generated packets must carry `isSimulated: true` and an explicit synthetic disclaimer.
3. **Deterministic Derivation**: Kinematic parameters (speed, acceleration, heading) must be mathematically derived from spatial displacement, not decoupled random values.
4. **Coordinate Frame**: Right-handed ISO 8855 vehicle/world coordinate convention:
   - $+X$: Forward (Longitudinal)
   - $+Y$: Left (Lateral)
   - $+Z$: Up (Vertical)
   - Yaw / Heading: $0^\circ$ aligned with $+X$, increasing counter-clockwise (towards $+Y$).

---

## Field Definitions

### 1. `header`
- `sequenceNumber` *(integer)*: Monotonically increasing 64-bit frame counter.
- `timestamp` *(string ISO 8601)*: Coordinated Universal Time string.
- `timestampMs` *(integer)*: Unix epoch in milliseconds.
- `dataSource` *(string)*: `"SYNTHETIC"` or `"CARLA_BRIDGE"`.
- `isSimulated` *(boolean)*: Always `true`.
- `simulationTick` *(integer)*: Internal clock tick counter.
- `updateRateHz` *(number)*: Actual emit frequency (default 20 Hz).
- `disclaimer` *(string)*: `"SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS"`.

### 2. `vehicle`
- `id` *(string)*: Unique identifier of ego vehicle (e.g. `"GARUDA-EGO-01"`).
- `status` *(string)*: `"MANUAL" | "AUTONOMOUS_ENGAGED" | "EMERGENCY_STOP" | "CALIBRATING"`.
- `position`: `{ x: number, y: number, z: number }` (meters).
- `orientation`: `{ yaw: deg, pitch: deg, roll: deg, headingDeg: deg, headingRad: rad }`.
- `motion`:
  - `speedKph` *(number)*: Scalar velocity in km/h.
  - `speedMps` *(number)*: Scalar velocity in m/s ($v = \sqrt{\dot{x}^2 + \dot{y}^2}$).
  - `acceleration` *(number)*: Longitudinal acceleration in $\text{m/s}^2$ ($a = \frac{dv}{dt}$).
  - `jerk` *(number)*: Derivative of acceleration in $\text{m/s}^3$.
  - `steeringAngleDeg` *(number)*: Wheel angle in degrees ($[-35^\circ, +35^\circ]$).
  - `angularVelocityDeg` *(number)*: Yaw rate in $\text{deg/s}$ ($\omega = \frac{d\theta}{dt}$).
  - `throttlePct` *(number)*: Pedal input percentage $[0, 100]$.
  - `brakePct` *(number)*: Braking input percentage $[0, 100]$.
  - `gear` *(string)*: `"P" | "R" | "N" | "D"`.
- `odometry`:
  - `totalDistanceMeters` *(number)*: Accumulated travel distance.
  - `tripDurationSeconds` *(number)*: Active simulation elapsed time.

### 3. `diagnostics`
- `systemHealth` *(string)*: `"OPTIMAL" | "NOMINAL" | "DEGRADED" | "CRITICAL"`.
- `cpuUsagePct` *(number)*: Embedded compute load percentage.
- `batteryVoltage` *(number)*: Traction pack voltage in Volts.
- `batteryLevelPct` *(number)*: State of Charge (SoC) $[0, 100]$.
- `motorTempC` *(number)*: Drive inverter / motor temperature in Celsius.
- `computeLatencyMs` *(number)*: Perception-to-actuation pipeline delay.
- `subsystems`: Object containing operational health status for perception, localization, planning, control, LiDAR, radar, and camera array.

### 4. `obstacles`
Array of detected bounding boxes and obstacle vectors:
- `id` *(string)*: Unique object identifier.
- `type` *(string)*: `"VEHICLE" | "PEDESTRIAN" | "CYCLIST" | "STATIC_BARRIER" | "DEBRIS"`.
- `position`: Global world coordinates `{ x, y, z }`.
- `dimensions`: Extents `{ length, width, height }`.
- `relativePosition`: Ego-relative `{ dx, dy, range, bearingDeg }`.
- `velocity`: Global vector `{ vx, vy, speedKph }`.
- `relativeSpeedMps`: Approach rate (negative indicates closing in).
- `timeToCollisionSec`: Calculated TTC in seconds, or `null` if divergent.
- `riskLevel`: `"SAFE" | "ATTENTION" | "WARNING" | "CRITICAL"`.
- `classificationConfidence`: Float between $0.0$ and $1.0$.

### 5. `lidar`
- `sensor` *(string)*: Sensor model name (e.g., `"GARUDA-LIDAR-32"`).
- `pointCount` *(integer)*: Count of active points in frame.
- `rangeMeters` *(number)*: Maximum scan range (e.g., 60.0m).
- `fieldOfViewDeg` *(number)*: Azimuth sweep range (e.g., 360).
- `points`: 2D Array `[ [x, y, z, intensity], ... ]` relative to vehicle sensor origin.

### 6. `alerts`
Array of active notifications, sorted by severity:
- `id`: Alert ID.
- `level`: `"INFO" | "WARNING" | "CRITICAL"`.
- `source`: Subsystem name (e.g. `"FORWARD_COLLISION_WARNING"`, `"BATTERY_THERMAL"`).
- `message`: Human-readable summary.
- `timestamp`: Generation time.

### 7. `navigation`
- `currentWaypointIndex`: Index in planned path.
- `totalWaypoints`: Total count in active route.
- `currentRoadSegment`: Street or sector descriptor.
- `lanePosition`: `"LEFT" | "CENTER" | "RIGHT"`.
- `distanceToNextTurnMeters`: Distance to upcoming maneuver.
- `targetSpeedKph`: Cruise control or path planner speed target.
