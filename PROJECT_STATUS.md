# PROJECT STATUS & INTEGRATION REGISTER
**Project**: GARUDA VISION — Real-Time Autonomous Vehicle Telemetry & Obstacle Monitoring Dashboard  
**Status Date**: 2026-11-24  
**Current Phase**: Phase 1 — Architecture & Core Module Implementation

---

## 1. File & Module Ownership

| Agent | Responsibility Scope | Owned Paths & Files |
| :--- | :--- | :--- |
| **Agent 0 (Orchestrator)** | Architecture, Shared Contracts, Final Integration | `package.json`, `PROJECT_STATUS.md`, `docs/TELEMETRY_SCHEMA.md` |
| **Agent 1 (Frontend)** | Dashboard UI, Telemetry Cards, Gauges, Controls, Charts | `public/index.html`, `public/css/*`, `public/js/app.js`, `public/js/telemetry-ui.js`, `public/js/charts.js` |
| **Agent 2 (Three.js)** | 3D Environment, Vehicle, Obstacles, LiDAR Point Cloud, Cameras | `public/js/visualization.js` |
| **Agent 3 (Backend)** | Express Server, Static Serving, WebSockets, REST APIs | `src/server.js`, `src/websocket/ws-handler.js` |
| **Agent 4 (Simulation)** | Deterministic Kinematics, Synthetic & CARLA Providers, LiDAR Raycasting, Risk Assessment | `src/simulation/*` |
| **Agent 5 (QA & Docs)** | Verification, Test Suites, Technical Docs, Interview Prep | `tests/*`, `README.md`, `docs/ARCHITECTURE.md`, `docs/INTERVIEW_QUESTIONS.md` |

---

## 2. Standardized Telemetry Interface Contract (v1.0.0)

Both `SyntheticDataProvider` and `CarlaDataProvider` MUST emit payloads strictly conforming to this structure:

```json
{
  "header": {
    "sequenceNumber": 1024,
    "timestamp": "2026-11-24T02:48:50.050Z",
    "timestampMs": 1795574930050,
    "dataSource": "SYNTHETIC",
    "isSimulated": true,
    "simulationTick": 1024,
    "updateRateHz": 20,
    "disclaimer": "SYNTHETIC / SIMULATED DATA - NOT PHYSICAL SENSOR MEASUREMENTS"
  },
  "vehicle": {
    "id": "GARUDA-EGO-01",
    "status": "AUTONOMOUS_ENGAGED",
    "position": { "x": 12.45, "y": 34.82, "z": 0.0 },
    "orientation": { "yaw": 45.2, "pitch": 0.0, "roll": 0.0, "headingDeg": 45.2, "headingRad": 0.7889 },
    "motion": {
      "speedKph": 48.5,
      "speedMps": 13.47,
      "acceleration": 0.85,
      "jerk": -0.05,
      "steeringAngleDeg": 2.1,
      "angularVelocityDeg": 1.4,
      "throttlePct": 38.0,
      "brakePct": 0.0,
      "gear": "D"
    },
    "odometry": {
      "totalDistanceMeters": 1420.5,
      "tripDurationSeconds": 105.4
    }
  },
  "diagnostics": {
    "systemHealth": "OPTIMAL",
    "cpuUsagePct": 18.2,
    "batteryVoltage": 398.4,
    "batteryLevelPct": 86.5,
    "motorTempC": 48.2,
    "computeLatencyMs": 14.2,
    "subsystems": {
      "perception": "HEALTHY",
      "localization": "HEALTHY",
      "planning": "HEALTHY",
      "control": "HEALTHY",
      "lidarSensor": "ONLINE",
      "radarSensor": "ONLINE",
      "cameraArray": "ONLINE"
    }
  },
  "obstacles": [
    {
      "id": "obs-001",
      "type": "VEHICLE",
      "position": { "x": 18.2, "y": 42.1, "z": 0.0 },
      "dimensions": { "length": 4.5, "width": 1.9, "height": 1.5 },
      "relativePosition": { "dx": 5.75, "dy": 7.28, "range": 9.28, "bearingDeg": 12.3 },
      "velocity": { "vx": 8.0, "vy": 7.5, "speedKph": 39.5 },
      "relativeSpeedMps": -2.5,
      "timeToCollisionSec": 3.71,
      "riskLevel": "WARNING",
      "classificationConfidence": 0.96
    }
  ],
  "lidar": {
    "sensor": "GARUDA-LIDAR-32",
    "pointCount": 850,
    "rangeMeters": 60.0,
    "fieldOfViewDeg": 360,
    "points": [ [0.5, 1.2, 0.3, 0.85], ... ]
  },
  "alerts": [
    {
      "id": "alt-01",
      "level": "WARNING",
      "source": "FORWARD_COLLISION_WARNING",
      "message": "Obstacle obs-001 detected at 9.3m (TTC: 3.7s)",
      "timestamp": "2026-11-24T02:48:50.050Z"
    }
  ],
  "navigation": {
    "currentWaypointIndex": 8,
    "totalWaypoints": 40,
    "currentRoadSegment": "Sector-4 Northbound Expressway",
    "lanePosition": "CENTER",
    "distanceToNextTurnMeters": 124.0,
    "targetSpeedKph": 50.0
  }
}
```

---

## 3. Communication Protocol

### WebSocket Commands (Client -> Server)
- `{ "type": "SIM_CONTROL", "command": "START" | "PAUSE" | "RESET" | "STEP" }`
- `{ "type": "SET_RATE", "rateHz": 10 | 20 | 30 | 50 }`
- `{ "type": "SET_PROVIDER", "provider": "SYNTHETIC" | "CARLA" }`
- `{ "type": "TOGGLE_NOISE", "enabled": true | false }`

### REST Endpoints
- `GET /api/status` — Health, active provider, connected clients count.
- `GET /api/telemetry/latest` — Instant JSON snapshot of latest telemetry.
- `POST /api/simulation/control` — REST fallback for simulation management.
- `GET /api/carla/status` — State of CARLA bridge connectivity.

---

## 4. Current Task Status

- [x] Architecture & contracts defined (Agent 0 - Lead Architect)
- [x] Backend server & WebSocket handler (Agent 3 - Backend Developer)
- [x] Simulation engine & modular data providers (Agent 4 - Simulation Engineer)
- [x] Frontend dashboard, gauges & charts (Agent 1 - Frontend Developer)
- [x] Three.js 3D digital twin & LiDAR rendering (Agent 2 - Three.js Specialist)
- [x] Verification, integration & 49/49 passing tests (Agent 0 & Agent 5)
- [x] Comprehensive documentation & technical interview guide (Agent 5 - QA & Docs)

---

## 5. Verification & Test Summary

- Total Automated Tests: 49 passing across 9 test suites (0 failed, 0 skipped).
- Test Execution Command: `npm test`
- Verification Areas:
  1. REST Endpoints & WebSocket streaming lifecycle (`tests/backend-server.test.js`)
  2. DataProvider Interface & CARLA normalization (`tests/data-provider-contract.test.js`)
  3. Collision Risk, relative speed & TTC calculation (`tests/risk-calculator.test.js`)
  4. SyntheticDataProvider lifecycle & state recovery (`tests/synthetic-provider.test.js`)
  5. Strict Telemetry Schema validation with synthetic disclaimer (`tests/telemetry-schema.test.js`)
  6. Deterministic kinematics, position displacement speed derivation, and jerk (`tests/vehicle-kinematics.test.js`)
