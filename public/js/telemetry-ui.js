/**
 * GARUDA VISION — TELEMETRY UI MANAGER
 * Handles high-frequency DOM telemetry updates, SVG gauges, alert banner state,
 * obstacle intelligence cards, and system diagnostics matrix.
 */

(function (window) {
  'use strict';

  // SVG Speedometer Arc Constants
  const SPEEDO_ARC_LENGTH = 283; // Total stroke-dasharray for 180-deg arc
  const SPEEDO_MAX_KPH = 140;

  // Cached DOM Elements
  const el = {};

  function cacheDom() {
    // Header elements
    el.headerProvider = document.getElementById('header-provider');
    el.headerSystemStatus = document.getElementById('header-system-status');
    el.headerTick = document.getElementById('header-tick');
    el.headerFps = document.getElementById('header-fps');
    el.wsDot = document.getElementById('ws-dot');
    el.wsStatusText = document.getElementById('ws-status-text');
    el.wsLatencyText = document.getElementById('ws-latency-text');

    // Vehicle kinematics
    el.egoVehicleId = document.getElementById('ego-vehicle-id');
    el.speedKph = document.getElementById('telemetry-speed-kph');
    el.speedMps = document.getElementById('telemetry-speed-mps');
    el.speedoArc = document.getElementById('speedo-arc');
    el.speedoNeedle = document.getElementById('speedo-needle');
    el.gearPills = document.querySelectorAll('#gear-selector .gear-pill');

    el.accel = document.getElementById('telemetry-accel');
    el.jerk = document.getElementById('telemetry-jerk');
    el.accelBar = document.getElementById('accel-bar');

    el.steerDeg = document.getElementById('telemetry-steer-deg');
    el.yawRate = document.getElementById('telemetry-yaw-rate');
    el.steerWheelIcon = document.getElementById('steering-wheel-icon');
    el.steerBar = document.getElementById('steer-bar');

    el.heading = document.getElementById('telemetry-heading');
    el.pitch = document.getElementById('telemetry-pitch');
    el.roll = document.getElementById('telemetry-roll');
    el.compassRose = document.getElementById('compass-rose');

    el.throttleFill = document.getElementById('throttle-fill');
    el.throttlePctText = document.getElementById('throttle-pct-text');
    el.brakeFill = document.getElementById('brake-fill');
    el.brakePctText = document.getElementById('brake-pct-text');

    el.posX = document.getElementById('telemetry-pos-x');
    el.posY = document.getElementById('telemetry-pos-y');
    el.posZ = document.getElementById('telemetry-pos-z');
    el.odoDist = document.getElementById('telemetry-odo-dist');
    el.odoTime = document.getElementById('telemetry-odo-time');

    el.navRoadSegment = document.getElementById('nav-road-segment');
    el.navLane = document.getElementById('nav-lane');
    el.navNextTurn = document.getElementById('nav-next-turn');
    el.navWaypoint = document.getElementById('nav-waypoint');
    el.navTargetSpeed = document.getElementById('nav-target-speed');

    // Viewport HUD & Alert Banner
    el.alertBanner = document.getElementById('tactical-alert-banner');
    el.alertHeadline = document.getElementById('alert-headline');
    el.alertDetail = document.getElementById('alert-detail');
    el.alertTtcBox = document.getElementById('alert-ttc-box');
    el.alertTtcVal = document.getElementById('alert-ttc-val');

    el.hudStampLat = document.getElementById('hud-stamp-lat');
    el.hudStampLon = document.getElementById('hud-stamp-lon');
    el.hudLidarCount = document.getElementById('hud-lidar-count');

    // Obstacles
    el.obstacleCountBadge = document.getElementById('obstacle-count-badge');
    el.threatSummary = document.getElementById('threat-summary');
    el.obstacleList = document.getElementById('obstacle-list-container');

    // Diagnostics
    el.systemHealthBadge = document.getElementById('system-health-badge');
    el.diagCpuVal = document.getElementById('diag-cpu-val');
    el.diagCpuBar = document.getElementById('diag-cpu-bar');
    el.diagBatteryVal = document.getElementById('diag-battery-val');
    el.diagBatteryBar = document.getElementById('diag-battery-bar');
    el.diagBatteryVolts = document.getElementById('diag-battery-volts');
    el.diagTempVal = document.getElementById('diag-temp-val');
    el.diagTempBar = document.getElementById('diag-temp-bar');
    el.diagLatencyVal = document.getElementById('diag-latency-val');
    el.diagLatencyBar = document.getElementById('diag-latency-bar');

    el.subsysPerception = document.getElementById('subsys-perception');
    el.subsysLocalization = document.getElementById('subsys-localization');
    el.subsysPlanning = document.getElementById('subsys-planning');
    el.subsysControl = document.getElementById('subsys-control');
    el.subsysLidar = document.getElementById('subsys-lidar');
    el.subsysRadar = document.getElementById('subsys-radar');
    el.subsysCamera = document.getElementById('subsys-camera');
  }

  // Format seconds into HH:MM:SS
  function formatSeconds(sec) {
    if (isNaN(sec) || sec < 0) return '00:00:00';
    const s = Math.floor(sec % 60);
    const m = Math.floor((sec / 60) % 60);
    const h = Math.floor(sec / 3600);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  }

  // Main UI update routine called per incoming telemetry packet
  function update(telemetry) {
    if (!telemetry) return;

    const { header, vehicle, diagnostics, obstacles, alerts, navigation, lidar } = telemetry;

    // 1. Header Bar
    if (header) {
      if (el.headerProvider) el.headerProvider.textContent = header.dataSource || 'SYNTHETIC';
      if (el.headerTick) el.headerTick.textContent = String(header.simulationTick ?? header.sequenceNumber ?? 0).padStart(6, '0');
      if (el.headerFps && typeof header.updateRateHz === 'number') {
        el.headerFps.textContent = `${header.updateRateHz.toFixed(1)} Hz`;
      }
    }

    // 2. Vehicle Kinematics
    if (vehicle) {
      if (el.egoVehicleId && vehicle.id) el.egoVehicleId.textContent = vehicle.id;
      if (el.headerSystemStatus && vehicle.status) {
        el.headerSystemStatus.textContent = vehicle.status.replace(/_/g, ' ');
      }

      const motion = vehicle.motion || {};
      const speedKph = motion.speedKph ?? 0;
      const speedMps = motion.speedMps ?? (speedKph / 3.6);

      if (el.speedKph) el.speedKph.textContent = speedKph.toFixed(1);
      if (el.speedMps) el.speedMps.textContent = speedMps.toFixed(2);

      // Speedometer SVG Gauge
      if (el.speedoArc) {
        const speedRatio = Math.min(Math.max(speedKph / SPEEDO_MAX_KPH, 0), 1);
        const dashOffset = SPEEDO_ARC_LENGTH * (1 - speedRatio);
        el.speedoArc.style.strokeDashoffset = dashOffset;
      }

      if (el.speedoNeedle) {
        // Needle sweeps from -90 deg (0 km/h) to +90 deg (140 km/h) around center (120, 130)
        const speedRatio = Math.min(Math.max(speedKph / SPEEDO_MAX_KPH, 0), 1);
        const angleDeg = -90 + (speedRatio * 180);
        el.speedoNeedle.setAttribute('transform', `rotate(${angleDeg} 120 130)`);
      }

      // Gear indicator
      if (motion.gear && el.gearPills) {
        el.gearPills.forEach(pill => {
          if (pill.getAttribute('data-gear') === motion.gear) {
            pill.classList.add('active');
          } else {
            pill.classList.remove('active');
          }
        });
      }

      // Longitudinal Acceleration (-6.0 to +4.0 m/s^2)
      const accel = motion.acceleration ?? 0;
      if (el.accel) {
        el.accel.textContent = (accel >= 0 ? '+' : '') + accel.toFixed(2);
      }
      if (el.jerk) {
        el.jerk.textContent = (motion.jerk ?? 0).toFixed(2);
      }
      if (el.accelBar) {
        // Bipolar bar: 0 is at 60%
        if (accel >= 0) {
          const w = Math.min((accel / 4.0) * 40, 40);
          el.accelBar.style.left = '60%';
          el.accelBar.style.width = `${w}%`;
          el.accelBar.classList.remove('braking');
        } else {
          const w = Math.min((Math.abs(accel) / 6.0) * 60, 60);
          el.accelBar.style.left = `${60 - w}%`;
          el.accelBar.style.width = `${w}%`;
          el.accelBar.classList.add('braking');
        }
      }

      // Steering Angle (-35 to +35 deg) & Yaw Rate
      const steerDeg = motion.steeringAngleDeg ?? 0;
      if (el.steerDeg) el.steerDeg.textContent = `${steerDeg.toFixed(1)}°`;
      if (el.yawRate) el.yawRate.textContent = `${(motion.angularVelocityDeg ?? 0).toFixed(1)} °/s`;
      if (el.steerWheelIcon) {
        el.steerWheelIcon.style.transform = `rotate(${steerDeg}deg)`;
      }
      if (el.steerBar) {
        if (steerDeg >= 0) {
          const w = Math.min((steerDeg / 35) * 50, 50);
          el.steerBar.style.left = '50%';
          el.steerBar.style.width = `${w}%`;
        } else {
          const w = Math.min((Math.abs(steerDeg) / 35) * 50, 50);
          el.steerBar.style.left = `${50 - w}%`;
          el.steerBar.style.width = `${w}%`;
        }
      }

      // Orientation & Compass
      const orientation = vehicle.orientation || {};
      const headingDeg = orientation.headingDeg ?? orientation.yaw ?? 0;
      if (el.heading) el.heading.textContent = `${headingDeg.toFixed(1)}°`;
      if (el.pitch) el.pitch.textContent = `${(orientation.pitch ?? 0).toFixed(1)}°`;
      if (el.roll) el.roll.textContent = `${(orientation.roll ?? 0).toFixed(1)}°`;
      if (el.compassRose) {
        // Heading is CCW in ISO 8855, so compass rose rotates by -headingDeg
        el.compassRose.setAttribute('transform', `rotate(${-headingDeg} 50 50)`);
      }

      // Actuator Demand
      const throttle = motion.throttlePct ?? 0;
      const brake = motion.brakePct ?? 0;
      if (el.throttlePctText) el.throttlePctText.textContent = `${Math.round(throttle)}%`;
      if (el.throttleFill) el.throttleFill.style.width = `${Math.min(Math.max(throttle, 0), 100)}%`;
      if (el.brakePctText) el.brakePctText.textContent = `${Math.round(brake)}%`;
      if (el.brakeFill) el.brakeFill.style.width = `${Math.min(Math.max(brake, 0), 100)}%`;

      // Position & Odometry
      const pos = vehicle.position || {};
      if (el.posX) el.posX.textContent = (pos.x ?? 0).toFixed(2);
      if (el.posY) el.posY.textContent = (pos.y ?? 0).toFixed(2);
      if (el.posZ) el.posZ.textContent = (pos.z ?? 0).toFixed(2);

      const odo = vehicle.odometry || {};
      if (el.odoDist) {
        const distKm = (odo.totalDistanceMeters ?? 0) / 1000;
        el.odoDist.textContent = `${distKm.toFixed(2)} km`;
      }
      if (el.odoTime) {
        el.odoTime.textContent = formatSeconds(odo.tripDurationSeconds ?? 0);
      }

      // Viewport watermark coordinate stamp
      if (el.hudStampLat) el.hudStampLat.textContent = `${(12.9716 + (pos.x || 0) * 0.00001).toFixed(5)}°`;
      if (el.hudStampLon) el.hudStampLon.textContent = `${(77.5946 + (pos.y || 0) * 0.00001).toFixed(5)}°`;
    }

    // 3. Navigation Details
    if (navigation) {
      if (el.navRoadSegment && navigation.currentRoadSegment) {
        el.navRoadSegment.textContent = navigation.currentRoadSegment;
      }
      if (el.navLane && navigation.lanePosition) {
        el.navLane.textContent = navigation.lanePosition;
      }
      if (el.navNextTurn && typeof navigation.distanceToNextTurnMeters === 'number') {
        el.navNextTurn.textContent = `${navigation.distanceToNextTurnMeters.toFixed(0)} m`;
      }
      if (el.navWaypoint) {
        el.navWaypoint.textContent = `${navigation.currentWaypointIndex ?? 0} / ${navigation.totalWaypoints ?? 0}`;
      }
      if (el.navTargetSpeed && typeof navigation.targetSpeedKph === 'number') {
        el.navTargetSpeed.textContent = `${navigation.targetSpeedKph.toFixed(0)} km/h`;
      }
    }

    // 4. LiDAR HUD
    if (lidar && el.hudLidarCount) {
      el.hudLidarCount.textContent = lidar.pointCount ?? (lidar.points ? lidar.points.length : 0);
    }

    // 5. Tactical Alert Banner
    updateAlertBanner(alerts, obstacles);

    // 6. Obstacle Intelligence Cards
    updateObstacles(obstacles);

    // 7. System Diagnostics Matrix
    if (diagnostics) {
      updateDiagnostics(diagnostics);
    }
  }

  // Tactical Alert Banner Logic
  function updateAlertBanner(alerts, obstacles) {
    if (!el.alertBanner) return;

    let highestLevel = 'NOMINAL';
    let headline = 'COLLISION MONITOR: ALL SECTORS CLEAR';
    let detail = 'Perception field clean • Safe trajectory confirmed';
    let minTtc = null;

    // Check alerts array first
    if (Array.isArray(alerts) && alerts.length > 0) {
      const topAlert = alerts[0];
      if (topAlert.level === 'CRITICAL') {
        highestLevel = 'CRITICAL';
        headline = `CRITICAL ALERT: ${topAlert.source.replace(/_/g, ' ')}`;
        detail = topAlert.message;
      } else if (topAlert.level === 'WARNING') {
        highestLevel = 'WARNING';
        headline = `TACTICAL WARNING: ${topAlert.source.replace(/_/g, ' ')}`;
        detail = topAlert.message;
      }
    }

    // Inspect obstacles for min TTC and riskLevel
    if (Array.isArray(obstacles) && obstacles.length > 0) {
      for (const obs of obstacles) {
        if (typeof obs.timeToCollisionSec === 'number' && obs.timeToCollisionSec > 0) {
          if (minTtc === null || obs.timeToCollisionSec < minTtc) {
            minTtc = obs.timeToCollisionSec;
          }
        }
        if (obs.riskLevel === 'CRITICAL' && highestLevel !== 'CRITICAL') {
          highestLevel = 'CRITICAL';
          headline = `CRITICAL: IMMINENT IMPACT HAZARD [${obs.id}]`;
          detail = `Range ${obs.relativePosition?.range?.toFixed(1) || '?'}m • Closing at ${Math.abs(obs.relativeSpeedMps || 0).toFixed(1)} m/s`;
        } else if (obs.riskLevel === 'WARNING' && highestLevel === 'NOMINAL') {
          highestLevel = 'WARNING';
          headline = `WARNING: APPROACHING TARGET [${obs.id}]`;
          detail = `Range ${obs.relativePosition?.range?.toFixed(1) || '?'}m • Predicted TTC ${obs.timeToCollisionSec?.toFixed(1) || '--'}s`;
        }
      }
    }

    // Apply banner styling
    el.alertBanner.className = 'tactical-alert-banner alert-' + highestLevel.toLowerCase();
    if (el.alertHeadline) el.alertHeadline.textContent = headline;
    if (el.alertDetail) el.alertDetail.textContent = detail;

    if (minTtc !== null && el.alertTtcBox && el.alertTtcVal) {
      el.alertTtcBox.style.display = 'flex';
      el.alertTtcVal.textContent = `${minTtc.toFixed(1)}s`;
    } else if (el.alertTtcBox) {
      el.alertTtcBox.style.display = 'none';
    }
  }

  // Obstacle Cards List Rendering
  function updateObstacles(obstacles) {
    if (!el.obstacleList) return;

    if (!Array.isArray(obstacles) || obstacles.length === 0) {
      if (el.obstacleCountBadge) el.obstacleCountBadge.textContent = '0 TARGETS';
      if (el.threatSummary) {
        el.threatSummary.textContent = 'ZERO HAZARDS';
        el.threatSummary.style.color = 'var(--accent-emerald)';
      }
      el.obstacleList.innerHTML = `
        <div class="obstacle-empty-state">
          <div class="empty-icon">&#10003;</div>
          <div>NO OBSTACLES IN SCANNING SECTOR</div>
          <div class="empty-sub">LiDAR and Radar scanning 60m radius</div>
        </div>
      `;
      return;
    }

    // Sort by TTC ascending, then by range ascending
    const sorted = [...obstacles].sort((a, b) => {
      const ttcA = a.timeToCollisionSec ?? 999;
      const ttcB = b.timeToCollisionSec ?? 999;
      return ttcA - ttcB;
    });

    if (el.obstacleCountBadge) {
      el.obstacleCountBadge.textContent = `${sorted.length} TARGET${sorted.length > 1 ? 'S' : ''}`;
    }

    const minTtc = sorted.find(o => typeof o.timeToCollisionSec === 'number')?.timeToCollisionSec;
    if (el.threatSummary) {
      if (minTtc !== undefined) {
        el.threatSummary.textContent = `MIN TTC: ${minTtc.toFixed(1)}s`;
        el.threatSummary.style.color = minTtc < 3.0 ? 'var(--accent-red)' : 'var(--accent-amber)';
      } else {
        el.threatSummary.textContent = 'MONITORING TRACKS';
        el.threatSummary.style.color = 'var(--accent-cyan)';
      }
    }

    // Render cards
    const cardsHtml = sorted.map(obs => {
      const risk = obs.riskLevel || 'SAFE';
      const range = obs.relativePosition?.range?.toFixed(1) ?? '0.0';
      const bearing = obs.relativePosition?.bearingDeg?.toFixed(1) ?? '0.0';
      const relSpeed = obs.relativeSpeedMps !== undefined ? obs.relativeSpeedMps.toFixed(1) : '0.0';
      const ttcText = obs.timeToCollisionSec ? `${obs.timeToCollisionSec.toFixed(1)}s` : 'N/A';
      const conf = obs.classificationConfidence ? `${Math.round(obs.classificationConfidence * 100)}%` : '--';

      return `
        <div class="obstacle-card risk-${risk}">
          <div class="obs-card-header">
            <div class="obs-id-type">
              <span>${obs.id}</span>
              <span class="obs-type-tag">${obs.type}</span>
            </div>
            <span class="obs-risk-badge ${risk}">${risk} (${conf})</span>
          </div>
          <div class="obs-metrics-row">
            <div>RANGE: <span class="obs-val">${range}m</span></div>
            <div>BEARING: <span class="obs-val">${bearing}°</span></div>
            <div>REL VEL: <span class="obs-val">${relSpeed}m/s</span></div>
            <div>TTC: <span class="obs-val">${ttcText}</span></div>
          </div>
        </div>
      `;
    }).join('');

    el.obstacleList.innerHTML = cardsHtml;
  }

  // Diagnostics Matrix Updates
  function updateDiagnostics(diag) {
    if (el.systemHealthBadge && diag.systemHealth) {
      el.systemHealthBadge.textContent = diag.systemHealth;
      if (diag.systemHealth === 'OPTIMAL' || diag.systemHealth === 'NOMINAL') {
        el.systemHealthBadge.style.color = 'var(--accent-emerald)';
      } else if (diag.systemHealth === 'DEGRADED') {
        el.systemHealthBadge.style.color = 'var(--accent-amber)';
      } else {
        el.systemHealthBadge.style.color = 'var(--accent-red)';
      }
    }

    // CPU load
    if (typeof diag.cpuUsagePct === 'number') {
      if (el.diagCpuVal) el.diagCpuVal.textContent = `${diag.cpuUsagePct.toFixed(1)}%`;
      if (el.diagCpuBar) el.diagCpuBar.style.width = `${Math.min(diag.cpuUsagePct, 100)}%`;
    }

    // Battery
    if (typeof diag.batteryLevelPct === 'number') {
      if (el.diagBatteryVal) el.diagBatteryVal.textContent = `${diag.batteryLevelPct.toFixed(1)}%`;
      if (el.diagBatteryBar) el.diagBatteryBar.style.width = `${Math.min(diag.batteryLevelPct, 100)}%`;
    }
    if (typeof diag.batteryVoltage === 'number' && el.diagBatteryVolts) {
      el.diagBatteryVolts.textContent = `${diag.batteryVoltage.toFixed(1)} V`;
    }

    // Inverter Temp
    if (typeof diag.motorTempC === 'number') {
      if (el.diagTempVal) el.diagTempVal.textContent = `${diag.motorTempC.toFixed(1)} °C`;
      if (el.diagTempBar) {
        // Range 20C to 100C
        const tempRatio = Math.min(Math.max((diag.motorTempC - 20) / 80, 0), 1);
        el.diagTempBar.style.width = `${tempRatio * 100}%`;
      }
    }

    // Compute pipeline latency
    if (typeof diag.computeLatencyMs === 'number') {
      if (el.diagLatencyVal) el.diagLatencyVal.textContent = `${diag.computeLatencyMs.toFixed(1)} ms`;
      if (el.diagLatencyBar) {
        const latRatio = Math.min(Math.max(diag.computeLatencyMs / 50, 0), 1);
        el.diagLatencyBar.style.width = `${latRatio * 100}%`;
      }
    }

    // Subsystems
    if (diag.subsystems) {
      updateSubsystemTile(el.subsysPerception, diag.subsystems.perception);
      updateSubsystemTile(el.subsysLocalization, diag.subsystems.localization);
      updateSubsystemTile(el.subsysPlanning, diag.subsystems.planning);
      updateSubsystemTile(el.subsysControl, diag.subsystems.control);
      updateSubsystemTile(el.subsysLidar, diag.subsystems.lidarSensor);
      updateSubsystemTile(el.subsysRadar, diag.subsystems.radarSensor);
      updateSubsystemTile(el.subsysCamera, diag.subsystems.cameraArray);
    }
  }

  function updateSubsystemTile(tileEl, status) {
    if (!tileEl || !status) return;
    const led = tileEl.querySelector('.tile-led');
    const text = tileEl.querySelector('.tile-status');
    if (text) text.textContent = status;

    if (led) {
      led.className = 'tile-led';
      if (status === 'ONLINE' || status === 'HEALTHY') {
        // normal green
      } else if (status === 'DEGRADED' || status === 'STANDBY') {
        led.classList.add('warn');
      } else {
        led.classList.add('error');
      }
    }
  }

  // WebSocket connection indicator
  function updateConnectionStatus(isConnected, latencyMs) {
    if (el.wsDot) {
      el.wsDot.className = 'ws-indicator-dot ' + (isConnected ? 'connected' : 'disconnected');
    }
    if (el.wsStatusText) {
      el.wsStatusText.textContent = isConnected ? 'CONNECTED' : 'DISCONNECTED';
      el.wsStatusText.style.color = isConnected ? 'var(--accent-emerald)' : 'var(--accent-red)';
    }
    if (el.wsLatencyText) {
      el.wsLatencyText.textContent = (isConnected && typeof latencyMs === 'number') ? `${latencyMs}ms` : '--ms';
    }
  }

  // Initialize on DOM ready
  document.addEventListener('DOMContentLoaded', cacheDom);

  // Expose module globally
  window.TelemetryUI = {
    init: cacheDom,
    update: update,
    updateConnectionStatus: updateConnectionStatus
  };

})(window);
