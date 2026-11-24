/**
 * GARUDA VISION — REAL-TIME CHARTS MANAGER
 * Manages 4 high-performance rolling Chart.js telemetry charts:
 * 1. Speed (km/h) vs Time
 * 2. Longitudinal Acceleration (m/s^2)
 * 3. Steering Angle (deg)
 * 4. Obstacle Proximity & Time-To-Collision (TTC) trend
 */

(function (window) {
  'use strict';

  const MAX_HISTORY_POINTS = 45; // Fixed rolling buffer length for buttery-smooth render

  let speedChart = null;
  let accelChart = null;
  let steeringChart = null;
  let proximityChart = null;

  // Cached DOM value badges above charts
  const liveValEls = {};

  // Global Chart.js styling defaults
  function configureChartDefaults() {
    if (typeof Chart === 'undefined') {
      console.warn('Chart.js library not loaded yet');
      return;
    }

    Chart.defaults.color = '#94a3b8';
    Chart.defaults.borderColor = 'rgba(255, 255, 255, 0.07)';
    Chart.defaults.font.family = "'Share Tech Mono', monospace";
    Chart.defaults.font.size = 10;
    Chart.defaults.animation = false; // Disable Chart.js animation for high-frequency streaming
    Chart.defaults.responsive = true;
    Chart.defaults.maintainAspectRatio = false;
  }

  // Common options factory
  function createCommonOptions(yMin, yMax, yStep, unit) {
    return {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      parsing: false,
      normalized: true,
      plugins: {
        legend: { display: false },
        tooltip: {
          enabled: true,
          backgroundColor: 'rgba(10, 14, 23, 0.9)',
          titleColor: '#00e5ff',
          bodyColor: '#ffffff',
          borderColor: 'rgba(0, 229, 255, 0.3)',
          borderWidth: 1,
          padding: 6,
          displayColors: false,
          callbacks: {
            label: context => `${context.parsed.y.toFixed(2)} ${unit}`
          }
        }
      },
      scales: {
        x: {
          display: false,
          grid: { display: false }
        },
        y: {
          min: yMin,
          max: yMax,
          ticks: {
            stepSize: yStep,
            color: '#64748b',
            font: { size: 9 },
            callback: value => `${value}`
          },
          grid: {
            color: 'rgba(255, 255, 255, 0.05)',
            lineWidth: 1
          }
        }
      },
      elements: {
        line: {
          tension: 0.25,
          borderWidth: 1.8
        },
        point: {
          radius: 0,
          hitRadius: 4,
          hoverRadius: 3
        }
      }
    };
  }

  function initCharts() {
    if (typeof Chart === 'undefined') return;

    configureChartDefaults();

    // Cache live value readout labels
    liveValEls.speed = document.getElementById('chart-val-speed');
    liveValEls.accel = document.getElementById('chart-val-accel');
    liveValEls.steer = document.getElementById('chart-val-steer');
    liveValEls.ttc = document.getElementById('chart-val-ttc');

    // 1. Speed Chart
    const ctxSpeed = document.getElementById('chart-speed');
    if (ctxSpeed) {
      speedChart = new Chart(ctxSpeed.getContext('2d'), {
        type: 'line',
        data: {
          labels: Array(MAX_HISTORY_POINTS).fill(''),
          datasets: [{
            label: 'Speed',
            data: Array(MAX_HISTORY_POINTS).fill(0),
            borderColor: '#00e5ff',
            backgroundColor: 'rgba(0, 229, 255, 0.1)',
            fill: true
          }]
        },
        options: createCommonOptions(0, 120, 30, 'km/h')
      });
    }

    // 2. Longitudinal Acceleration Chart
    const ctxAccel = document.getElementById('chart-accel');
    if (ctxAccel) {
      accelChart = new Chart(ctxAccel.getContext('2d'), {
        type: 'line',
        data: {
          labels: Array(MAX_HISTORY_POINTS).fill(''),
          datasets: [{
            label: 'Acceleration',
            data: Array(MAX_HISTORY_POINTS).fill(0),
            borderColor: '#00e676',
            backgroundColor: 'rgba(0, 230, 118, 0.08)',
            fill: true
          }]
        },
        options: createCommonOptions(-6, 4, 2, 'm/s²')
      });
    }

    // 3. Steering Angle Chart
    const ctxSteer = document.getElementById('chart-steering');
    if (ctxSteer) {
      steeringChart = new Chart(ctxSteer.getContext('2d'), {
        type: 'line',
        data: {
          labels: Array(MAX_HISTORY_POINTS).fill(''),
          datasets: [{
            label: 'Steering',
            data: Array(MAX_HISTORY_POINTS).fill(0),
            borderColor: '#ffb300',
            backgroundColor: 'rgba(255, 179, 0, 0.08)',
            fill: true
          }]
        },
        options: createCommonOptions(-35, 35, 15, '°')
      });
    }

    // 4. Proximity & TTC Chart
    const ctxProx = document.getElementById('chart-proximity');
    if (ctxProx) {
      proximityChart = new Chart(ctxProx.getContext('2d'), {
        type: 'line',
        data: {
          labels: Array(MAX_HISTORY_POINTS).fill(''),
          datasets: [
            {
              label: 'Min Range (m)',
              data: Array(MAX_HISTORY_POINTS).fill(60),
              borderColor: '#b388ff',
              backgroundColor: 'rgba(179, 136, 255, 0.08)',
              fill: false
            },
            {
              label: 'Min TTC (s)',
              data: Array(MAX_HISTORY_POINTS).fill(15),
              borderColor: '#ff3d00',
              backgroundColor: 'rgba(255, 61, 0, 0.08)',
              fill: false
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          parsing: false,
          normalized: true,
          plugins: {
            legend: {
              display: true,
              position: 'top',
              align: 'end',
              labels: {
                boxWidth: 8,
                boxHeight: 8,
                color: '#94a3b8',
                font: { size: 9 }
              }
            }
          },
          scales: {
            x: { display: false },
            y: {
              min: 0,
              max: 60,
              ticks: {
                stepSize: 15,
                color: '#64748b',
                font: { size: 9 }
              },
              grid: {
                color: 'rgba(255, 255, 255, 0.05)',
                lineWidth: 1
              }
            }
          },
          elements: {
            line: { tension: 0.2, borderWidth: 1.8 },
            point: { radius: 0 }
          }
        }
      });
    }
  }

  // Rolling buffer update helper
  function pushDataPoint(chart, datasetIndex, value) {
    if (!chart || !chart.data || !chart.data.datasets[datasetIndex]) return;
    const dataset = chart.data.datasets[datasetIndex];
    dataset.data.shift();
    dataset.data.push(value);
  }

  // Update routine called with every telemetry frame
  function update(telemetry) {
    if (!telemetry) return;

    const vehicle = telemetry.vehicle || {};
    const motion = vehicle.motion || {};
    const speedKph = motion.speedKph ?? 0;
    const accel = motion.acceleration ?? 0;
    const steer = motion.steeringAngleDeg ?? 0;

    // Determine nearest obstacle range and TTC
    let minRange = 60.0;
    let minTtc = 15.0;
    const obstacles = telemetry.obstacles || [];

    if (obstacles.length > 0) {
      for (const obs of obstacles) {
        if (obs.relativePosition && typeof obs.relativePosition.range === 'number') {
          if (obs.relativePosition.range < minRange) {
            minRange = obs.relativePosition.range;
          }
        }
        if (typeof obs.timeToCollisionSec === 'number' && obs.timeToCollisionSec > 0) {
          if (obs.timeToCollisionSec < minTtc) {
            minTtc = obs.timeToCollisionSec;
          }
        }
      }
    }

    // 1. Update Speed Chart
    if (speedChart) {
      pushDataPoint(speedChart, 0, speedKph);
      speedChart.update('none');
    }
    if (liveValEls.speed) {
      liveValEls.speed.textContent = `${speedKph.toFixed(1)} km/h`;
    }

    // 2. Update Accel Chart
    if (accelChart) {
      pushDataPoint(accelChart, 0, accel);
      accelChart.update('none');
    }
    if (liveValEls.accel) {
      liveValEls.accel.textContent = `${(accel >= 0 ? '+' : '')}${accel.toFixed(2)} m/s²`;
    }

    // 3. Update Steering Chart
    if (steeringChart) {
      pushDataPoint(steeringChart, 0, steer);
      steeringChart.update('none');
    }
    if (liveValEls.steer) {
      liveValEls.steer.textContent = `${steer.toFixed(1)}°`;
    }

    // 4. Update Proximity Chart
    if (proximityChart) {
      pushDataPoint(proximityChart, 0, minRange);
      pushDataPoint(proximityChart, 1, minTtc);
      proximityChart.update('none');
    }
    if (liveValEls.ttc) {
      if (obstacles.length === 0) {
        liveValEls.ttc.textContent = 'SECTORS CLEAR';
        liveValEls.ttc.style.color = 'var(--accent-emerald)';
      } else {
        liveValEls.ttc.textContent = `R: ${minRange.toFixed(1)}m | TTC: ${minTtc < 15 ? minTtc.toFixed(1) + 's' : 'N/A'}`;
        liveValEls.ttc.style.color = minTtc < 3.0 ? 'var(--accent-red)' : 'var(--accent-cyan)';
      }
    }
  }

  // Reset buffers
  function reset() {
    [speedChart, accelChart, steeringChart].forEach(chart => {
      if (chart && chart.data && chart.data.datasets[0]) {
        chart.data.datasets[0].data = Array(MAX_HISTORY_POINTS).fill(0);
        chart.update('none');
      }
    });
    if (proximityChart && proximityChart.data) {
      if (proximityChart.data.datasets[0]) proximityChart.data.datasets[0].data = Array(MAX_HISTORY_POINTS).fill(60);
      if (proximityChart.data.datasets[1]) proximityChart.data.datasets[1].data = Array(MAX_HISTORY_POINTS).fill(15);
      proximityChart.update('none');
    }
  }

  document.addEventListener('DOMContentLoaded', initCharts);

  window.ChartsManager = {
    init: initCharts,
    update: update,
    reset: reset
  };

})(window);
