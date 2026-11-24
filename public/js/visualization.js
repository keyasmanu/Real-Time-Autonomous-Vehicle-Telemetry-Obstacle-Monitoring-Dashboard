/**
 * ============================================================================
 * GARUDA VISION — 3D Digital Twin & Sensor Visualization Engine
 * ============================================================================
 * File: public/js/visualization.js
 * Author: AGENT 2 — Three.js Visualization Specialist
 * Version: 1.0.0
 * 
 * Coordinate System Convention (ISO 8855 to Three.js Mapping):
 *  - ISO 8855:
 *      +X : Forward (Longitudinal)
 *      +Y : Left (Lateral)
 *      +Z : Up (Vertical)
 *      Yaw: 0 deg aligned with +X, counter-clockwise positive (+Y = 90 deg).
 *  - Three.js Standard World:
 *      +X : East / Forward along +X
 *      +Y : Up (Vertical)
 *      +Z : South / Lateral (-Z = Left, +Z = Right)
 *  - Right-Handed Orthonormal Basis Mapping:
 *      threeX = isoX
 *      threeY = isoZ
 *      threeZ = -isoY
 *      threeRotationY = headingRad (or yaw in radians)
 * ============================================================================
 */

(function (root, factory) {
  const mod = factory();
  if (typeof define === 'function' && define.amd) {
    define([], function () { return mod; });
  }
  if (typeof module === 'object' && module.exports) {
    module.exports = mod;
  }
  if (typeof root !== 'undefined') {
    root.GarudaVisualization = mod;
  }
  if (typeof window !== 'undefined') {
    window.GarudaVisualization = mod;
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // ==========================================================================
  // CONSTANTS & PALETTE
  // ==========================================================================
  const THEME = {
    bgDark: 0x080c14,
    gridMajor: 0x00e5ff,
    gridMinor: 0x111e33,
    roadSurface: 0x0b111a,
    roadMarkingYellow: 0xffb300,
    roadMarkingCyan: 0x00e5ff,
    roadShoulder: 0x1e293b,
    vehicleChassis: 0x0f172a,
    vehicleAccent: 0x00f0ff,
    vehicleGlass: 0x07111e,
    vehicleWheelTire: 0x121418,
    vehicleWheelRim: 0x8899aa,
    headlightColor: 0xd0f0ff,
    taillightNominal: 0x550008,
    taillightBraking: 0xff002e,
    lidarNear: 0x00e5ff,     // Cyan
    lidarMid: 0xb388ff,      // Neon violet / purple
    lidarFar: 0xffd700,      // Hot amber / gold
    riskSafe: 0x00e676,      // Neon Green
    riskAttention: 0x00e5ff, // Cyan
    riskWarning: 0xffb300,   // Tactical Amber
    riskCritical: 0xff1744   // Bright Glowing Red
  };

  const CAMERA_MODES = {
    CHASE: 'CHASE',
    TOP_DOWN: 'TOP_DOWN',
    COCKPIT: 'COCKPIT',
    FREE_ORBIT: 'FREE_ORBIT'
  };

  const MAX_LIDAR_POINTS = 20000;
  const WHEEL_RADIUS = 0.36; // meters
  const WHEELBASE = 2.8;     // meters

  // ==========================================================================
  // MODULE INTERNAL STATE
  // ==========================================================================
  let container = null;
  let scene = null;
  let camera = null;
  let renderer = null;
  let orbitControls = null;
  let fallbackOrbitActive = false;
  let clock = null;
  let animationFrameId = null;
  let resizeObserver = null;

  // Visualization Layers & Toggles
  let layers = {
    lidar: true,
    obstacles: true,
    grid: true,
    trajectory: true
  };

  // Active Camera Mode
  let currentCameraMode = CAMERA_MODES.CHASE;

  // Scene Component Groups
  let environmentGroup = null;
  let gridHelper = null;
  let roadMesh = null;
  let trajectoryLine = null;
  let vehicleGroup = null;
  let obstaclesGroup = null;
  let lidarPointCloud = null;
  let lidarPointsGeometry = null;
  let lidarPositions = null;
  let lidarColors = null;
  let lidarPuckMesh = null;
  let lidarScanFan = null;
  let compassHudGizmo = null;

  // Vehicle Sub-Components
  let steerGroupFL = null;
  let steerGroupFR = null;
  let wheelFL = null;
  let wheelFR = null;
  let wheelRL = null;
  let wheelRR = null;
  let wheelSpinAngle = 0;
  let tailLightMesh = null;
  let tailLightMaterial = null;
  let brakePointLight = null;
  let headLightL = null;
  let headLightR = null;

  // Active Obstacles Map: id -> { group, bboxMesh, wireframeMesh, labelSprite, canvas, ctx, lastData, currentPos, targetPos }
  const activeObstacles = new Map();

  // Telemetry Interpolation States
  const currentVehicleState = {
    position: null, // THREE.Vector3
    headingRad: 0,
    speedMps: 0,
    steeringRad: 0,
    brakePct: 0,
    acceleration: 0
  };

  const targetVehicleState = {
    position: null, // THREE.Vector3
    headingRad: 0,
    speedMps: 0,
    steeringRad: 0,
    brakePct: 0,
    acceleration: 0
  };

  // Reusable Math Temporaries (Zero Garbage Collection)
  let tmpVecA = null;
  let tmpVecB = null;
  let tmpVecC = null;
  let tmpQuat = null;
  let tmpEuler = null;

  // Fallback Orbit Controls State (if THREE.OrbitControls is unavailable)
  const fallbackOrbit = {
    isDragging: false,
    previousMouseX: 0,
    previousMouseY: 0,
    azimuth: 0,
    elevation: Math.PI / 6,
    distance: 18.0,
    target: null
  };

  // ==========================================================================
  // HELPER UTILITIES
  // ==========================================================================

  /**
   * Transforms an ISO 8855 coordinate to Three.js standard space.
   * +X Forward -> Three.js +X
   * +Y Left    -> Three.js -Z
   * +Z Up      -> Three.js +Y
   */
  function isoToThree(isoX, isoY, isoZ, outVec) {
    const x = Number(isoX) || 0;
    const y = Number(isoY) || 0;
    const z = Number(isoZ) || 0;
    if (outVec) {
      outVec.set(x, z, -y);
      return outVec;
    }
    return new THREE.Vector3(x, z, -y);
  }

  /**
   * Normalizes an angle in radians to [-PI, +PI].
   */
  function normalizeAngleRad(rad) {
    let a = rad % (2 * Math.PI);
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * Resolves numerical hex color from risk level string.
   */
  function getRiskColor(riskLevel) {
    switch (String(riskLevel).toUpperCase()) {
      case 'CRITICAL':
        return THEME.riskCritical;
      case 'WARNING':
        return THEME.riskWarning;
      case 'ATTENTION':
        return THEME.riskAttention;
      case 'SAFE':
      default:
        return THEME.riskSafe;
    }
  }

  /**
   * Resolves CSS hex string from risk level string.
   */
  function getRiskColorCSS(riskLevel) {
    switch (String(riskLevel).toUpperCase()) {
      case 'CRITICAL':
        return '#ff1744';
      case 'WARNING':
        return '#ffb300';
      case 'ATTENTION':
        return '#00e5ff';
      case 'SAFE':
      default:
        return '#00e676';
    }
  }

  /**
   * Creates a procedural soft glowing circle texture for LiDAR points.
   */
  function createPointTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    const grad = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
    grad.addColorStop(0, 'rgba(255, 255, 255, 1.0)');
    grad.addColorStop(0.25, 'rgba(255, 255, 255, 0.9)');
    grad.addColorStop(0.55, 'rgba(0, 229, 255, 0.5)');
    grad.addColorStop(1, 'rgba(0, 0, 0, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 64, 64);

    const texture = new THREE.CanvasTexture(canvas);
    texture.generateMipmaps = false;
    texture.minFilter = THREE.LinearFilter;
    return texture;
  }

  // ==========================================================================
  // PROCEDURAL EGO VEHICLE GENERATOR
  // ==========================================================================
  function createEgoVehicle() {
    vehicleGroup = new THREE.Group();
    vehicleGroup.name = 'EgoVehicle';

    // Materials
    const chassisMat = new THREE.MeshStandardMaterial({
      color: THEME.vehicleChassis,
      metalness: 0.88,
      roughness: 0.22,
      envMapIntensity: 1.2
    });

    const carbonMat = new THREE.MeshStandardMaterial({
      color: 0x090d14,
      metalness: 0.4,
      roughness: 0.8
    });

    const accentMat = new THREE.MeshBasicMaterial({
      color: THEME.vehicleAccent
    });

    const glassMat = new THREE.MeshPhysicalMaterial ? new THREE.MeshPhysicalMaterial({
      color: THEME.vehicleGlass,
      metalness: 0.9,
      roughness: 0.08,
      transmission: 0.6,
      transparent: true,
      opacity: 0.82,
      depthWrite: false
    }) : new THREE.MeshStandardMaterial({
      color: THEME.vehicleGlass,
      metalness: 0.9,
      roughness: 0.1,
      transparent: true,
      opacity: 0.8
    });

    // 1. Lower Aerodynamic Hull / Chassis
    // Length: 4.6m (X), Height: 0.55m (Y), Width: 1.95m (Z)
    const hullGeo = new THREE.BoxGeometry(4.6, 0.55, 1.95);
    const hullMesh = new THREE.Mesh(hullGeo, chassisMat);
    hullMesh.position.set(0, 0.45, 0);
    hullMesh.castShadow = true;
    hullMesh.receiveShadow = true;
    vehicleGroup.add(hullMesh);

    // Front Nose / Splitter
    const noseGeo = new THREE.BoxGeometry(0.8, 0.22, 1.9);
    const noseMesh = new THREE.Mesh(noseGeo, carbonMat);
    noseMesh.position.set(2.4, 0.22, 0);
    noseMesh.castShadow = true;
    vehicleGroup.add(noseMesh);

    // Rear Diffuser
    const diffuserGeo = new THREE.BoxGeometry(0.7, 0.28, 1.88);
    const diffuserMesh = new THREE.Mesh(diffuserGeo, carbonMat);
    diffuserMesh.position.set(-2.38, 0.25, 0);
    diffuserMesh.castShadow = true;
    vehicleGroup.add(diffuserMesh);

    // Aerodynamic Hood Slope
    const hoodGeo = new THREE.BoxGeometry(1.6, 0.15, 1.7);
    const hoodMesh = new THREE.Mesh(hoodGeo, chassisMat);
    hoodMesh.position.set(1.4, 0.72, 0);
    hoodMesh.rotation.z = -0.06;
    hoodMesh.castShadow = true;
    vehicleGroup.add(hoodMesh);

    // Cyber Neon Accent Trim Lines (Left & Right)
    const stripGeo = new THREE.BoxGeometry(3.6, 0.04, 0.03);
    const stripL = new THREE.Mesh(stripGeo, accentMat);
    stripL.position.set(0, 0.55, -0.99);
    vehicleGroup.add(stripL);

    const stripR = new THREE.Mesh(stripGeo, accentMat);
    stripR.position.set(0, 0.55, 0.99);
    vehicleGroup.add(stripR);

    // 2. Cockpit Canopy / Windshield
    const cabinGeo = new THREE.BoxGeometry(2.3, 0.65, 1.48);
    const cabinMesh = new THREE.Mesh(cabinGeo, glassMat);
    cabinMesh.position.set(-0.1, 0.98, 0);
    cabinMesh.castShadow = true;
    vehicleGroup.add(cabinMesh);

    // Interior Dashboard Glow (Visible through glass)
    const dashGeo = new THREE.BoxGeometry(0.5, 0.1, 0.8);
    const dashMesh = new THREE.Mesh(dashGeo, new THREE.MeshBasicMaterial({ color: 0x00e5ff }));
    dashMesh.position.set(0.7, 0.85, 0);
    vehicleGroup.add(dashMesh);

    // 3. Dual Front LED Headlights & Spotlights
    const headlightLensGeo = new THREE.BoxGeometry(0.12, 0.1, 0.4);
    const headlightMat = new THREE.MeshBasicMaterial({ color: 0xe0f7ff });

    const headLensL = new THREE.Mesh(headlightLensGeo, headlightMat);
    headLensL.position.set(2.32, 0.52, -0.7);
    vehicleGroup.add(headLensL);

    const headLensR = new THREE.Mesh(headlightLensGeo, headlightMat);
    headLensR.position.set(2.32, 0.52, 0.7);
    vehicleGroup.add(headLensR);

    // Headlight Spotlights (illuminating road ahead)
    headLightL = new THREE.SpotLight(THEME.headlightColor, 3.2, 50, Math.PI / 7, 0.45, 1.2);
    headLightL.position.set(2.35, 0.55, -0.7);
    const targetL = new THREE.Object3D();
    targetL.position.set(30, 0, -0.7);
    vehicleGroup.add(targetL);
    headLightL.target = targetL;
    vehicleGroup.add(headLightL);

    headLightR = new THREE.SpotLight(THEME.headlightColor, 3.2, 50, Math.PI / 7, 0.45, 1.2);
    headLightR.position.set(2.35, 0.55, 0.7);
    const targetR = new THREE.Object3D();
    targetR.position.set(30, 0, 0.7);
    vehicleGroup.add(targetR);
    headLightR.target = targetR;
    vehicleGroup.add(headLightR);

    // 4. Rear Cyberpunk Tail Light Bar
    const tailBarGeo = new THREE.BoxGeometry(0.08, 0.1, 1.84);
    tailLightMaterial = new THREE.MeshStandardMaterial({
      color: THEME.taillightNominal,
      emissive: THEME.taillightNominal,
      emissiveIntensity: 0.9,
      roughness: 0.3
    });
    tailLightMesh = new THREE.Mesh(tailBarGeo, tailLightMaterial);
    tailLightMesh.position.set(-2.32, 0.62, 0);
    vehicleGroup.add(tailLightMesh);

    // Active Brake Point Light (ground reflection)
    brakePointLight = new THREE.PointLight(0xff002e, 0, 8, 2.0);
    brakePointLight.position.set(-2.6, 0.6, 0);
    vehicleGroup.add(brakePointLight);

    // 5. Roof-Mounted Autonomous Sensor Rack & LiDAR Puck
    const rackBaseGeo = new THREE.BoxGeometry(0.7, 0.08, 0.6);
    const rackMesh = new THREE.Mesh(rackBaseGeo, carbonMat);
    rackMesh.position.set(0.1, 1.34, 0);
    vehicleGroup.add(rackMesh);

    // Rotating LiDAR Puck Cylinder
    const puckGeo = new THREE.CylinderGeometry(0.14, 0.14, 0.18, 24);
    const puckMat = new THREE.MeshStandardMaterial({
      color: 0x111622,
      metalness: 0.9,
      roughness: 0.2
    });
    lidarPuckMesh = new THREE.Mesh(puckGeo, puckMat);
    lidarPuckMesh.position.set(0.1, 1.48, 0);

    // Optical Window Stripe on Puck
    const windowRingGeo = new THREE.CylinderGeometry(0.142, 0.142, 0.06, 24);
    const windowRingMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
    const windowRing = new THREE.Mesh(windowRingGeo, windowRingMat);
    windowRing.position.set(0, 0, 0);
    lidarPuckMesh.add(windowRing);
    vehicleGroup.add(lidarPuckMesh);

    // 6. Wheels & Steering Groups
    const tireMat = new THREE.MeshStandardMaterial({
      color: THEME.vehicleWheelTire,
      roughness: 0.85,
      metalness: 0.1
    });

    const rimMat = new THREE.MeshStandardMaterial({
      color: THEME.vehicleWheelRim,
      roughness: 0.25,
      metalness: 0.85
    });

    function buildWheelAssembly() {
      const wheelAssembly = new THREE.Group();
      // Tire cylinder along local Z axis
      const tireGeo = new THREE.CylinderGeometry(WHEEL_RADIUS, WHEEL_RADIUS, 0.26, 24);
      tireGeo.rotateX(Math.PI / 2);
      const tire = new THREE.Mesh(tireGeo, tireMat);
      tire.castShadow = true;
      wheelAssembly.add(tire);

      // Rim
      const rimGeo = new THREE.CylinderGeometry(WHEEL_RADIUS * 0.68, WHEEL_RADIUS * 0.68, 0.27, 16);
      rimGeo.rotateX(Math.PI / 2);
      const rim = new THREE.Mesh(rimGeo, rimMat);
      wheelAssembly.add(rim);

      // Center Hub Accent
      const hubGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.28, 12);
      hubGeo.rotateX(Math.PI / 2);
      const hub = new THREE.Mesh(hubGeo, accentMat);
      wheelAssembly.add(hub);

      return wheelAssembly;
    }

    // Rear Wheels (fixed rotation)
    wheelRL = buildWheelAssembly();
    wheelRL.position.set(-1.4, WHEEL_RADIUS, -0.92);
    vehicleGroup.add(wheelRL);

    wheelRR = buildWheelAssembly();
    wheelRR.position.set(-1.4, WHEEL_RADIUS, 0.92);
    vehicleGroup.add(wheelRR);

    // Front Wheels (with steering yaw pivot)
    steerGroupFL = new THREE.Group();
    steerGroupFL.position.set(1.4, WHEEL_RADIUS, -0.92);
    wheelFL = buildWheelAssembly();
    steerGroupFL.add(wheelFL);
    vehicleGroup.add(steerGroupFL);

    steerGroupFR = new THREE.Group();
    steerGroupFR.position.set(1.4, WHEEL_RADIUS, 0.92);
    wheelFR = buildWheelAssembly();
    steerGroupFR.add(wheelFR);
    vehicleGroup.add(steerGroupFR);

    // 7. Vehicle Underglow Ground Illumination
    const underglowGeo = new THREE.PlaneGeometry(3.6, 1.8);
    underglowGeo.rotateX(-Math.PI / 2);
    const underglowMat = new THREE.MeshBasicMaterial({
      color: 0x00e5ff,
      transparent: true,
      opacity: 0.18,
      depthWrite: false
    });
    const underglowMesh = new THREE.Mesh(underglowGeo, underglowMat);
    underglowMesh.position.set(0, 0.04, 0);
    vehicleGroup.add(underglowMesh);

    // Ground Proximity Sensor Range Rings (10m, 25m, 50m circles)
    const rangeRingMat = new THREE.LineBasicMaterial({
      color: 0x00e5ff,
      transparent: true,
      opacity: 0.22
    });

    [10, 25, 50].forEach(radius => {
      const ringGeo = new THREE.BufferGeometry();
      const pts = [];
      const segs = 64;
      for (let i = 0; i <= segs; i++) {
        const theta = (i / segs) * Math.PI * 2;
        pts.push(new THREE.Vector3(Math.cos(theta) * radius, 0.02, Math.sin(theta) * radius));
      }
      ringGeo.setFromPoints(pts);
      const ringLine = new THREE.Line(ringGeo, rangeRingMat);
      vehicleGroup.add(ringLine);
    });

    return vehicleGroup;
  }

  // ==========================================================================
  // ENVIRONMENT, ROAD & TACTICAL GRID
  // ==========================================================================
  function createEnvironment() {
    environmentGroup = new THREE.Group();
    environmentGroup.name = 'EnvironmentGroup';

    // 1. Dark Atmospheric Ground Plane
    const groundGeo = new THREE.PlaneGeometry(800, 800);
    groundGeo.rotateX(-Math.PI / 2);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x05080e,
      roughness: 0.95,
      metalness: 0.1
    });
    const groundMesh = new THREE.Mesh(groundGeo, groundMat);
    groundMesh.receiveShadow = true;
    groundMesh.position.set(0, -0.01, 0);
    environmentGroup.add(groundMesh);

    // 2. High-Tech Expressway Road Strip
    // Length: 600m along X, Width: 16m along Z
    const roadGeo = new THREE.PlaneGeometry(600, 16);
    roadGeo.rotateX(-Math.PI / 2);
    const roadMat = new THREE.MeshStandardMaterial({
      color: THEME.roadSurface,
      roughness: 0.82,
      metalness: 0.18
    });
    roadMesh = new THREE.Mesh(roadGeo, roadMat);
    roadMesh.position.set(0, 0.005, 0);
    roadMesh.receiveShadow = true;
    environmentGroup.add(roadMesh);

    // Dashed Center Road Markings
    const dashCount = 80;
    const dashLen = 3.5;
    const dashGap = 4.0;
    const dashGroup = new THREE.Group();
    const dashMat = new THREE.MeshBasicMaterial({ color: THEME.roadMarkingYellow });
    const dashGeo = new THREE.PlaneGeometry(dashLen, 0.22);
    dashGeo.rotateX(-Math.PI / 2);

    for (let i = -dashCount / 2; i < dashCount / 2; i++) {
      const dash = new THREE.Mesh(dashGeo, dashMat);
      dash.position.set(i * (dashLen + dashGap), 0.01, 0);
      dashGroup.add(dash);
    }
    environmentGroup.add(dashGroup);

    // Outer Solid Curb Lines (Glowing Neon Cyan)
    const curbGeo = new THREE.PlaneGeometry(600, 0.18);
    curbGeo.rotateX(-Math.PI / 2);
    const curbMat = new THREE.MeshBasicMaterial({ color: THEME.roadMarkingCyan });

    const curbLeft = new THREE.Mesh(curbGeo, curbMat);
    curbLeft.position.set(0, 0.01, -7.5);
    environmentGroup.add(curbLeft);

    const curbRight = new THREE.Mesh(curbGeo, curbMat);
    curbRight.position.set(0, 0.01, 7.5);
    environmentGroup.add(curbRight);

    // Outer Guardrail Shoulder
    const railGeo = new THREE.BoxGeometry(600, 0.45, 0.2);
    const railMat = new THREE.MeshStandardMaterial({ color: THEME.roadShoulder, metalness: 0.6, roughness: 0.4 });

    const railL = new THREE.Mesh(railGeo, railMat);
    railL.position.set(0, 0.22, -8.1);
    environmentGroup.add(railL);

    const railR = new THREE.Mesh(railGeo, railMat);
    railR.position.set(0, 0.22, 8.1);
    environmentGroup.add(railR);

    // 3. Tactical Coordinate Grid
    gridHelper = new THREE.GridHelper(400, 100, THEME.gridMajor, THEME.gridMinor);
    gridHelper.position.set(0, 0.015, 0);
    gridHelper.material.transparent = true;
    gridHelper.material.opacity = 0.35;
    environmentGroup.add(gridHelper);

    // 4. Subtle Coordinate Axes Marker (Origin Reference)
    const axesHelper = new THREE.AxesHelper(3.0);
    axesHelper.position.set(0, 0.05, 0);
    environmentGroup.add(axesHelper);

    return environmentGroup;
  }

  // ==========================================================================
  // LIDAR POINT CLOUD ENGINE
  // ==========================================================================
  function createLidarPointCloud() {
    lidarPositions = new Float32Array(MAX_LIDAR_POINTS * 3);
    lidarColors = new Float32Array(MAX_LIDAR_POINTS * 3);

    lidarPointsGeometry = new THREE.BufferGeometry();
    lidarPointsGeometry.setAttribute('position', new THREE.BufferAttribute(lidarPositions, 3));
    lidarPointsGeometry.setAttribute('color', new THREE.BufferAttribute(lidarColors, 3));
    lidarPointsGeometry.setDrawRange(0, 0);

    const pointTexture = createPointTexture();

    const lidarMaterial = new THREE.PointsMaterial({
      size: 0.22,
      map: pointTexture,
      transparent: true,
      opacity: 0.88,
      vertexColors: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      sizeAttenuation: true
    });

    lidarPointCloud = new THREE.Points(lidarPointsGeometry, lidarMaterial);
    lidarPointCloud.name = 'LidarPointCloud';

    // LiDAR points are expressed relative to vehicle sensor origin:
    // Attaching directly to vehicleGroup guarantees lock-step synchronization!
    vehicleGroup.add(lidarPointCloud);

    // Scanning Radar Fan / Beam Ring
    const fanGeo = new THREE.RingGeometry(0.2, 55, 64, 1, 0, Math.PI / 4);
    fanGeo.rotateX(-Math.PI / 2);
    const fanMat = new THREE.MeshBasicMaterial({
      color: 0x00e5ff,
      transparent: true,
      opacity: 0.08,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    lidarScanFan = new THREE.Mesh(fanGeo, fanMat);
    lidarScanFan.position.set(0, 0.04, 0);
    vehicleGroup.add(lidarScanFan);

    return lidarPointCloud;
  }

  /**
   * Updates LiDAR point cloud buffer attributes in-place.
   */
  function updateLidarBuffer(pointsArray) {
    if (!lidarPointsGeometry || !pointsArray || !layers.lidar) {
      if (lidarPointsGeometry) lidarPointsGeometry.setDrawRange(0, 0);
      return;
    }

    const count = Math.min(pointsArray.length, MAX_LIDAR_POINTS);
    const pos = lidarPositions;
    const col = lidarColors;

    // Pre-allocated RGB color objects for interpolation
    const cLow = new THREE.Color(THEME.lidarNear);
    const cMid = new THREE.Color(THEME.lidarMid);
    const cHigh = new THREE.Color(THEME.lidarFar);
    const ptCol = new THREE.Color();

    for (let i = 0; i < count; i++) {
      const pt = pointsArray[i];
      // Format: [x, y, z, intensity] in ISO 8855 vehicle frame
      const x = pt[0] || 0;
      const y = pt[1] || 0;
      const z = pt[2] || 0;
      const intensity = typeof pt[3] === 'number' ? Math.max(0, Math.min(1, pt[3])) : 0.5;

      const idx = i * 3;
      // Map to Three.js vehicle local coordinates (+X forward, +Y up, -Z left)
      pos[idx] = x;
      pos[idx + 1] = z + 1.48; // Elevate to LiDAR sensor puck height
      pos[idx + 2] = -y;

      // Color mapping: blend based on intensity and elevation
      if (intensity < 0.5) {
        ptCol.copy(cLow).lerp(cMid, intensity * 2.0);
      } else {
        ptCol.copy(cMid).lerp(cHigh, (intensity - 0.5) * 2.0);
      }

      col[idx] = ptCol.r;
      col[idx + 1] = ptCol.g;
      col[idx + 2] = ptCol.b;
    }

    lidarPointsGeometry.attributes.position.needsUpdate = true;
    lidarPointsGeometry.attributes.color.needsUpdate = true;
    lidarPointsGeometry.setDrawRange(0, count);
  }

  // ==========================================================================
  // DYNAMIC OBSTACLE ENGINE & HUD BADGES
  // ==========================================================================

  /**
   * Generates or updates a high-DPI billboard HUD canvas badge for an obstacle.
   */
  function updateObstacleCanvasBadge(obsData, canvas, ctx) {
    const dpr = 2;
    const w = 380;
    const h = 180;

    if (canvas.width !== w * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    const riskColor = getRiskColorCSS(obsData.riskLevel || 'SAFE');
    const riskText = (obsData.riskLevel || 'SAFE').toUpperCase();
    const obsId = obsData.id || 'TARGET';
    const obsType = (obsData.type || 'OBSTACLE').toUpperCase();
    const conf = Math.round((obsData.classificationConfidence || 0.95) * 100);
    const range = (obsData.relativePosition && typeof obsData.relativePosition.range === 'number')
      ? `${obsData.relativePosition.range.toFixed(1)}m`
      : '--m';
    const ttc = (typeof obsData.timeToCollisionSec === 'number')
      ? `${obsData.timeToCollisionSec.toFixed(1)}s`
      : 'NONE';

    // Badge Background Box
    ctx.fillStyle = 'rgba(8, 14, 26, 0.88)';
    ctx.strokeStyle = riskColor;
    ctx.lineWidth = 2.5;

    // Rounded rectangle
    const r = 8;
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(w - r, 0);
    ctx.quadraticCurveTo(w, 0, w, r);
    ctx.lineTo(w, h - r);
    ctx.quadraticCurveTo(w, h, w - r, h);
    ctx.lineTo(r, h);
    ctx.quadraticCurveTo(0, h, 0, h - r);
    ctx.lineTo(0, r);
    ctx.quadraticCurveTo(0, 0, r, 0);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Top Header Banner
    ctx.fillStyle = riskColor;
    ctx.font = 'bold 20px "Share Tech Mono", monospace, sans-serif';
    ctx.fillText(`[${obsType}] ${obsId}`, 18, 36);

    // Confidence Tag
    ctx.fillStyle = '#a0aec0';
    ctx.font = '16px "Share Tech Mono", monospace, sans-serif';
    ctx.fillText(`${conf}% CONF`, w - 105, 36);

    // Divider Line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(18, 52);
    ctx.lineTo(w - 18, 52);
    ctx.stroke();

    // Telemetry Line 1: Range & TTC
    ctx.fillStyle = '#e2e8f0';
    ctx.font = '22px "Rajdhani", sans-serif';
    ctx.fillText(`RANGE: ${range}`, 18, 86);
    ctx.fillText(`TTC: ${ttc}`, 190, 86);

    // Telemetry Line 2: Relative Speed / Velocity
    const speedKph = (obsData.velocity && typeof obsData.velocity.speedKph === 'number')
      ? `${obsData.velocity.speedKph.toFixed(1)} km/h`
      : '--';
    ctx.fillStyle = '#94a3b8';
    ctx.font = '18px "Rajdhani", sans-serif';
    ctx.fillText(`VELOCITY: ${speedKph}`, 18, 120);

    // Bottom Risk Pill Badge
    ctx.fillStyle = riskColor;
    ctx.beginPath();
    ctx.roundRect ? ctx.roundRect(18, 138, 140, 28, 4) : ctx.rect(18, 138, 140, 28);
    ctx.fill();

    ctx.fillStyle = '#080c14';
    ctx.font = 'bold 16px "Share Tech Mono", monospace, sans-serif';
    ctx.fillText(riskText, 32, 158);

    ctx.restore();
  }

  /**
   * Creates a stylized 3D obstacle representation.
   */
  function createObstacleObject(obsData) {
    const group = new THREE.Group();
    group.name = `Obstacle_${obsData.id}`;

    const dims = obsData.dimensions || { length: 4.2, width: 1.9, height: 1.5 };
    const length = Number(dims.length) || 4.2; // Along X
    const width = Number(dims.width) || 1.9;   // Along Z
    const height = Number(dims.height) || 1.5; // Along Y

    const riskColor = getRiskColor(obsData.riskLevel);

    // 1. Outer Wireframe Bounding Box
    const boxGeo = new THREE.BoxGeometry(length, height, width);
    const edgesGeo = new THREE.EdgesGeometry(boxGeo);
    const wireframeMat = new THREE.LineBasicMaterial({
      color: riskColor,
      linewidth: 2,
      transparent: true,
      opacity: 0.95
    });
    const wireframeMesh = new THREE.LineSegments(edgesGeo, wireframeMat);
    wireframeMesh.position.set(0, height / 2, 0);
    group.add(wireframeMesh);

    // 2. Translucent Inner Box
    const fillMat = new THREE.MeshBasicMaterial({
      color: riskColor,
      transparent: true,
      opacity: 0.2,
      depthWrite: false
    });
    const bboxMesh = new THREE.Mesh(boxGeo, fillMat);
    bboxMesh.position.set(0, height / 2, 0);
    group.add(bboxMesh);

    // 3. Type-Specific Internal Hologram
    const type = (obsData.type || 'VEHICLE').toUpperCase();
    if (type === 'VEHICLE') {
      const carCoreGeo = new THREE.BoxGeometry(length * 0.85, height * 0.55, width * 0.85);
      const carCoreMat = new THREE.MeshStandardMaterial({
        color: 0x1a2638,
        metalness: 0.8,
        roughness: 0.3
      });
      const carCore = new THREE.Mesh(carCoreGeo, carCoreMat);
      carCore.position.set(0, height * 0.35, 0);
      group.add(carCore);
    } else if (type === 'PEDESTRIAN' || type === 'CYCLIST') {
      const pedGeo = new THREE.CylinderGeometry(0.3, 0.3, height * 0.8, 12);
      const pedMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.5 });
      const pedMesh = new THREE.Mesh(pedGeo, pedMat);
      pedMesh.position.set(0, height * 0.45, 0);
      group.add(pedMesh);

      const headGeo = new THREE.SphereGeometry(0.22, 12, 12);
      const headMesh = new THREE.Mesh(headGeo, pedMat);
      headMesh.position.set(0, height * 0.9, 0);
      group.add(headMesh);
    } else {
      // Barrier / Debris Hazard block
      const barrierGeo = new THREE.BoxGeometry(length * 0.9, height * 0.7, width * 0.9);
      const barrierMat = new THREE.MeshStandardMaterial({ color: 0x475569, roughness: 0.9 });
      const barrierMesh = new THREE.Mesh(barrierGeo, barrierMat);
      barrierMesh.position.set(0, height * 0.38, 0);
      group.add(barrierMesh);
    }

    // 4. Ground Shadow / Radar Footprint
    const footGeo = new THREE.PlaneGeometry(length * 1.1, width * 1.1);
    footGeo.rotateX(-Math.PI / 2);
    const footMat = new THREE.MeshBasicMaterial({
      color: riskColor,
      transparent: true,
      opacity: 0.25,
      depthWrite: false
    });
    const footMesh = new THREE.Mesh(footGeo, footMat);
    footMesh.position.set(0, 0.02, 0);
    group.add(footMesh);

    // 5. Dynamic HUD Billboard Sprite Badge
    const canvas = document.createElement('canvas');
    canvas.width = 380 * 2;
    canvas.height = 180 * 2;
    const ctx = canvas.getContext('2d');
    updateObstacleCanvasBadge(obsData, canvas, ctx);

    const spriteTexture = new THREE.CanvasTexture(canvas);
    spriteTexture.minFilter = THREE.LinearFilter;
    const spriteMat = new THREE.SpriteMaterial({
      map: spriteTexture,
      transparent: true,
      depthTest: false
    });
    const labelSprite = new THREE.Sprite(spriteMat);
    labelSprite.scale.set(3.2, 1.5, 1);
    labelSprite.position.set(0, height + 1.25, 0);
    group.add(labelSprite);

    // Initial position
    const initPos = isoToThree(obsData.position?.x, obsData.position?.y, obsData.position?.z);
    group.position.copy(initPos);

    return {
      group,
      bboxMesh,
      wireframeMesh,
      footMesh,
      labelSprite,
      canvas,
      ctx,
      lastData: obsData,
      currentPos: initPos.clone(),
      targetPos: initPos.clone()
    };
  }

  /**
   * Synchronizes active 3D obstacles with incoming telemetry obstacles array.
   */
  function syncObstacles(obstaclesArray) {
    if (!obstaclesGroup) return;

    if (!layers.obstacles || !obstaclesArray || !Array.isArray(obstaclesArray)) {
      // Remove all if layer is toggled off
      if (!layers.obstacles) {
        activeObstacles.forEach(entry => obstaclesGroup.remove(entry.group));
        activeObstacles.clear();
      }
      return;
    }

    const currentIds = new Set();

    obstaclesArray.forEach(obsData => {
      const id = String(obsData.id || `obs_${Math.random().toString(36).substr(2, 6)}`);
      currentIds.add(id);

      let entry = activeObstacles.get(id);
      if (!entry) {
        entry = createObstacleObject(obsData);
        activeObstacles.set(id, entry);
        obstaclesGroup.add(entry.group);
      } else {
        // Update target position for smooth 60fps interpolation
        isoToThree(obsData.position?.x, obsData.position?.y, obsData.position?.z, entry.targetPos);

        // Update colors if risk level shifted
        if (entry.lastData.riskLevel !== obsData.riskLevel) {
          const newColor = getRiskColor(obsData.riskLevel);
          entry.wireframeMesh.material.color.setHex(newColor);
          entry.bboxMesh.material.color.setHex(newColor);
          entry.footMesh.material.color.setHex(newColor);
        }

        // Refresh dynamic canvas HUD text
        updateObstacleCanvasBadge(obsData, entry.canvas, entry.ctx);
        entry.labelSprite.material.map.needsUpdate = true;
        entry.lastData = obsData;
      }
    });

    // Remove obsolete obstacles that disappeared from telemetry
    activeObstacles.forEach((entry, id) => {
      if (!currentIds.has(id)) {
        obstaclesGroup.remove(entry.group);
        if (entry.bboxMesh.geometry) entry.bboxMesh.geometry.dispose();
        if (entry.wireframeMesh.geometry) entry.wireframeMesh.geometry.dispose();
        if (entry.labelSprite.material.map) entry.labelSprite.material.map.dispose();
        activeObstacles.delete(id);
      }
    });
  }

  // ==========================================================================
  // TRAJECTORY & NAVIGATION PATH
  // ==========================================================================
  function createTrajectoryPath() {
    const maxPoints = 50;
    const pathPositions = new Float32Array(maxPoints * 3);
    const pathGeo = new THREE.BufferGeometry();
    pathGeo.setAttribute('position', new THREE.BufferAttribute(pathPositions, 3));
    pathGeo.setDrawRange(0, 0);

    const pathMat = new THREE.LineBasicMaterial({
      color: 0x00f0ff,
      linewidth: 3,
      transparent: true,
      opacity: 0.75
    });

    trajectoryLine = new THREE.Line(pathGeo, pathMat);
    trajectoryLine.name = 'TrajectoryLine';
    scene.add(trajectoryLine);
  }

  function updateTrajectory(vehiclePos, headingRad, speedMps, steeringRad) {
    if (!trajectoryLine || !layers.trajectory) {
      if (trajectoryLine) trajectoryLine.geometry.setDrawRange(0, 0);
      return;
    }

    const posAttr = trajectoryLine.geometry.attributes.position;
    const array = posAttr.array;
    const steps = 30;
    const dt = 0.12; // 3.6 seconds lookahead
    let simX = vehiclePos.x;
    let simZ = vehiclePos.z;
    let simHeading = headingRad;

    for (let i = 0; i < steps; i++) {
      array[i * 3] = simX;
      array[i * 3 + 1] = 0.05; // Slightly above road
      array[i * 3 + 2] = simZ;

      // Kinematic bicycle model projection
      const dist = Math.max(speedMps, 5.0) * dt;
      simX += Math.cos(simHeading) * dist;
      simZ -= Math.sin(simHeading) * dist;
      simHeading += (dist / WHEELBASE) * Math.tan(steeringRad);
    }

    posAttr.needsUpdate = true;
    trajectoryLine.geometry.setDrawRange(0, steps);
  }

  // ==========================================================================
  // CAMERA TRACKING CONTROLLERS
  // ==========================================================================
  function setupOrbitControls() {
    const OrbitControlsClass = (typeof THREE !== 'undefined' && THREE.OrbitControls) ||
                               (typeof window !== 'undefined' && (window.OrbitControls || (window.THREE && window.THREE.OrbitControls))) ||
                               (typeof globalThis !== 'undefined' && (globalThis.OrbitControls || (globalThis.THREE && globalThis.THREE.OrbitControls)));
    if (typeof OrbitControlsClass === 'function') {
      orbitControls = new OrbitControlsClass(camera, renderer.domElement);
      orbitControls.enableDamping = true;
      orbitControls.dampingFactor = 0.08;
      orbitControls.maxPolarAngle = Math.PI / 2 - 0.04; // Never go below road plane
      orbitControls.minDistance = 3.0;
      orbitControls.maxDistance = 180.0;
      orbitControls.target.copy(currentVehicleState.position);
      orbitControls.enabled = (currentCameraMode === CAMERA_MODES.FREE_ORBIT);
      fallbackOrbitActive = false;
    } else {
      setupFallbackOrbitControls();
    }
  }

  function setupFallbackOrbitControls() {
    fallbackOrbitActive = true;
    fallbackOrbit.target = currentVehicleState.position;

    const dom = renderer ? renderer.domElement : null;
    if (!dom || typeof dom.addEventListener !== 'function') return;

    dom.addEventListener('mousedown', (e) => {
      if (currentCameraMode !== CAMERA_MODES.FREE_ORBIT) return;
      fallbackOrbit.isDragging = true;
      fallbackOrbit.previousMouseX = e.clientX;
      fallbackOrbit.previousMouseY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!fallbackOrbit.isDragging || currentCameraMode !== CAMERA_MODES.FREE_ORBIT) return;
      const dx = e.clientX - fallbackOrbit.previousMouseX;
      const dy = e.clientY - fallbackOrbit.previousMouseY;
      fallbackOrbit.previousMouseX = e.clientX;
      fallbackOrbit.previousMouseY = e.clientY;

      fallbackOrbit.azimuth -= dx * 0.008;
      fallbackOrbit.elevation = Math.max(0.08, Math.min(Math.PI / 2 - 0.05, fallbackOrbit.elevation + dy * 0.008));
    });

    window.addEventListener('mouseup', () => {
      fallbackOrbit.isDragging = false;
    });

    dom.addEventListener('wheel', (e) => {
      if (currentCameraMode !== CAMERA_MODES.FREE_ORBIT) return;
      fallbackOrbit.distance = Math.max(3.5, Math.min(160.0, fallbackOrbit.distance + e.deltaY * 0.02));
      e.preventDefault();
    }, { passive: false });
  }

  /**
   * Smoothly updates camera position and orientation based on active mode.
   */
  function updateCameraTracking(dt) {
    if (!vehicleGroup || !camera) return;

    const vehPos = currentVehicleState.position;
    const heading = currentVehicleState.headingRad;

    // Heading forward unit vector (+X cos, -Z sin)
    const fwdX = Math.cos(heading);
    const fwdZ = -Math.sin(heading);

    switch (currentCameraMode) {
      case CAMERA_MODES.CHASE: {
        // Third-person chase: 10.5m behind, 4.0m above, looking 8m ahead
        const targetCamX = vehPos.x - fwdX * 10.5;
        const targetCamY = vehPos.y + 4.2;
        const targetCamZ = vehPos.z - fwdZ * 10.5;

        const lookAtX = vehPos.x + fwdX * 8.0;
        const lookAtY = vehPos.y + 1.2;
        const lookAtZ = vehPos.z + fwdZ * 8.0;

        // Exponential smoothing (damping)
        const lerpFactor = Math.min(1.0, dt * 6.5);
        camera.position.x += (targetCamX - camera.position.x) * lerpFactor;
        camera.position.y += (targetCamY - camera.position.y) * lerpFactor;
        camera.position.z += (targetCamZ - camera.position.z) * lerpFactor;

        tmpVecA.set(lookAtX, lookAtY, lookAtZ);
        camera.lookAt(tmpVecA);
        break;
      }

      case CAMERA_MODES.TOP_DOWN: {
        // God / Tactical Bird's-Eye View: 48m directly above
        const targetCamX = vehPos.x;
        const targetCamY = vehPos.y + 48.0;
        const targetCamZ = vehPos.z;

        const lerpFactor = Math.min(1.0, dt * 8.0);
        camera.position.x += (targetCamX - camera.position.x) * lerpFactor;
        camera.position.y += (targetCamY - camera.position.y) * lerpFactor;
        camera.position.z += (targetCamZ - camera.position.z) * lerpFactor;

        // Look straight down at vehicle, aligning camera up with forward heading
        camera.up.set(fwdX, 0, fwdZ);
        tmpVecA.set(vehPos.x, vehPos.y, vehPos.z);
        camera.lookAt(tmpVecA);
        break;
      }

      case CAMERA_MODES.COCKPIT: {
        // First-person driver cockpit view
        camera.up.set(0, 1, 0);
        // Position inside cabin at driver's head level
        const eyeX = vehPos.x + fwdX * 0.45;
        const eyeY = vehPos.y + 1.18;
        const eyeZ = vehPos.z + fwdZ * 0.45;

        camera.position.set(eyeX, eyeY, eyeZ);

        const lookAtX = eyeX + fwdX * 35.0;
        const lookAtY = eyeY - 0.2;
        const lookAtZ = eyeZ + fwdZ * 35.0;

        tmpVecA.set(lookAtX, lookAtY, lookAtZ);
        camera.lookAt(tmpVecA);
        break;
      }

      case CAMERA_MODES.FREE_ORBIT: {
        camera.up.set(0, 1, 0);
        if (orbitControls && orbitControls.enabled) {
          // Smoothly track vehicle as orbit pivot
          orbitControls.target.lerp(vehPos, Math.min(1.0, dt * 5.0));
          orbitControls.update();
        } else if (fallbackOrbitActive) {
          const dist = fallbackOrbit.distance;
          const az = fallbackOrbit.azimuth;
          const el = fallbackOrbit.elevation;

          camera.position.x = vehPos.x + dist * Math.cos(el) * Math.sin(az);
          camera.position.y = vehPos.y + dist * Math.sin(el);
          camera.position.z = vehPos.z + dist * Math.cos(el) * Math.cos(az);
          camera.lookAt(vehPos.x, vehPos.y + 1.0, vehPos.z);
        }
        break;
      }
    }
  }

  // ==========================================================================
  // RENDER & ANIMATION LOOP (60 FPS)
  // ==========================================================================
  function animate() {
    animationFrameId = requestAnimationFrame(animate);

    if (!renderer || !scene || !camera) return;

    const dt = Math.min(clock.getDelta(), 0.1);

    // 1. Smooth Vehicle Kinematics Interpolation
    const posLerp = Math.min(1.0, dt * 14.0);
    currentVehicleState.position.lerp(targetVehicleState.position, posLerp);

    const headingDiff = normalizeAngleRad(targetVehicleState.headingRad - currentVehicleState.headingRad);
    currentVehicleState.headingRad += headingDiff * posLerp;

    currentVehicleState.speedMps += (targetVehicleState.speedMps - currentVehicleState.speedMps) * posLerp;
    currentVehicleState.steeringRad += (targetVehicleState.steeringRad - currentVehicleState.steeringRad) * posLerp;
    currentVehicleState.brakePct += (targetVehicleState.brakePct - currentVehicleState.brakePct) * posLerp;

    // Apply to vehicle 3D scene group
    vehicleGroup.position.copy(currentVehicleState.position);
    vehicleGroup.rotation.y = currentVehicleState.headingRad;

    // 2. Wheel Rotation & Steering Physics
    if (wheelFL && wheelFR && wheelRL && wheelRR) {
      const wheelRollDelta = (currentVehicleState.speedMps * dt) / WHEEL_RADIUS;
      wheelSpinAngle += wheelRollDelta;

      // Wheel spin around axle (local X or Z)
      wheelFL.rotation.z = wheelSpinAngle;
      wheelFR.rotation.z = wheelSpinAngle;
      wheelRL.rotation.z = wheelSpinAngle;
      wheelRR.rotation.z = wheelSpinAngle;

      // Front wheel steering deflection
      if (steerGroupFL && steerGroupFR) {
        steerGroupFL.rotation.y = currentVehicleState.steeringRad;
        steerGroupFR.rotation.y = currentVehicleState.steeringRad;
      }
    }

    // 3. Dynamic Brake Light Response
    if (tailLightMaterial && brakePointLight) {
      const isBraking = currentVehicleState.brakePct > 2.0 || targetVehicleState.acceleration < -0.4;
      if (isBraking) {
        tailLightMaterial.color.setHex(THEME.taillightBraking);
        tailLightMaterial.emissive.setHex(THEME.taillightBraking);
        tailLightMaterial.emissiveIntensity = 3.6;
        brakePointLight.intensity = 2.4;
      } else {
        tailLightMaterial.color.setHex(THEME.taillightNominal);
        tailLightMaterial.emissive.setHex(THEME.taillightNominal);
        tailLightMaterial.emissiveIntensity = 0.8;
        brakePointLight.intensity = 0.0;
      }
    }

    // 4. Rotating LiDAR Sensor Puck & Radar Fan
    if (lidarPuckMesh) {
      lidarPuckMesh.rotation.y += dt * 24.0; // Fast realistic spin (~600 RPM)
    }
    if (lidarScanFan) {
      lidarScanFan.rotation.y += dt * 8.0;
    }

    // 5. Endless Ground/Road Tracking
    if (roadMesh && vehicleGroup) {
      // Keep road and grid aligned near the vehicle along X
      const roadTileSize = 100;
      roadMesh.position.x = Math.floor(currentVehicleState.position.x / roadTileSize) * roadTileSize;
      if (gridHelper) {
        gridHelper.position.x = Math.floor(currentVehicleState.position.x / 50) * 50;
      }
    }

    // 6. Smooth Obstacle Position Interpolation
    activeObstacles.forEach(entry => {
      entry.currentPos.lerp(entry.targetPos, Math.min(1.0, dt * 12.0));
      entry.group.position.copy(entry.currentPos);
    });

    // 7. Update Trajectory Ribbon
    updateTrajectory(
      currentVehicleState.position,
      currentVehicleState.headingRad,
      currentVehicleState.speedMps,
      currentVehicleState.steeringRad
    );

    // 8. Camera Tracking
    updateCameraTracking(dt);

    // 9. WebGL Render Execution
    renderer.render(scene, camera);
  }

  // ==========================================================================
  // PUBLIC EXPORTED API
  // ==========================================================================
  const GarudaVisualization = {

    /**
     * Initializes the Three.js WebGL rendering engine inside containerElement.
     * @param {HTMLElement|string} containerEl 
     */
    init(containerEl) {
      if (typeof containerEl === 'string') {
        container = document.querySelector(containerEl);
      } else {
        container = containerEl;
      }

      if (!container) {
        console.error('[GarudaVisualization] Container element not found.');
        return;
      }

      if (typeof THREE === 'undefined') {
        console.error('[GarudaVisualization] Three.js library (THREE) is not loaded.');
        return;
      }

      // Reusable Vector Instances
      tmpVecA = new THREE.Vector3();
      tmpVecB = new THREE.Vector3();
      tmpVecC = new THREE.Vector3();
      tmpQuat = new THREE.Quaternion();
      tmpEuler = new THREE.Euler();
      currentVehicleState.position = new THREE.Vector3(0, 0, 0);
      targetVehicleState.position = new THREE.Vector3(0, 0, 0);

      // 1. Scene Construction
      scene = new THREE.Scene();
      scene.background = new THREE.Color(THEME.bgDark);
      scene.fog = new THREE.FogExp2(THEME.bgDark, 0.0065);

      // 2. Camera Setup
      const width = container.clientWidth || 800;
      const height = container.clientHeight || 500;
      camera = new THREE.PerspectiveCamera(58, width / height, 0.1, 1000);
      camera.position.set(-12, 6, 0);

      // 3. WebGLRenderer Setup
      renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: false,
        powerPreference: 'high-performance'
      });
      renderer.setSize(width, height);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      if (renderer.toneMapping !== undefined) {
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.05;
      }

      // Empty container before attaching canvas
      while (container.firstChild) {
        container.removeChild(container.firstChild);
      }
      container.appendChild(renderer.domElement);

      // 4. Lighting System
      const ambientLight = new THREE.AmbientLight(0x1a2638, 1.2);
      scene.add(ambientLight);

      const sunLight = new THREE.DirectionalLight(0xdde8ff, 1.4);
      sunLight.position.set(40, 80, 30);
      sunLight.castShadow = true;
      sunLight.shadow.mapSize.width = 2048;
      sunLight.shadow.mapSize.height = 2048;
      sunLight.shadow.camera.near = 10;
      sunLight.shadow.camera.far = 180;
      sunLight.shadow.camera.left = -30;
      sunLight.shadow.camera.right = 30;
      sunLight.shadow.camera.top = 30;
      sunLight.shadow.camera.bottom = -30;
      scene.add(sunLight);

      // Subtle Cyber Under-light (Horizon Blue)
      const hemiLight = new THREE.HemisphereLight(0x1e3a8a, 0x080c14, 0.6);
      scene.add(hemiLight);

      // 5. Component Construction
      createEnvironment();
      scene.add(environmentGroup);

      createEgoVehicle();
      scene.add(vehicleGroup);

      obstaclesGroup = new THREE.Group();
      obstaclesGroup.name = 'ObstaclesGroup';
      scene.add(obstaclesGroup);

      createLidarPointCloud();
      createTrajectoryPath();

      // 6. Camera Controls Setup
      clock = new THREE.Clock();
      setupOrbitControls();

      // 7. Resize Observer for dynamic flexbox / panel changes
      if (window.ResizeObserver) {
        resizeObserver = new ResizeObserver(() => {
          this.resize();
        });
        resizeObserver.observe(container);
      }
      window.addEventListener('resize', () => this.resize());

      // 8. Start 60 FPS Render Loop
      if (animationFrameId) cancelAnimationFrame(animationFrameId);
      animate();

      console.log('[GarudaVisualization] Initialized successfully. 3D Digital Twin ready.');
    },

    /**
     * Updates scene state from real-time telemetry packet.
     * @param {Object} telemetry 
     */
    updateTelemetry(telemetry) {
      if (!telemetry || !vehicleGroup) return;

      // 1. Vehicle Localization & Orientation
      const v = telemetry.vehicle;
      if (v) {
        if (v.position) {
          isoToThree(v.position.x, v.position.y, v.position.z, targetVehicleState.position);
        }

        if (v.orientation) {
          if (typeof v.orientation.headingRad === 'number') {
            targetVehicleState.headingRad = v.orientation.headingRad;
          } else if (typeof v.orientation.headingDeg === 'number') {
            targetVehicleState.headingRad = (v.orientation.headingDeg * Math.PI) / 180.0;
          } else if (typeof v.orientation.yaw === 'number') {
            targetVehicleState.headingRad = (v.orientation.yaw * Math.PI) / 180.0;
          }
        }

        if (v.motion) {
          targetVehicleState.speedMps = Number(v.motion.speedMps) || 0;
          if (typeof v.motion.steeringAngleDeg === 'number') {
            targetVehicleState.steeringRad = (v.motion.steeringAngleDeg * Math.PI) / 180.0;
          }
          targetVehicleState.brakePct = Number(v.motion.brakePct) || 0;
          targetVehicleState.acceleration = Number(v.motion.acceleration) || 0;
        }
      }

      // 2. LiDAR Point Cloud Update
      if (telemetry.lidar && telemetry.lidar.points) {
        updateLidarBuffer(telemetry.lidar.points);
      }

      // 3. Obstacle Perception Synchronization
      if (telemetry.obstacles) {
        syncObstacles(telemetry.obstacles);
      }
    },

    /**
     * Switches viewport camera mode.
     * @param {string} mode - "CHASE" | "TOP_DOWN" | "COCKPIT" | "FREE_ORBIT"
     */
    setCameraMode(mode) {
      const normalized = String(mode).toUpperCase().trim();
      let selected = CAMERA_MODES.CHASE;

      if (normalized === 'GOD' || normalized === 'TOP_DOWN' || normalized === 'TOPDOWN') {
        selected = CAMERA_MODES.TOP_DOWN;
      } else if (normalized === 'COCKPIT' || normalized === 'FIRST_PERSON') {
        selected = CAMERA_MODES.COCKPIT;
      } else if (normalized === 'ORBIT' || normalized === 'FREE_ORBIT') {
        selected = CAMERA_MODES.FREE_ORBIT;
      } else {
        selected = CAMERA_MODES.CHASE;
      }

      currentCameraMode = selected;

      // Handle OrbitControls enable/disable
      if (orbitControls) {
        orbitControls.enabled = (currentCameraMode === CAMERA_MODES.FREE_ORBIT);
        if (orbitControls.enabled && currentVehicleState.position) {
          orbitControls.target.copy(currentVehicleState.position);
          camera.position.set(
            currentVehicleState.position.x - 14,
            currentVehicleState.position.y + 8,
            currentVehicleState.position.z + 10
          );
        }
      }

      // Reset camera up vector if leaving top-down
      if (currentCameraMode !== CAMERA_MODES.TOP_DOWN && camera) {
        camera.up.set(0, 1, 0);
      }

      console.log(`[GarudaVisualization] Camera mode set to: ${currentCameraMode}`);
    },

    /**
     * Returns the active camera mode string.
     */
    getCameraMode() {
      return currentCameraMode;
    },

    /**
     * Toggles LiDAR point cloud layer visibility.
     * @param {boolean} visible 
     */
    toggleLidar(visible) {
      layers.lidar = !!visible;
      if (lidarPointCloud) lidarPointCloud.visible = layers.lidar;
      if (lidarScanFan) lidarScanFan.visible = layers.lidar;
      if (!layers.lidar && lidarPointsGeometry) {
        lidarPointsGeometry.setDrawRange(0, 0);
      }
    },

    /**
     * Toggles 3D obstacle bounding boxes & HUD badges visibility.
     * @param {boolean} visible 
     */
    toggleObstacles(visible) {
      layers.obstacles = !!visible;
      if (obstaclesGroup) obstaclesGroup.visible = layers.obstacles;
    },

    /**
     * Toggles ground reference grid visibility.
     * @param {boolean} visible 
     */
    toggleGrid(visible) {
      layers.grid = !!visible;
      if (gridHelper) gridHelper.visible = layers.grid;
    },

    /**
     * Toggles predicted trajectory ribbon visibility.
     * @param {boolean} visible 
     */
    toggleTrajectory(visible) {
      layers.trajectory = !!visible;
      if (trajectoryLine) trajectoryLine.visible = layers.trajectory;
    },

    /**
     * Recalculates canvas dimensions and camera projection matrix.
     */
    resize() {
      if (!container || !renderer || !camera) return;
      const width = container.clientWidth;
      const height = container.clientHeight;
      if (width === 0 || height === 0) return;

      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    },

    /**
     * Cleans up WebGL resources and stops animation loop.
     */
    dispose() {
      if (animationFrameId) cancelAnimationFrame(animationFrameId);
      if (resizeObserver) resizeObserver.disconnect();
      if (renderer && renderer.domElement && renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
      console.log('[GarudaVisualization] Disposed.');
    }
  };

  return GarudaVisualization;
}));
