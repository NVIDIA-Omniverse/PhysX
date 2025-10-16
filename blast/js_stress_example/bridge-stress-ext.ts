// Ensure a global harness object exists as early as possible
const __existingHarness = (typeof globalThis !== 'undefined') ? (globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {})) : {};
if (__existingHarness) {
  __existingHarness.scriptLoaded = true;
}

/**
 * Rapier + Three.js + Blast Ext Stress Solver (destructible bridge)
 * -----------------------------------------------------------------
 * Browser visualization that mirrors the sequencing used by the headless tests:
 *   - Safe frames (after splits): world.step() [no events] → applyPendingMigrations/remove → render
 *   - Eventful frames: sweepBeforeEventfulStep → world.step(eventQueue) → drain → solver → debug → render
 * Heavy lifting (solver, queuing, migrations, drains) lives in the shared modules.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Stats from 'three/addons/libs/stats.module.js';
import RAPIER from '@dimforge/rapier3d-compat';
// import RAPIER from '@dimforge/rapier3d-debug';
import RapierDebugRenderer from './rapier-debug-renderer.js';

import {
  loadStressSolver,
  formatNumber,
  ExtDebugMode
} from './stress.js';

import {
  applyForcesAndSolve as coreApplyForcesAndSolve,
  processSolverFractures as coreProcessSolverFractures,
  collectContactForceEvents,
  accumulateContactForcesFromEvents,
  sweepBeforeEventfulStep
} from './bridge/coreLogic.js';

import { applyPendingMigrations, pruneStaleHandles, enqueueSplitResults } from './bridge/rapierHierarchyApplier.js';

import {
  spawnLoadVehicle as spawnCar,
  updateLoadVehicle as updateCar,
  spawnProjectile as spawnProj,
  updateProjectiles as updateProjList
} from './bridge/dynamics.js';

import { buildBridgeScenario } from './extBridgeScenario.js';
import { updateBondTable } from './bridge/ui.js';
import { createBridgeCore } from './bridge/buildBridge.headless.js';

// --------------------------- Constants & UI ---------------------------

const GRAVITY_DEFAULT = -9.81;
const CONTACT_FORCE_THRESHOLD = 0.0;

const CAR_PROPS = {
  mass: 5200,
  length: 3.6,
  width: 1.8,
  height: 1.2,
  speed: 4.5,
  idleDuration: 0.75
};

const BASE_LIMITS = {
  compressionElasticLimit: 0.001,
  compressionFatalLimit: 0.002,
  tensionElasticLimit: 0.001,
  tensionFatalLimit: 0.002,
  shearElasticLimit: 0.001,
  shearFatalLimit: 0.002
};

const HUD = {
  gravityValue: document.getElementById('gravity-value'),
  strengthValue: document.getElementById('strength-value'),
  iterValue: document.getElementById('iter-value'),
  toleranceValue: document.getElementById('tolerance-value'),
  bondTable: document.getElementById('bond-table'),
  eventLog: document.getElementById('event-log'),
  overlay: document.getElementById('overlay')
};

const controlsUI = {
  gravitySlider: document.getElementById('gravity-slider'),
  strengthSlider: document.getElementById('strength-slider'),
  fireButton: document.getElementById('fire-projectile'),
  resetButton: document.getElementById('reset-bridge'),
  debugToggle: document.getElementById('toggle-debug'),
  iterSlider: document.getElementById('iter-slider'),
  toleranceSlider: document.getElementById('tolerance-slider')
};

const STRESS_COLOR_LOW = new THREE.Color('#3fb3ff');
const STRESS_COLOR_MEDIUM = new THREE.Color('#ff9f43');
const STRESS_COLOR_HIGH = new THREE.Color('#DC143C');
const DETACHED_COLOR = new THREE.Color('#3f3f3f');

const stressColorScratch = new THREE.Color();

const SEVERITY_EPSILON = 1.0e-6;
const COLOR_LOW_R = STRESS_COLOR_LOW.r;
const COLOR_HIGH_R = STRESS_COLOR_HIGH.r;
const COLOR_LOW_G = STRESS_COLOR_LOW.g;
const COLOR_HIGH_G = STRESS_COLOR_HIGH.g;
const COLOR_RANGE_R = Math.max(COLOR_HIGH_R - COLOR_LOW_R, 1.0e-6);
const COLOR_RANGE_G = Math.max(COLOR_LOW_G - COLOR_HIGH_G, 1.0e-6);

// Stress legend thresholds (UI only)
const STRESS_SEVERITY_MEDIUM_THRESHOLD = 0.5;
const STRESS_SEVERITY_HIGH_THRESHOLD = STRESS_SEVERITY_MEDIUM_THRESHOLD;

// --------------------------- Helpers ---------------------------

function colorFromUint24(u24) {
  const r = ((u24 >> 16) & 0xff) / 255;
  const g = ((u24 >> 8) & 0xff) / 255;
  const b = (u24 & 0xff) / 255;
  return { r, g, b };
}

function clamp01(v) {
  if (!Number.isFinite(v)) return 0;
  return Math.min(Math.max(v, 0), 1);
}

function stressSeverityFromLine(line) {
  if (!line) return 0;
  const c0 = colorFromUint24(line.color0 ?? 0);
  const c1 = colorFromUint24(line.color1 ?? line.color0 ?? 0);
  const red = Math.max(c0.r, c1.r);
  const green = Math.min(c0.g, c1.g);
  const redSeverity = COLOR_RANGE_R > 0 ? (red - COLOR_LOW_R) / COLOR_RANGE_R : 0;
  const greenSeverity = COLOR_RANGE_G > 0 ? (COLOR_LOW_G - green) / COLOR_RANGE_G : 0;
  return clamp01(Math.max(redSeverity, greenSeverity, 0));
}

function formatTolerance(value) {
  return Number(value).toExponential(2);
}

// --------------------------- Main Entry ---------------------------

async function init() {
  const harness = globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {});
  harness.tickCount = 0;
  harness.lastError = null;
  harness.ready = false;
  harness.url = globalThis.location?.href ?? harness.url ?? '';
  harness.console = [];

  const logHook = (level, ...args) => {
    try {
      harness.console.push({ level, message: args.map((a) => String(a ?? '')).join(' '), time: Date.now() });
      if (harness.console.length > 200) harness.console.splice(0, harness.console.length - 200);
    } catch {}
  };
  console.log = ((orig) => (...args) => { logHook('log', ...args); orig(...args); })(console.log.bind(console));
  console.warn = ((orig) => (...args) => { logHook('warn', ...args); orig(...args); })(console.warn.bind(console));
  console.error = ((orig) => (...args) => { logHook('error', ...args); orig(...args); })(console.error.bind(console));

  await RAPIER.init();
  const runtime = await loadStressSolver();

  const { scene, renderer, camera, controls } = initThree();
  const stats = new Stats();
  try { document.body.appendChild(stats.dom); } catch {}

  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));

  const bridge = buildBridge(scene, world, runtime);
  setupControls(world, bridge);

  // Ensure we have an event queue for eventful frames
  try {
    if (!bridge.eventQueue) bridge.eventQueue = new RAPIER.EventQueue(true);
  } catch {}

  const clock = new THREE.Clock();

  function loop() {
    const delta = clock.getDelta();
    const worldNow = bridge.world;

    const loopHarness = globalThis.__bridgeExt;
    if (loopHarness) {
      loopHarness.tickCount = (loopHarness.tickCount ?? 0) + 1;
      loopHarness.overstressed = bridge.overstressed ?? 0;
      loopHarness.actorCount = bridge.solver?.actorCount?.() ?? bridge.solver?.actors?.()?.length ?? null;
    }

    const isSafeFrame = !!(bridge.safeFrames && bridge.safeFrames > 0);

    if (isSafeFrame) {
      // -------- SAFE STEP (no events): migration/removal only, no drains --------
      try {
        worldNow.step();
      } catch (err) {
        try { const h = globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {}); h.lastError = h.lastError ?? (err?.stack ?? String(err)); } catch {}
      }

      bridge.safeFrames = Math.max((bridge.safeFrames ?? 1) - 1, 0);

      // Process pending splits from stress solver → migrations only in safe frames
      if (bridge.pendingSplitResults?.length > 0) {
        const splits = bridge.pendingSplitResults;
        bridge.pendingSplitResults = [];
        try { enqueueSplitResults(bridge, splits); } catch {}
      }

      // try { applyPendingMigrations(bridge); } catch {}
      try { removeDisabledHandles(bridge); } catch {}

      // For safety with Rapier aliasing, skip mesh sync in safe frames
      // Meshes will be synced on the next eventful frame
      if (bridge.debugRenderer?.enabled) bridge.debugRenderer.update();

      controls.update();
      renderer.render(scene, camera);
      stats.update();
      requestAnimationFrame(loop);
      return;
    }

    // -------- EVENTFUL STEP (with events): mirror test sequencing exactly --------
    // 0) Pre-step sweep (drop stale handles & remove disabled colliders queued earlier)
    try { sweepBeforeEventfulStep(bridge); } catch {}

    // 1) Eventful step with our queue
    let stepped = false;
    try {
      if (!bridge.eventQueue) bridge.eventQueue = new RAPIER.EventQueue(true);
      worldNow.step(bridge.eventQueue);
      stepped = true;
    } catch (err) {
      // Reset queue and schedule a safe frame
      try { bridge.eventQueue = new RAPIER.EventQueue(true); } catch {}
      bridge.safeFrames = Math.max(bridge.safeFrames ?? 0, 1);
      try { const h = globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {}); h.lastError = h.lastError ?? (err?.stack ?? String(err)); } catch {}
    }
    if (!stepped) {
      // Skip mesh sync during safe frames to avoid aliasing
      if (bridge.debugRenderer?.enabled) bridge.debugRenderer.update();
      controls.update();
      renderer.render(scene, camera);
      stats.update();
      requestAnimationFrame(loop);
      return;
    }

    // 2) Post-step drains (two-phase) + sync
    pruneStaleHandles(bridge);
    const collected = collectContactForceEvents(bridge.eventQueue);
    accumulateContactForcesFromEvents(bridge, collected);

    try {
      syncMeshes(worldNow, bridge);
    } catch (e) {
      try { bridge.safeFrames = Math.max(bridge.safeFrames ?? 0, 1); } catch {}
    }
    if (bridge.debugRenderer?.enabled) {
      try { bridge.debugRenderer.update(); } catch {}
    }

    // 3) Solver pass
    updateBridgeLogical(bridge, delta);
    if (bridge.pendingSplitResults && bridge.pendingSplitResults.length > 0) {
      bridge.safeFrames = Math.max(bridge.safeFrames ?? 0, 2);
    }

    controls.update();
    renderer.render(scene, camera);
    stats.update();

    requestAnimationFrame(loop);
  }

  harness.ready = true;
  loop();
}

init().catch((err) => {
  console.error('Failed to initialize bridge demo', err);
  pushEvent(`Initialization failed: ${err?.message ?? err}`);
  const harness = globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {});
  harness.lastError = err?.stack ?? err?.message ?? String(err ?? 'Unknown error');
});

// --------------------------- Scene / Build ---------------------------

function initThree() {
  const canvas = document.getElementById('bridge-canvas');
  let renderer = null;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
  } catch (err) {
    const h = globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {});
    h.lastError = h.lastError ?? `Renderer init failed: ${err?.message ?? err}`;
    // Fallback no-op renderer for CI/headless
    renderer = {
      setPixelRatio: () => {},
      setSize: () => {},
      shadowMap: { enabled: false },
      render: () => {},
      domElement: canvas
    };
  }

  const initialWidth = canvas.clientWidth || window.innerWidth;
  const initialHeight = canvas.clientHeight || window.innerHeight;
  renderer.setSize(initialWidth, initialHeight, false);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x05070a);

  const camera = new THREE.PerspectiveCamera(60, initialWidth / initialHeight, 0.1, 500);
  camera.position.set(0, 8, 18);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 0, 0);

  const ambient = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.85);
  dir.position.set(12, 18, 10);
  dir.castShadow = true;
  dir.shadow.mapSize.set(2048, 2048);
  scene.add(dir);

  const ground = new THREE.Mesh(
    new THREE.BoxGeometry(40, 1, 40),
    new THREE.MeshStandardMaterial({ color: 0x1a1e2f, roughness: 0.8, metalness: 0.1 })
  );
  ground.position.set(0, -4, 0);
  ground.receiveShadow = true;
  scene.add(ground);

  try {
    window.addEventListener('resize', () => {
      const width = canvas.clientWidth || window.innerWidth;
      const height = window.innerHeight - 1;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    });
  } catch {}

  return { scene, renderer, camera, controls };
}

function buildBridge(scene, world, runtime) {
  // Shared scenario and strength scale (must match headless)
  const scenario = buildBridgeScenario();
  const strengthScale = 0.03;

  // Ensure world exposes Rapier for shared dynamics
  world.RAPIER = RAPIER;

  // Build core physics/solver identically to headless path
  const core = createBridgeCore({ runtime, world, scenario, gravity: GRAVITY_DEFAULT, strengthScale });

  // Materials for visuals
  const deckMaterial = new THREE.MeshStandardMaterial({ color: 0x486fe3, roughness: 0.35, metalness: 0.45 });
  const topMaterial = new THREE.MeshStandardMaterial({ color: 0x5b86ff, roughness: 0.28, metalness: 0.55 });
  const supportMaterial = new THREE.MeshStandardMaterial({ color: 0x2f3e56, roughness: 0.6, metalness: 0.25 });

  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: true });
  const debugHelper = createDebugLineHelper();
  scene.add(debugHelper.object);

  // Attach meshes to core chunks (no new colliders/bodies created here!)
  const meshByNode = new Map();
  const supportChunks = [];
  core.chunks.forEach((chunk) => {
    const coord = scenario.gridCoordinates[chunk.nodeIndex];
    const mat = chunk.isSupport
      ? supportMaterial.clone()
      : ((coord && coord.iy === scenario.thicknessLayers - 1) ? topMaterial : deckMaterial);

    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z),
      mat
    );
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    chunk.mesh = mesh;

    chunk.baseWorldPosition = new THREE.Vector3(
      chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z
    );
    chunk.baseLocalOffset = new THREE.Vector3(
      chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z
    );
    chunk.localOffset = chunk.baseLocalOffset.clone();
    chunk.baseColor = mesh.material.color.clone();
    scene.add(mesh);
    meshByNode.set(chunk.nodeIndex, mesh);
    if (chunk.isSupport) supportChunks.push(chunk);
  });

  // Bonds for UI
  const bonds = scenario.bonds.map((b, i) => ({
    index: i,
    node0: b.node0,
    node1: b.node1,
    centroid: b.centroid,
    active: true,
    key: `${Math.min(b.node0, b.node1)}-${Math.max(b.node0, b.node1)}`,
    severity: 0
  }));

  // Rapier ground
  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setTranslation(0, -4, 0).setFriction(1.0).setRestitution(0.0),
    groundBody
  );

  // Load vehicle (Rapier) and projectiles list
  const car = spawnCar(world, { bodyDescFactory: () => RAPIER.RigidBodyDesc.dynamic(), scene, THREE });
  const projectiles = [];

  // Overlays
  pushEvent(`Bridge scenario loaded with ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`);
  updateBondTable(bonds);

  const topLayerNodeIndices = scenario.topColumnNodes.reduce((acc, column) => {
    if (!Array.isArray(column)) return acc;
    column.forEach((nodeIndex) => { if (Number.isInteger(nodeIndex)) acc.push(nodeIndex); });
    return acc;
  }, []);
  const topLayerNodePositions = topLayerNodeIndices.map((nodeIndex) => {
    const node = scenario.nodes[nodeIndex];
    if (!node?.centroid) return null;
    return new THREE.Vector3(node.centroid.x, node.centroid.y, node.centroid.z);
  });

  // Compose bridge state (physics from core + browser visuals)
  return {
    runtime,
    world,
    scene,
    solverNodes: core.solverNodes ?? scenario.nodes, // harmless
    settings: core.settings,
    solver: core.solver,
    scenario,
    chunks: core.chunks,
    bonds,
    meshByNode,
    bodyHandle: core.bodyHandle,
    car,
    projectiles,
    eventQueue: core.eventQueue,
    supports: supportChunks,
    gravity: GRAVITY_DEFAULT,
    strengthScale,
    targetError: 1.0e-6,
    iterationCount: 0,
    lastError: { lin: 0, ang: 0 },
    overstressed: 0,
    failureDetected: false,
    forceBoost: 1.0,
    debugEnabled: false,
    debugHelper,
    debugRenderer,
    bondCentroids: scenario.bonds.map((b) => new THREE.Vector3(b.centroid.x, b.centroid.y, b.centroid.z)),
    topLayerNodeIndices,
    topLayerNodePositions,
    colliderToNode: core.colliderToNode,
    projectileColliderHandles: new Set(),
    activeContactColliders: core.activeContactColliders,
    pendingContactForces: core.pendingContactForces,
    contactForceScratch: [],
    actorMap: core.actorMap,
    safeFrames: 10, // let BVH settle at startup
    // Two-phase migration state
    pendingSplitResults: [],
    pendingBodiesToCreate: [],
    pendingColliderMigrations: [],
    disabledCollidersToRemove: new Set<number>(),
    bodiesToRemove: new Set(),
    _handleSplitEventsCount: 0,
    _splitLog: []
  };
}

// --------------------------- Solver Update / Destruction ---------------------------

function updateBridgeLogical(bridge, delta) {
  (void delta);
  coreApplyForcesAndSolve(bridge);

  if (bridge.overstressed > 0) {
    const result = coreProcessSolverFractures(bridge, RAPIER, CONTACT_FORCE_THRESHOLD);
    if (result?.splitResults) {
      bridge.pendingSplitResults.push(...result.splitResults);
      bridge._justSplitFrames = Math.max(bridge._justSplitFrames ?? 0, 3);
    }
  }

  if (bridge.forceBoost > 1.0) {
    bridge.forceBoost = THREE.MathUtils.lerp(bridge.forceBoost, 1.0, Math.min(delta * 0.75, 1.0));
  }
}

function removeDisabledHandles(bridge) {
  const { world, disabledCollidersToRemove, bodiesToRemove } = bridge;
  
  if (disabledCollidersToRemove?.size > 0) {
    for (const h of Array.from(disabledCollidersToRemove)) {
      const c = world.getCollider(h);
      if (c) world.removeCollider(c, false);
      disabledCollidersToRemove.delete(h);
    }
  }
  
  if (bodiesToRemove?.size > 0) {
    for (const bh of Array.from(bodiesToRemove)) {
      const b = world.getRigidBody(bh);
      if (b) world.removeRigidBody(b);
      bodiesToRemove.delete(bh);
    }
  }
}

// --------------------------- Rendering / Sync ---------------------------

function syncMeshes(world, bridge) {
  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) return;

    let body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      const root = bridge.bodyHandle ? world.getRigidBody(bridge.bodyHandle) : null;
      if (root) {
        chunk.bodyHandle = bridge.bodyHandle;
        body = root;
      } else {
        return;
      }
    }

    const p = body.translation();
    const r = body.rotation();

    chunk.mesh.position.set(p.x, p.y, p.z);
    quat.set(r.x, r.y, r.z, r.w);
    chunk.mesh.quaternion.copy(quat);

    // apply local offset (bridge-local -> world)
    tmp.copy(chunk.localOffset).applyQuaternion(chunk.mesh.quaternion);
    chunk.mesh.position.add(tmp);

    const stressValue = bridge.chunkStress?.get(chunk.nodeIndex) ?? 0;
    const color = chunkColorForSeverity(chunk, stressValue);
    if (chunk.mesh.material.color.r !== color.r || chunk.mesh.material.color.g !== color.g || chunk.mesh.material.color.b !== color.b) {
      chunk.mesh.material.color.copy(color);
    }
  });

  // Sync car mesh to its body
  const car = bridge.car;
  if (car && car.mesh) {
    const b = world.getRigidBody(car.bodyHandle);
    if (b) {
      const t = b.translation();
      const q = b.rotation();
      car.mesh.position.set(t.x, t.y, t.z);
      car.mesh.quaternion.set(q.x, q.y, q.z, q.w);
    }
  }
}

function renderDebugLines(bridge) {
  const scale = solverSeverityScale(bridge);
  const debugLines = bridge.solver?.fillDebugRender({ mode: ExtDebugMode.Max, scale }) ?? [];
  bridge.debugHelper.currentLines = debugLines;
  processDebugLinesForSeverity(bridge, debugLines);

  if (!bridge.debugEnabled) {
    updateDebugLineObject(bridge.debugHelper, [], false);
    return;
  }

  const body = bridge.world.getRigidBody(bridge.bodyHandle);
  if (!body) {
    updateDebugLineObject(bridge.debugHelper, [], false);
    return;
  }

  const tr = body.translation();
  const rot = body.rotation();
  const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
  const t = new THREE.Vector3(tr.x, tr.y, tr.z);

  const worldLines = debugLines.map((line) => {
    const p0 = new THREE.Vector3(line.p0.x, line.p0.y, line.p0.z).applyQuaternion(q).add(t);
    const p1 = new THREE.Vector3(line.p1.x, line.p1.y, line.p1.z).applyQuaternion(q).add(t);
    return { p0: { x: p0.x, y: p0.y, z: p0.z }, p1: { x: p1.x, y: p1.y, z: p1.z }, color0: line.color0, color1: line.color1 };
  });

  updateDebugLineObject(bridge.debugHelper, worldLines, true);
}

// Debug helper (line object)
function createDebugLineHelper() {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));

  const material = new THREE.LineBasicMaterial({
    vertexColors: true,
    transparent: true,
    opacity: 0.95,
    depthTest: false
  });

  const object = new THREE.LineSegments(geometry, material);
  object.visible = false;

  return { object, geometry, material, currentLines: [] };
}

function updateDebugLineObject(debugHelper, debugLines, enabled) {
  if (!enabled || !debugLines || debugLines.length === 0) {
    debugHelper.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    debugHelper.geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
    debugHelper.object.visible = false;
    return;
  }

  const positions = new Float32Array(debugLines.length * 2 * 3);
  const colors = new Float32Array(debugLines.length * 2 * 3);

  debugLines.forEach((line, index) => {
    const base = index * 6;
    positions[base] = line.p0.x;
    positions[base + 1] = line.p0.y;
    positions[base + 2] = line.p0.z;
    positions[base + 3] = line.p1.x;
    positions[base + 4] = line.p1.y;
    positions[base + 5] = line.p1.z;

    const c0 = colorFromUint24(line.color0);
    const c1 = colorFromUint24(line.color1 ?? line.color0);
    colors[base] = c0.r; colors[base + 1] = c0.g; colors[base + 2] = c0.b;
    colors[base + 3] = c1.r; colors[base + 4] = c1.g; colors[base + 5] = c1.b;
  });

  debugHelper.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  debugHelper.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  debugHelper.geometry.computeBoundingSphere();
  debugHelper.object.visible = true;
}

function chunkColorForSeverity(chunk, severity) {
  if (severity <= 0) return chunk.baseColor;
  // Blend from baseColor to STRESS_COLOR_MEDIUM as severity goes from 0 to MEDIUM_THRESHOLD
  const lerpMedium = Math.min(severity, STRESS_SEVERITY_MEDIUM_THRESHOLD) / STRESS_SEVERITY_MEDIUM_THRESHOLD;
  stressColorScratch.copy(chunk.baseColor).lerp(STRESS_COLOR_MEDIUM, lerpMedium);

  // If severity exceeds HIGH_THRESHOLD, blend further toward STRESS_COLOR_HIGH
  if (severity >= STRESS_SEVERITY_HIGH_THRESHOLD) {
    const lerpHigh = Math.min(severity - STRESS_SEVERITY_HIGH_THRESHOLD, 1.0);
    stressColorScratch.lerp(STRESS_COLOR_HIGH, lerpHigh);
  }
  return stressColorScratch;
}

function processDebugLinesForSeverity(bridge, debugLines) {
  const chunkStress = bridge.chunkStress ?? new Map();
  chunkStress.clear();

  const { bonds, bondCentroids, chunks } = bridge;

  // Reset existing severities before we accumulate fresh values
  bonds.forEach((bond) => { if (bond) bond.severity = 0; });

  if (!Array.isArray(debugLines) || debugLines.length === 0) {
    bridge.chunkStress = chunkStress;
    chunks.forEach((chunk) => { chunk.stressSeverity = 0; });
    return;
  }

  debugLines.forEach((line) => {
    const bondIndex = nearestBondIndexFast(bondCentroids, line);
    if (bondIndex < 0) return;

    const bond = bonds[bondIndex];
    if (!bond || !bond.active) return;

    const severity = stressSeverityFromLine(line);
    if (!(severity > 0)) return;

    bond.severity = Math.max(bond.severity ?? 0, severity);
  });

  const severitySumByNode = new Map();
  const severityCountByNode = new Map();

  bonds.forEach((bond) => {
    if (!bond || !bond.active) return;
    const severity = bond.severity ?? 0;
    severitySumByNode.set(bond.node0, (severitySumByNode.get(bond.node0) ?? 0) + severity);
    severitySumByNode.set(bond.node1, (severitySumByNode.get(bond.node1) ?? 0) + severity);
    severityCountByNode.set(bond.node0, (severityCountByNode.get(bond.node0) ?? 0) + 1);
    severityCountByNode.set(bond.node1, (severityCountByNode.get(bond.node1) ?? 0) + 1);
  });

  bridge.chunks.forEach((chunk) => {
    const sum = severitySumByNode.get(chunk.nodeIndex) ?? 0;
    const count = severityCountByNode.get(chunk.nodeIndex) ?? 0;
    const avgSeverity = count > 0 ? sum / Math.max(count, 1) : 0;
    chunk.stressSeverity = avgSeverity;
    chunkStress.set(chunk.nodeIndex, chunk.stressSeverity);
  });

  bridge.chunkStress = chunkStress;
}

function nearestBondIndexFast(centroids, line) {
  let best = -1;
  let bestDistSq = Infinity;
  const cx = (line.p0.x + line.p1.x) * 0.5;
  const cy = (line.p0.y + line.p1.y) * 0.5;
  const cz = (line.p0.z + line.p1.z) * 0.5;

  for (let i = 0; i < centroids.length; ++i) {
    const centroid = centroids[i];
    if (!centroid) continue;
    const dx = centroid.x - cx;
    const dy = centroid.y - cy;
    const dz = centroid.z - cz;
    const distSq = dx * dx + dy * dy + dz * dz;
    if (distSq < bestDistSq) { bestDistSq = distSq; best = i; }
  }

  return best;
}

function solverSeverityScale(bridge) {
  const scale = bridge?.strengthScale;
  if (!scale || !Number.isFinite(scale) || scale <= 0) return 1.0;
  return 1.0 / scale;
}

function resetChunkColors(bridge) {
  if (bridge.chunkStress) bridge.chunkStress.clear();
  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) return;
    chunk.mesh.material.color.copy(chunk.baseColor);
    chunk.detached = false;
    chunk.stressSeverity = 0;
  });
  bridge.bonds.forEach((bond) => { if (bond) bond.severity = 0; });
}

// --------------------------- Car / Projectiles (visual glue only) ---------------------------

function updateProjectiles(world, bridge, delta) {
  bridge.projectiles = updateProjList(world, bridge.projectiles, delta);
  bridge.projectiles.forEach((proj) => {
    const body = world.getRigidBody(proj.bodyHandle);
    if (!body || !proj.mesh) return;
    const t = body.translation();
    const q = body.rotation();
    proj.mesh.position.set(t.x, t.y, t.z);
    proj.mesh.quaternion.set(q.x, q.y, q.z, q.w);
  });

  // Prune stale projectile collider handles from contact tracking to avoid Rapier errors
  if (bridge.projectileColliderHandles && bridge.activeContactColliders) {
    const stillAlive = new Set();
    bridge.projectiles.forEach((p) => { if (p?.colliderHandle != null) stillAlive.add(p.colliderHandle); });
    const toDelete = [];
    bridge.projectileColliderHandles.forEach((h) => {
      const exists = world.getCollider(h) != null;
      if (!exists || !stillAlive.has(h)) toDelete.push(h);
    });
    toDelete.forEach((h) => {
      bridge.projectileColliderHandles.delete(h);
      bridge.activeContactColliders.delete(h);
      if (bridge.colliderToNode) bridge.colliderToNode.delete(h);
    });
  }
}

function fireProjectile(world, bridge) {
  const p = spawnProj(world, { kind: 'box' });
  // create matching mesh
  const size = { x: 1.2, y: 1.2, z: 1.2 };
  const material = new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x552211 });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), material);
  mesh.castShadow = true; mesh.receiveShadow = true;
  const body = world.getRigidBody(p.bodyHandle);
  if (body) {
    const t = body.translation();
    mesh.position.set(t.x, t.y, t.z);
  }
  bridge.scene.add(mesh);
  bridge.projectiles.push({ ...p, mesh });
  bridge.projectileColliderHandles.add(p.colliderHandle);
  if (bridge.activeContactColliders) bridge.activeContactColliders.add(p.colliderHandle);
}

// --------------------------- UI / HUD ---------------------------

function setupControls(world, bridge) {
  // Gravity
  if (controlsUI.gravitySlider) {
    (controlsUI.gravitySlider as HTMLInputElement).value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (e) => {
      const value = parseFloat((e.target as HTMLInputElement).value);
      world.gravity = new RAPIER.Vector3(0, value, 0);
      bridge.gravity = value;
      if (HUD.gravityValue) HUD.gravityValue.textContent = value.toFixed(2);
      pushEvent(`Gravity set to ${value.toFixed(2)} m/s²`);
    });
    if (HUD.gravityValue) HUD.gravityValue.textContent = GRAVITY_DEFAULT.toFixed(2);
  }

  // Strength scale
  if (controlsUI.strengthSlider) {
    (controlsUI.strengthSlider as HTMLInputElement).value = bridge.strengthScale.toString();
    controlsUI.strengthSlider.addEventListener('input', (e) => {
      const value = parseFloat((e.target as HTMLInputElement).value);
      bridge.strengthScale = Number.isFinite(value) ? value : 1.0;
      applyStrengthScale(bridge.settings, BASE_LIMITS, bridge.strengthScale);
      bridge.solver.setSettings(bridge.settings);
      updateStrengthDisplay(bridge);
      pushEvent(`Material strength scaled to ${(bridge.strengthScale * 100).toFixed(1)}%`);
    });
  }

  // Max iterations
  if (controlsUI.iterSlider) {
    (controlsUI.iterSlider as HTMLInputElement).value = bridge.settings.maxSolverIterationsPerFrame.toString();
    controlsUI.iterSlider.addEventListener('input', (e) => {
      const value = parseInt((e.target as HTMLInputElement).value, 10);
      bridge.settings.maxSolverIterationsPerFrame = Number.isFinite(value) ? value : 32;
      bridge.solver.setSettings(bridge.settings);
      updateIterationDisplay(bridge);
    });
    updateIterationDisplay(bridge);
  }

  // Solver tolerance (10^x)
  if (controlsUI.toleranceSlider) {
    const initial = Math.log10(bridge.targetError);
    (controlsUI.toleranceSlider as HTMLInputElement).value = initial.toString();
    controlsUI.toleranceSlider.addEventListener('input', (e) => {
      const exponent = parseFloat((e.target as HTMLInputElement).value);
      bridge.targetError = Math.pow(10, exponent);
      updateToleranceDisplay(bridge);
    });
    updateToleranceDisplay(bridge);
  }

  // Fire projectile
  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => fireProjectile(world, bridge));
  }

  // Reset
  if (controlsUI.resetButton) {
    controlsUI.resetButton.addEventListener('click', () => resetBridge(world, bridge));
  }

  // Toggle debug (Blast lines + Rapier overlay)
  if (controlsUI.debugToggle) {
    controlsUI.debugToggle.addEventListener('click', () => {
      if (!bridge.debugRenderer) {
        return;
      }

      // bridge.debugEnabled = !bridge.debugEnabled;
      // const enabled = bridge.debugEnabled;
      const enabled = bridge.debugRenderer.toggle();
      bridge.debugEnabled = enabled;

      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
      bridge.debugHelper.object.visible = enabled && bridge.debugHelper.currentLines.length > 0;
      // const was = bridge.debugRenderer.enabled;
      // const now = enabled ? true : false;
      // if (was !== now) {
      // bridge.debugRenderer.toggle();
      console.log('Debug renderer toggled:', bridge.debugRenderer.enabled);
      // }
    });
    controlsUI.debugToggle.textContent = bridge.debugEnabled ? 'Hide Debug' : 'Show Debug';
  }
}

function applyStrengthScale(settings, baseLimits, scale) {
  settings.compressionElasticLimit = baseLimits.compressionElasticLimit * scale;
  settings.compressionFatalLimit = baseLimits.compressionFatalLimit * scale;
  settings.tensionElasticLimit = baseLimits.tensionElasticLimit * scale;
  settings.tensionFatalLimit = baseLimits.tensionFatalLimit * scale;
  settings.shearElasticLimit = baseLimits.shearElasticLimit * scale;
  settings.shearFatalLimit = baseLimits.shearFatalLimit * scale;
}

function updateStrengthDisplay(bridge) {
  if (HUD.strengthValue) {
    HUD.strengthValue.textContent = `${(bridge.strengthScale * 100).toFixed(1)}%`;
  }
}

function updateIterationDisplay(bridge) {
  if (HUD.iterValue) {
    HUD.iterValue.textContent = bridge.settings.maxSolverIterationsPerFrame.toString();
  }
}

function updateToleranceDisplay(bridge) {
  if (HUD.toleranceValue) {
    HUD.toleranceValue.textContent = formatTolerance(bridge.targetError);
  }
}

function pushEvent(_message) { /* disabled for perf */ }

function resetBridge(world, bridge) {
  // Remove projectiles
  bridge.projectiles.forEach((p) => {
    const body = world.getRigidBody(p.bodyHandle);
    if (body) world.removeRigidBody(body);
    p.mesh?.removeFromParent();
  });
  bridge.projectiles.length = 0;

  // Reset car
  const carBody = world.getRigidBody(bridge.car.bodyHandle);
  if (carBody) {
    carBody.setTranslation({ x: bridge.car.minX - bridge.car.margin, y: 2.0 + CAR_PROPS.height * 0.5 + 0.1, z: 0 }, true);
    carBody.setLinvel({ x: bridge.car.baseSpeed, y: 0, z: 0 }, true);
  }
  bridge.car.direction = 1;
  bridge.car.waitTimer = CAR_PROPS.idleDuration;
  bridge.car.speed = bridge.car.baseSpeed;
  bridge.car.active = true;

  // Rebuild colliders/bodies so supports return to their original locations
  const root = world.getRigidBody(bridge.bodyHandle);
  bridge.colliderToNode.clear();
  bridge.activeContactColliders.clear();
  bridge.pendingContactForces.clear();

  bridge.chunks.forEach((chunk) => {
    // Clean up existing collider/body state
    const oldCollider = world.getCollider(chunk.colliderHandle);
    if (oldCollider) world.removeCollider(oldCollider, false);

    if (chunk.isSupport) {
      const oldBody = world.getRigidBody(chunk.bodyHandle);
      if (oldBody) world.removeRigidBody(oldBody);

      const supportBodyDesc = RAPIER.RigidBodyDesc.fixed()
        .setTranslation(chunk.baseWorldPosition.x, chunk.baseWorldPosition.y, chunk.baseWorldPosition.z)
        .setCanSleep(false);
      const supportBody = world.createRigidBody(supportBodyDesc);

      const supportColliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
        .setTranslation(0, 0, 0)
        // .setFriction(1.0)
        .setFriction(0.25)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      const supportCollider = world.createCollider(supportColliderDesc, supportBody);

      chunk.bodyHandle = supportBody.handle;
      chunk.colliderHandle = supportCollider.handle;
      chunk.localOffset.set(0, 0, 0);
    } else {
      const oldBody = world.getRigidBody(chunk.bodyHandle);
      if (oldBody && oldBody.handle !== root.handle) world.removeRigidBody(oldBody);

      chunk.localOffset.copy(chunk.baseLocalOffset);

      const deckColliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        // .setFriction(1.0)
        .setFriction(0.25)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      const deckCollider = world.createCollider(deckColliderDesc, root);

      chunk.bodyHandle = root.handle;
      chunk.colliderHandle = deckCollider.handle;
    }

    chunk.active = true;
    chunk.detached = false;
    chunk.stressSeverity = 0;

    bridge.colliderToNode.set(chunk.colliderHandle, chunk.nodeIndex);
    bridge.activeContactColliders.add(chunk.colliderHandle);
  });

  // Restore solver with all bonds
  bridge.bonds.forEach((b) => (b.active = true));
  bridge.solver.destroy();
  bridge.solver = bridge.runtime.createExtSolver({
    nodes: bridge.solverNodes,
    bonds: bridge.scenario.bonds,
    settings: bridge.settings
  });

  // fresh event queue; next frames will be eventful again
  try { bridge.eventQueue = new RAPIER.EventQueue(true); } catch {}

  bridge.actorMap = new Map();
  bridge.solver.actors().forEach((actor) => {
    bridge.actorMap.set(actor.actorIndex, { bodyHandle: bridge.bodyHandle });
  });

  // Clear debug/overlay
  bridge.iterationCount = 0;
  bridge.lastError = { lin: 0, ang: 0 };
  bridge.overstressed = 0;
  bridge.failureDetected = false;
  bridge.forceBoost = 1.0;

  bridge.debugHelper.currentLines = [];
  updateDebugLineObject(bridge.debugHelper, [], bridge.debugEnabled);

  resetChunkColors(bridge);
  updateBondTable(bridge.bonds);
  if (HUD.eventLog) HUD.eventLog.innerHTML = '';
  
  // Reset two-phase migration state
  bridge.pendingSplitResults = [];
  bridge.pendingBodiesToCreate = [];
  bridge.pendingColliderMigrations = [];
  
  pushEvent('Bridge reset to initial state');
}
