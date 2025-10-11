// Ensure a global harness object exists as early as possible
const __existingHarness = (typeof globalThis !== 'undefined') ? (globalThis.__bridgeExt ?? (globalThis.__bridgeExt = {})) : {};
if (__existingHarness) {
  __existingHarness.scriptLoaded = true;
}

/**
 * Rapier + Three.js + Blast Ext Stress Solver (destructible bridge)
 * -----------------------------------------------------------------
 * - Uses the new ExtStressSolver (fillDebugRender, overstress count, settings)
 * - Builds the modern grid-based bridge from extBridgeScenario.js
 * - Syncs meshes with Rapier bodies (colliders per deck voxel)
 * - Shows Blast debug lines (transformed into world space)
 * - Fractures bonds -> splits voxels off the main body (and rebuilds solver)
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Stats from 'three/addons/libs/stats.module.js';
import RAPIER from '@dimforge/rapier3d-compat';
// import RAPIER from '@dimforge/rapier3d-debug';
import RapierDebugRenderer from './rapier-debug-renderer.js';
import { handleSplitEvents as handleSplitEventsCore } from './bridge/handleSplitEvents.js';

import {
  loadStressSolver,
  vec3,
  formatNumber,
  ExtDebugMode,
  ExtForceMode
} from './stress.js';
import {
  applyForcesAndSolve as coreApplyForcesAndSolve,
  processSolverFractures as coreProcessSolverFractures,
  drainContactForces as coreDrainContactForces
} from './bridge/coreLogic.js';
import {
  spawnLoadVehicle as spawnCar,
  updateLoadVehicle as updateCar,
  spawnProjectile as spawnProj,
  updateProjectiles as updateProjList
} from './bridge/dynamics.js';
import { buildBridgeScenario } from './extBridgeScenario.js';
import { updateBondTable } from './bridge/ui.js';

// --------------------------- Constants & UI ---------------------------

const GRAVITY_DEFAULT = -9.81;
const PROJECTILE_SPEED = 5;
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
  /*
  compressionElasticLimit: 0.05,
  compressionFatalLimit: 0.1,
  tensionElasticLimit: 0.05,
  tensionFatalLimit: 0.1,
  shearElasticLimit: 0.05,
  shearFatalLimit: 0.1
  */
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
// const DETACHED_COLOR = new THREE.Color('#ffd166');
const DETACHED_COLOR = new THREE.Color('#3f3f3f');

const stressColorScratch = new THREE.Color();

const SEVERITY_EPSILON = 1.0e-6;
const COLOR_LOW_R = STRESS_COLOR_LOW.r;
const COLOR_HIGH_R = STRESS_COLOR_HIGH.r;
const COLOR_LOW_G = STRESS_COLOR_LOW.g;
const COLOR_HIGH_G = STRESS_COLOR_HIGH.g;
const COLOR_RANGE_R = Math.max(COLOR_HIGH_R - COLOR_LOW_R, 1.0e-6);
const COLOR_RANGE_G = Math.max(COLOR_LOW_G - COLOR_HIGH_G, 1.0e-6);

// --------------------------- Helpers / Types ---------------------------

class BridgeChunk {
  constructor({ nodeIndex, centroid, size, material, isSupport = false }) {
    this.nodeIndex = nodeIndex;
    this.baseWorldPosition = new THREE.Vector3(centroid.x, centroid.y, centroid.z);
    this.baseLocalOffset = new THREE.Vector3(centroid.x, centroid.y, centroid.z);
    this.localOffset = this.baseLocalOffset.clone();
    this.localQuat = new THREE.Quaternion(0, 0, 0, 1);
    this.size = size; // { x, y, z } (box)
    this.mesh = createVoxelMesh(size, material);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.active = true;
    this.baseColor = this.mesh.material.color.clone();
    this.stressSeverity = 0;
    this.detached = false;
    this.isSupport = isSupport;
  }
}

class BridgeBond {
  constructor({ index, node0, node1, centroid }) {
    this.index = index;
    this.node0 = node0;
    this.node1 = node1;
    this.centroid = centroid; // local (bridge frame)
    this.active = true;
    this.key = bondKey(node0, node1);
    this.severity = 0;
  }
}

function bondKey(a, b) {
  const n0 = Math.min(a, b);
  const n1 = Math.max(a, b);
  return `${n0}-${n1}`;
}

function createVoxelMesh(size, material) {
  const geom = new THREE.BoxGeometry(size.x, size.y, size.z);
  const mat = material.clone();
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function buildSolverNodesFromScenario(scenario) {
  if (!scenario?.nodes) {
    return [];
  }

  return scenario.nodes.map((node, nodeIndex) => {
    const coord = scenario.gridCoordinates?.[nodeIndex];
    const isSupport = coord?.iy === -1;
    return {
      centroid: node.centroid,
      mass: isSupport ? 0 : node.mass ?? 0,
      volume: isSupport ? 0 : node.volume ?? 0
    };
  });
}

function colorFromUint24(u24) {
  const r = ((u24 >> 16) & 0xff) / 255;
  const g = ((u24 >> 8) & 0xff) / 255;
  const b = (u24 & 0xff) / 255;
  return { r, g, b };
}

function clamp01(value) {
  if (!Number.isFinite(value)) {
    return 0;
  }
  return Math.min(Math.max(value, 0), 1);
}

function stressSeverityFromLine(line) {
  if (!line) {
    return 0;
  }

  const c0 = colorFromUint24(line.color0 ?? 0);
  const c1 = colorFromUint24(line.color1 ?? line.color0 ?? 0);
  const red = Math.max(c0.r, c1.r);
  const green = Math.min(c0.g, c1.g);

  const redSeverity = COLOR_RANGE_R > 0 ? (red - COLOR_LOW_R) / COLOR_RANGE_R : 0;
  const greenSeverity = COLOR_RANGE_G > 0 ? (COLOR_LOW_G - green) / COLOR_RANGE_G : 0;

  const severity = Math.max(redSeverity, greenSeverity, 0);
  return clamp01(severity);
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
      if (harness.console.length > 200) {
        harness.console.splice(0, harness.console.length - 200);
      }
    } catch (err) {
      // ignore logging errors
    }
  };
  console.log = ((orig) => (...args) => { logHook('log', ...args); orig(...args); })(console.log.bind(console));
  console.warn = ((orig) => (...args) => { logHook('warn', ...args); orig(...args); })(console.warn.bind(console));
  console.error = ((orig) => (...args) => { logHook('error', ...args); orig(...args); })(console.error.bind(console));

  await RAPIER.init();
  const runtime = await loadStressSolver();

  const { scene, renderer, camera, controls } = initThree();
  const stats = new Stats();
  try {
  document.body.appendChild(stats.dom);
  } catch (_) {}
  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));

  const bridge = buildBridge(scene, world, runtime);

  setupControls(world, bridge);

  const clock = new THREE.Clock();
  function loop() {
    const delta = clock.getDelta();

    const loopHarness = globalThis.__bridgeExt;
    if (loopHarness) {
      loopHarness.tickCount = (loopHarness.tickCount ?? 0) + 1;
      loopHarness.overstressed = bridge.overstressed ?? 0;
      loopHarness.actorCount = bridge.solver?.actorCount?.() ?? bridge.solver?.actors?.()?.length ?? null;
    }

    // 1) Update dynamics to produce contacts
    updateProjectiles(world, bridge, delta);
    // updateCar(world, bridge.car, delta);

    // 2) Step Rapier and drain contact forces into solver-space
    world.step(bridge.eventQueue);
    coreDrainContactForces(bridge);
    if (bridge.debugRenderer?.enabled) bridge.debugRenderer.update();

    // 3) Solve stresses and handle splits using shared logic
    updateBridgeLogical(bridge, delta);

    // 4) Visuals
    renderDebugLines(bridge);
    syncMeshes(world, bridge);

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
    // Fallback no-op renderer to keep logic running in headless CI
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
  } catch (_) {}

  return { scene, renderer, camera, controls };
}

function buildBridge(scene, world, runtime) {
  // --- Build the new scenario (grid deck + supports) ---
  const scenario = buildBridgeScenario(); // defaults are fine; tune as needed
  const solverNodes = buildSolverNodesFromScenario(scenario);
  const settings = runtime.defaultExtSettings();
  // a bit more iterations by default
  settings.maxSolverIterationsPerFrame = 32;
  settings.graphReductionLevel = 0;
  // const strengthScale = 10000000.0; // Strong
  // const strengthScale = 1.0; // Strong
  // const strengthScale = 0.6;
  // const strengthScale = 0.5;
  const strengthScale = 0.05;

  applyStrengthScale(settings, BASE_LIMITS, strengthScale);

  // Ext solver instance
  let solver = runtime.createExtSolver({
    nodes: solverNodes,
    bonds: scenario.bonds,
    settings
  });

  // --- Rapier: one static parent body that holds all deck voxels until fracture ---
  const bridgeBodyDesc = RAPIER.RigidBodyDesc.fixed()
    .setTranslation(0, 0, 0)
    .setCanSleep(false);
  const bridgeBody = world.createRigidBody(bridgeBodyDesc);

  const deckMaterial = new THREE.MeshStandardMaterial({
    color: 0x486fe3, roughness: 0.35, metalness: 0.45
  });
  const topMaterial = new THREE.MeshStandardMaterial({
    color: 0x5b86ff, roughness: 0.28, metalness: 0.55
  });
  const supportMaterial = new THREE.MeshStandardMaterial({
    color: 0x2f3e56, roughness: 0.6, metalness: 0.25
  });

  // Use scenario spacing to size voxels similar to your ext visualization
  const spacing = scenario.spacing;
  const chunks = [];
  const meshByNode = new Map();
  const supportChunks = [];
  const colliderToNode = new Map();
  const activeContactColliders = new Set();
  const pendingContactForces = new Map();

  // const nodeSizeScale = 0.9;
  const nodeSizeScale = 1.0;

  // Create colliders/meshes for deck nodes (iy >= 0). Supports get visual columns only.
  scenario.nodes.forEach((node, nodeIndex) => {
    const coord = scenario.gridCoordinates[nodeIndex];
    if (!coord) return;

    if (coord.iy >= 0) {
      const sizeX = Math.max(spacing.x * nodeSizeScale, 0.25);
      const sizeY = Math.max(spacing.y * nodeSizeScale, 0.2);
      const sizeZ = scenario.widthSegments > 1
        ? Math.max(spacing.z * nodeSizeScale, 0.25)
        : Math.max(scenario.parameters.deckWidth * nodeSizeScale, 0.8);
      const size = { x: sizeX, y: sizeY, z: sizeZ };

      const mat = (coord.iy === scenario.thicknessLayers - 1) ? topMaterial : deckMaterial;
      const chunk = new BridgeChunk({ nodeIndex, centroid: node.centroid, size, material: mat, isSupport: false });
      chunks.push(chunk);
      meshByNode.set(nodeIndex, chunk.mesh);
      scene.add(chunk.mesh);

      // Rapier collider on the single parent body (offset to node centroid)
      const colliderDesc = RAPIER.ColliderDesc.cuboid(size.x * 0.5, size.y * 0.5, size.z * 0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        // .setMass(10000.0)
        // .setMass(node.mass ?? 1.0)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      const collider = world.createCollider(colliderDesc, bridgeBody);
      collider.setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS);
      collider.setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      // Optional: give mass to voxels based on scenario.mass (Rapier sums densities)
      // collider.setMass(node.mass ?? 1.0); // uncomment if you want heavier deck

      chunk.colliderHandle = collider.handle;
      chunk.bodyHandle = bridgeBody.handle;
      colliderToNode.set(collider.handle, nodeIndex);
      activeContactColliders.add(collider.handle);
    } else if (coord.iy === -1) {
      const sizeX = Math.max(spacing.x * 0.6, 0.4);
      const sizeZ = scenario.widthSegments > 1
        ? Math.max(spacing.z * 0.6, 0.4)
        : Math.max(scenario.parameters.deckWidth * 0.4, 0.4);
      const sizeY = scenario.parameters.pierHeight;
      const size = { x: sizeX, y: sizeY, z: sizeZ };

      const chunk = new BridgeChunk({ nodeIndex, centroid: node.centroid, size, material: supportMaterial.clone(), isSupport: true });
      chunk.baseLocalOffset.set(0, 0, 0);
      chunk.localOffset.set(0, 0, 0);
      chunks.push(chunk);
      meshByNode.set(nodeIndex, chunk.mesh);
      scene.add(chunk.mesh);
      supportChunks.push(chunk);

      const supportBodyDesc = RAPIER.RigidBodyDesc.fixed()
        .setTranslation(node.centroid.x, node.centroid.y, node.centroid.z)
        .setCanSleep(false);
      const supportBody = world.createRigidBody(supportBodyDesc);

      const colliderDesc = RAPIER.ColliderDesc.cuboid(size.x * 0.5, size.y * 0.5, size.z * 0.5)
        .setTranslation(0, 0, 0)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      const collider = world.createCollider(colliderDesc, supportBody);
      collider.setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS);
      collider.setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);

      chunk.bodyHandle = supportBody.handle;
      chunk.colliderHandle = collider.handle;
      colliderToNode.set(collider.handle, nodeIndex);
      activeContactColliders.add(collider.handle);
    }
  });

  // Bonds (we mirror scenario.bonds with active flags)
  const bonds = scenario.bonds.map((b, i) => new BridgeBond({
    index: i,
    node0: b.node0,
    node1: b.node1,
    centroid: b.centroid
  }));

  // Rapier ground
  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20)
    .setTranslation(0, -4, 0)
    .setFriction(1.0)
    .setRestitution(0.0)
  , groundBody);

  // Rapier debug overlay
  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: false });

  // Blast debug helper (line segments)
  const debugHelper = createDebugLineHelper();
  scene.add(debugHelper.object);

  // Load vehicle (Rapier)
  // Ensure world exposes Rapier for shared dynamics
  world.RAPIER = RAPIER;
  const car = spawnCar(world, { bodyDescFactory: () => RAPIER.RigidBodyDesc.dynamic(), scene, THREE });

  // Projectile list
  const projectiles = [];

  // Overlays
  pushEvent(`Bridge scenario loaded with ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`);
  updateBondTable(bonds);

  // Rapier event queue for contact forces
  const eventQueue = new RAPIER.EventQueue(true);

  const topLayerNodeIndices = scenario.topColumnNodes.reduce((acc, column) => {
    if (!Array.isArray(column)) return acc;
    column.forEach((nodeIndex) => {
      if (Number.isInteger(nodeIndex)) {
        acc.push(nodeIndex);
      }
    });
    return acc;
  }, []);
  const topLayerNodePositions = topLayerNodeIndices.map((nodeIndex) => {
    const node = scenario.nodes[nodeIndex];
    if (!node?.centroid) {
      return null;
    }
    return new THREE.Vector3(node.centroid.x, node.centroid.y, node.centroid.z);
  });

  const actorMap = new Map();
  solver.actors().forEach((actor) => {
    actorMap.set(actor.actorIndex, { bodyHandle: bridgeBody.handle });
  });

  // Core bridge state
  return {
    runtime,
    world,
    scene,
    solverNodes,
    settings,
    solver,
    scenario,
    chunks,
    bonds,
    meshByNode,
    body: bridgeBody,
    car,
    projectiles,
    eventQueue,
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
    // caches for mapping lines -> bonds quickly
    bondCentroids: scenario.bonds.map((b) => new THREE.Vector3(b.centroid.x, b.centroid.y, b.centroid.z)),
    topLayerNodeIndices,
    topLayerNodePositions,
    colliderToNode,
    projectileColliderHandles: new Set(),
    activeContactColliders,
    pendingContactForces,
    contactForceScratch: [],
    actorMap
  };
}

// --------------------------- Solver Update / Destruction ---------------------------

function updateBridgeLogical(bridge, delta) {
  (void delta);

  // const contactForces = gatherSolverForcesFromRapier(bridge);
  // console.log('    updateBridgeLogical: contactForces', contactForces.length, contactForces);
  coreApplyForcesAndSolve(bridge);

  if (bridge.overstressed > 0) {
    coreProcessSolverFractures(bridge, RAPIER, CONTACT_FORCE_THRESHOLD);
  }

  if (bridge.forceBoost > 1.0) {
    bridge.forceBoost = THREE.MathUtils.lerp(bridge.forceBoost, 1.0, Math.min(delta * 0.75, 1.0));
  }

  // FIXME: re-enable after performance fixed
  // UI updates
  // updateOverlay(bridge, contact);
  // updateIterationDisplay(bridge);
  // updateToleranceDisplay(bridge);
  // updateStrengthDisplay(bridge);
}

function processSolverFractures(bridge) {
  const { solver } = bridge;
  if (!solver) {
    return;
  }

  const fractureSets = solver.generateFractureCommandsPerActor();
  // console.log(`    fracture sets generated: ${fractureSets.length}`);
  if (!Array.isArray(fractureSets) || fractureSets.length === 0) {
    return;
  }

  const splitResults = solver.applyFractureCommands(fractureSets);
  // console.log(`    split results returned: ${splitResults.length}`);
  // console.log(
  //   `processSolverFractures: overstressed=${bridge.overstressed}, fractureSets.length=${Array.isArray(fractureSets) ? fractureSets.length : 0}, splitResults.length=${Array.isArray(splitResults) ? splitResults.length : 0}`,
  //   {
  //     fractureSets,
  //     splitResults,
  //     bridge
  //   }
  // );
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return;
  }

  console.log(
    `processSolverFractures: overstressed=${bridge.overstressed}, fractureSets.length=${Array.isArray(fractureSets) ? fractureSets.length : 0}, splitResults.length=${Array.isArray(splitResults) ? splitResults.length : 0}`,
    {
      fractureSets,
      splitResults,
      bridge
    }
  );

  handleSplitEventsCore(bridge, splitResults, RAPIER, CONTACT_FORCE_THRESHOLD);
  rebuildActorsFromSolver(bridge);
  bridge.bonds.forEach((bond) => {
    if (bond) {
      bond.active = true;
      bond.severity = 0;
    }
  });
}

function handleSplitEvents(bridge, splitResults) {
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return;
  }

  const { solver } = bridge;
  const chunkByNode = new Map();
  bridge.chunks.forEach((chunk) => {
    chunkByNode.set(chunk.nodeIndex, chunk);
  });

  // Snapshot current actor→body mapping for context
  const actorMapSnapshot = [];
  if (bridge.actorMap) {
    for (const [actorIndex, entry] of bridge.actorMap.entries()) {
      actorMapSnapshot.push({ actorIndex, bodyHandle: entry?.bodyHandle });
    }
  }
  console.log(`    handleSplitEvents: split results length: ${splitResults.length}`, {
    splitResults,
    actorMap: actorMapSnapshot
  });

  splitResults.forEach((evt) => {
    const parentActorIndex = evt?.parentActorIndex;
    if (!Number.isInteger(parentActorIndex)) {
      console.error('    handleSplitEvents: parent actor index not found', evt);
      return;
    }

    const parentEntry = bridge.actorMap?.get(parentActorIndex);
    if (!parentEntry) {
      console.error('    handleSplitEvents: parent entry not found', parentActorIndex);
      return;
    }

    const { bodyHandle: parentBodyHandle } = parentEntry;
    const parentBody = parentBodyHandle != null ? bridge.world.getRigidBody(parentBodyHandle) : null;

    // Gather chunks currently attached to the parent body (pre-move), to verify coverage
    const prevChunksOnParent = [];
    bridge.chunks.forEach((c) => {
      if (c && c.bodyHandle === parentBodyHandle) prevChunksOnParent.push(c.nodeIndex);
    });

    const unionChildNodes = new Set();
    const assignedNodes = new Set();
    const missingChunkNodes = [];
    const duplicateNodes = [];
    const createdBodies = [];
    const supportNodesSkipped = [];

    // Detect if one of the children reuses the parent actor index; if so, we will keep the parent body
    const parentChild = (evt.children ?? []).find((c) => c && c.actorIndex === parentActorIndex);

    evt.children?.forEach((child) => {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) {
        return;
      }

      let bodyHandle = null;
      let body = null;

      // Note: mapping might not exist for a newly-created child. If this child is the parent actor, reuse the parent body.
      if (parentChild && child.actorIndex === parentActorIndex) {
        body = parentBody;
        bodyHandle = parentBodyHandle;
      } else {
        bodyHandle = bridge.actorMap?.get(child.actorIndex)?.bodyHandle;
      }

      if (bodyHandle != null) {
        body = bridge.world.getRigidBody(bodyHandle);
      }

      if (!body) {
        console.log('    handleSplitEvents: creating rigid body for child', {
          childActorIndex: child.actorIndex,
          nodeCount: child.nodes.length,
          inferredBodyHandle: bodyHandle,
          parentBodyHandle
        });
        // Spawn a new dynamic body at the parent's pose (if available) otherwise at chunk locale.
        const spawnDesc = RAPIER.RigidBodyDesc.dynamic();
        if (parentBody) {
          const pt = parentBody.translation();
          const pq = parentBody.rotation();
          spawnDesc
            .setTranslation(pt.x, pt.y, pt.z)
            .setRotation(pq)
            .setLinvel(parentBody.linvel().x, parentBody.linvel().y, parentBody.linvel().z)
            .setAngvel(parentBody.angvel().x, parentBody.angvel().y, parentBody.angvel().z);
        }

        body = bridge.world.createRigidBody(spawnDesc);
        bodyHandle = body.handle;

        if (bridge.actorMap) {
          bridge.actorMap.set(child.actorIndex, { bodyHandle });
        }
        createdBodies.push({ actorIndex: child.actorIndex, bodyHandle });
        console.log('    handleSplitEvents: created rigid body', { actorIndex: child.actorIndex, bodyHandle });
      }

      // Sanity log for body handle type/shape
      try {
        const rawHandle = body && body.handle;
        console.log('    handleSplitEvents: body handle sanity', {
          actorIndex: child.actorIndex,
          raw: rawHandle,
          type: typeof rawHandle,
          intCoerce: (typeof rawHandle === 'number') ? (rawHandle >>> 0) : null
        });
      } catch (e) {
        console.warn('    handleSplitEvents: body handle sanity failed', e);
      }

      // Attach colliders for every visible chunk belonging to this child.
      child.nodes.forEach((nodeIndex) => {
        unionChildNodes.add(nodeIndex);
        if (assignedNodes.has(nodeIndex)) {
          duplicateNodes.push(nodeIndex);
        }
        assignedNodes.add(nodeIndex);
        const chunk = chunkByNode.get(nodeIndex);
        if (!chunk) {
          missingChunkNodes.push(nodeIndex);
          return;
        }

        // Skip supports: keep them on their own fixed bodies
        if (chunk.isSupport) {
          supportNodesSkipped.push(nodeIndex);
          return;
        }

        if (chunk.colliderHandle != null) {
          const oldHandle = chunk.colliderHandle;
          const collider = bridge.world.getCollider(oldHandle);
          if (collider) {
            bridge.world.removeCollider(collider, false);
          }
          bridge.activeContactColliders.delete(oldHandle);
          bridge.colliderToNode.delete(oldHandle);
          chunk.colliderHandle = null;
        }

        // Ensure offsets are consistent on new body
        if (chunk.localOffset && chunk.baseLocalOffset) {
          chunk.localOffset.copy(chunk.baseLocalOffset);
        }

        const colliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
          .setTranslation(chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z)
          .setFriction(1.0)
          .setRestitution(0.0)
          .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);

        const collider = bridge.world.createCollider(colliderDesc, body);
        chunk.bodyHandle = bodyHandle;
        chunk.colliderHandle = collider.handle;
        chunk.detached = true;
        chunk.active = true;
        bridge.colliderToNode.set(collider.handle, chunk.nodeIndex);
        bridge.activeContactColliders.add(collider.handle);

        console.log('    handleSplitEvents: created collider for chunk', {
          nodeIndex: chunk.nodeIndex,
          actorIndex: child.actorIndex,
          bodyHandle: chunk.bodyHandle,
          colliderHandle: chunk.colliderHandle
        });
      });

      if (bridge.actorMap) {
        bridge.actorMap.set(child.actorIndex, { bodyHandle });
      }
    });

    // Summary for this parent split
    const notCovered = prevChunksOnParent.filter((n) => !assignedNodes.has(n));
    console.log('    handleSplitEvents: split summary', {
      parentActorIndex,
      parentBodyHandle,
      prevChunkCount: prevChunksOnParent.length,
      childrenCount: evt.children?.length ?? 0,
      unionChildNodes: unionChildNodes.size,
      assignedNodes: assignedNodes.size,
      missingChunkNodesCount: missingChunkNodes.length,
      missingChunkNodes: missingChunkNodes.slice(0, 32),
      duplicateNodesCount: duplicateNodes.length,
      duplicateNodes: duplicateNodes.slice(0, 32),
      supportNodesSkippedCount: supportNodesSkipped.length,
      supportNodesSkipped: supportNodesSkipped.slice(0, 32),
      notCoveredCount: notCovered.length,
      notCovered: notCovered.slice(0, 32),
      createdBodies
    });

    /*
    // If the parent child exists, we keep the parent body and mapping. Otherwise, remove mapping/body.
    if (!parentChild) {
      if (bridge.actorMap) {
        bridge.actorMap.delete(parentActorIndex);
      }
      // Safety: remove only if no chunk still references the parent body
      const stillHasChunks = bridge.chunks.some((c) => c && c.bodyHandle === parentBodyHandle);
      if (parentBody && !stillHasChunks) {
        bridge.world.removeRigidBody(parentBody);
      }
    }
    */
  });

  // Refresh solver actor table after the split to keep JS view in sync.
  const actorsAfter = solver.actors();
  const actorMapAfter = [];
  if (bridge.actorMap) {
    for (const [actorIndex, entry] of bridge.actorMap.entries()) {
      actorMapAfter.push({ actorIndex, bodyHandle: entry?.bodyHandle });
    }
  }
  console.log('    handleSplitEvents: solver actors after split', { actorsAfter, actorMapAfter });
}

function rebuildActorsFromSolver(bridge) {
  const solverActors = bridge.solver.actors();
  bridge.actorMap = bridge.actorMap ?? new Map();

  const knownActorIndices = new Set();
  solverActors.forEach((actor) => {
    knownActorIndices.add(actor.actorIndex);
  });

  // Clean up stale entries in the actor map.
  [...bridge.actorMap.keys()].forEach((key) => {
    if (!knownActorIndices.has(key)) {
      bridge.actorMap.delete(key);
    }
  });
}

function findBodyForNodes(bridge, nodes) {
  const nodeSet = new Set(nodes);
  const chunks = bridge.chunks.filter((chunk) => nodeSet.has(chunk.nodeIndex));
  if (chunks.length === 0) {
    return null;
  }

  const bodyHandle = chunks[0].bodyHandle;
  if (!chunks.every((chunk) => chunk.bodyHandle === bodyHandle)) {
    return null;
  }

  return bridge.world.getRigidBody(bodyHandle);
}

function advanceCarLogic(bridge, delta) {
  return;

  const car = bridge.car;
  if (!car.active) return;
  if (bridge.failureDetected) return;

  if (car.waitTimer > 0) {
    car.waitTimer = Math.max(0, car.waitTimer - delta);
    return;
  }

  // Move car kinematically along X in world; Rapier body follows below
  const body = bridge.world.getRigidBody(car.bodyHandle);
  if (!body) return;

  const tr = body.translation();
  body.setTranslation({ x: tr.x + car.speed * delta * car.direction, y: tr.y, z: tr.z }, true);

  if (tr.x > car.maxX + car.margin) {
    body.setTranslation({ x: car.maxX + car.margin, y: tr.y, z: tr.z }, true);
    car.direction = -1;
    car.waitTimer = CAR_PROPS.idleDuration;
  } else if (tr.x < car.minX - car.margin) {
    body.setTranslation({ x: car.minX - car.margin, y: tr.y, z: tr.z }, true);
    car.direction = 1;
    car.waitTimer = CAR_PROPS.idleDuration;
  }
}

function applyForcesAndSolve(bridge, solverForces) {
  const { solver } = bridge;
  const gravity = bridge.gravity * 0.1;

  // Gravity (solver-local frame)
  solver.addGravity(vec3(0.0, gravity, 0.0));

  (solverForces ?? []).forEach((force) => {
    solver.addForce(force.nodeIndex, force.localPoint, force.localForce, force.mode ?? ExtForceMode.Force);
    if (force.impulse) {
      // solver.addForce(force.nodeIndex, force.localPoint, force.impulse, ExtForceMode.Impulse);
    }
  });

  // Iterate solver
  let iterations = 0;
  let error = { lin: 0, ang: 0 };
  const maxIterations = bridge.settings.maxSolverIterationsPerFrame;
  for (; iterations < maxIterations; ++iterations) {
    solver.update();
    error = solver.stressError();
    if (solver.converged()) break;
    if (error.lin <= bridge.targetError && error.ang <= bridge.targetError) break;
  }

  bridge.iterationCount = iterations + 1;
  bridge.lastError = error;
  bridge.overstressed = solver.overstressedBondCount();

  updateBondStressFromSolver(bridge);
}

function splitChunk(world, chunk) {
  if (!chunk.active || chunk.isSupport) return;

  const parent = world.getRigidBody(chunk.bodyHandle);
  const oldCollider = world.getCollider(chunk.colliderHandle);
  if (!parent || !oldCollider) return;

  chunk.stressSeverity = 0;

  // Remove old collider from parent
  world.removeCollider(oldCollider, /* wakeUpBodies: */ false);

  // Reset local offset so the new rigid body keeps the chunk mesh at its centroid
  chunk.localOffset.copy(chunk.baseLocalOffset);

  // Create new rigid body at parent's current pose/vel
  const translation = parent.translation();
  const rotation = parent.rotation();
  const velocity = parent.linvel();
  const angvel = parent.angvel();

  const newBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(translation.x, translation.y, translation.z)
    .setRotation(rotation)
    .setLinearDamping(parent.linearDamping())
    .setAngularDamping(parent.angularDamping());
  const newBody = world.createRigidBody(newBodyDesc);

  if (velocity) {
    newBody.setLinvel({ x: velocity.x, y: velocity.y, z: velocity.z }, true);
  }
  if (angvel) {
    newBody.setAngvel({ x: angvel.x, y: angvel.y, z: angvel.z }, true);
  }

  // Recreate voxel collider on new body at the same local offset
  const colliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
    .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
    .setFriction(1.0)
    .setRestitution(0.0);

  const newCollider = world.createCollider(colliderDesc, newBody);

  chunk.bodyHandle = newBody.handle;
  chunk.colliderHandle = newCollider.handle;
  chunk.active = true;
  chunk.detached = true;
}

// --------------------------- Rendering / Sync ---------------------------

function syncMeshes(world, bridge) {
  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) {
      console.log(`No mesh found for chunk. nodeIndex: ${chunk.nodeIndex}, bodyHandle: ${chunk.bodyHandle}`);
      return;
    }

    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      // console.log(`No body found for chunk. nodeIndex: ${chunk.nodeIndex}, bodyHandle: ${chunk.bodyHandle}`, world.bodies.getAll());
      console.log(`No body found for chunk.`);
      return;
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
    // const color = chunk.detached ? DETACHED_COLOR : chunkColorForSeverity(chunk, stressValue);
    const color = chunkColorForSeverity(chunk, stressValue);
    if (chunk.mesh.material.color.r !== color.r || chunk.mesh.material.color.g !== color.g || chunk.mesh.material.color.b !== color.b) {
      chunk.mesh.material.color.copy(color);
    }
    // if (chunk.detached) {
    //   console.log('chunk.mesh.material.color', chunk.nodeIndex, chunk.detached, chunk.mesh.material.color);
    // }
  });
  // console.log('bridge.chunks', bridge.chunks, bridge);

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

  // Transform to world using the "root" bridge body transform
  const body = bridge.world.getRigidBody(bridge.body.handle);
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

function lineMagnitude(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// --------------------------- Contact Mapping (Bridge-local) ---------------------------


function drainContactForces(bridge) {
  const { eventQueue, colliderToNode, activeContactColliders, pendingContactForces, world } = bridge;
  if (!eventQueue || typeof eventQueue.drainContactForceEvents !== 'function' || !world) {
    return;
  }

  eventQueue.drainContactForceEvents((event) => {
    if (!event) {
      return;
    }

    const totalForce = event.totalForce?.();
    const magnitude = event.totalForceMagnitude?.();
    if (!totalForce || !(magnitude > 0)) {
      return;
    }

    const collider1 = event.collider1?.();
    const collider2 = event.collider2?.();
    const worldPoint = event.worldContactPoint ? event.worldContactPoint() : undefined;
    const worldPoint2 = event.worldContactPoint2 ? event.worldContactPoint2() : undefined;
    const totalImpulse = event.totalImpulse ? event.totalImpulse() : undefined;

    const register = (handle, direction) => {
      if (handle == null || !activeContactColliders.has(handle)) {
        return;
      }

      const collider = world.getCollider(handle);
      const bodyHandle = collider?.parent()?.handle;
      const body = bodyHandle != null ? world.getRigidBody(bodyHandle) : undefined;
      const bodyTranslation = body?.translation?.();

      const forceVec = {
        x: (totalForce.x ?? 0) * direction,
        y: (totalForce.y ?? 0) * direction,
        z: (totalForce.z ?? 0) * direction
      };
      const point = worldPoint
        ? { x: worldPoint.x ?? 0, y: worldPoint.y ?? 0, z: worldPoint.z ?? 0 }
        : (worldPoint2
            ? { x: worldPoint2.x ?? 0, y: worldPoint2.y ?? 0, z: worldPoint2.z ?? 0 }
            : (bodyTranslation
                ? { x: bodyTranslation.x ?? 0, y: bodyTranslation.y ?? 0, z: bodyTranslation.z ?? 0 }
                : { x: 0, y: 0, z: 0 }));
      const impulse = totalImpulse
        ? {
            x: (totalImpulse.x ?? 0) * direction,
            y: (totalImpulse.y ?? 0) * direction,
            z: (totalImpulse.z ?? 0) * direction
          }
        : undefined;

      const existing = pendingContactForces.get(handle);
      if (existing) {
        existing.force.x += forceVec.x;
        existing.force.y += forceVec.y;
        existing.force.z += forceVec.z;
        existing.point = point;
        if (impulse) {
          if (existing.impulse) {
            existing.impulse.x += impulse.x;
            existing.impulse.y += impulse.y;
            existing.impulse.z += impulse.z;
          } else {
            existing.impulse = { ...impulse };
          }
        }
      } else {
        pendingContactForces.set(handle, {
          force: forceVec,
          point,
          impulse: impulse || { x: 0, y: 0, z: 0 }
        });
      }
    };

    register(collider1, 1);
    register(collider2, -1);
  });
}

function accumulateContactForce(map, handle, force) {
  if (!map || handle == null || !force) {
    return;
  }
  const prev = map.get(handle);
  if (!prev) {
    map.set(handle, { x: force.x ?? 0, y: force.y ?? 0, z: force.z ?? 0 });
    return;
  }
  prev.x += force.x ?? 0;
  prev.y += force.y ?? 0;
  prev.z += force.z ?? 0;
}

// --------------------------- Car / Projectiles ---------------------------

function spawnLoadVehicle(world, scene) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(CAR_PROPS.length, CAR_PROPS.height, CAR_PROPS.width),
    new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.6 })
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);

  // Place car at the left off-deck position in world (we'll move it)
  const minX = -6.0; // will be normalized against bridge-local in computeContactNodes
  const maxX = +6.0;
  const margin = CAR_PROPS.length * 0.2;

  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(minX - margin, 2.0 + CAR_PROPS.height * 0.5 + 0.1, 0)
    .setLinearDamping(0.4)
    .setAngularDamping(1.0);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(CAR_PROPS.length * 0.5, CAR_PROPS.height * 0.5, CAR_PROPS.width * 0.5)
      .setFriction(1.0)
      .setRestitution(0.0)
      .setMass(CAR_PROPS.mass),
    body
  );

  // Initial slow drift to kick interactions
  body.setLinvel({ x: CAR_PROPS.speed, y: 0, z: 0 }, true);

  return {
    mesh,
    bodyHandle: body.handle,
    colliderHandle: collider.handle,
    mass: CAR_PROPS.mass,
    speed: CAR_PROPS.speed,
    baseSpeed: CAR_PROPS.speed,
    direction: 1,
    waitTimer: CAR_PROPS.idleDuration,
    active: true,
    minX,
    maxX,
    margin
  };
}

// use shared dynamics update (no-op here as we move car in tests via shared code)

function spawnBallProjectile(world, bridge) {
  // Fire from above the bridge, aiming downward
  const origin = new THREE.Vector3(0, 20, 0); // x=0 (center), y=20 (above), z=0
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(origin.x, origin.y, origin.z)
    .setCanSleep(false);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.6), body);
  body.setLinvel({ x: (Math.random() - 0.5) * 5, y: -PROJECTILE_SPEED, z: (Math.random() - 0.5) * 5 }, true);

  const material = new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x552211 });
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.6, 20, 20), material);
  mesh.castShadow = true; mesh.receiveShadow = true;
  mesh.position.copy(origin);
  bridge.scene.add(mesh);

  bridge.projectiles.push({
    bodyHandle: body.handle,
    colliderHandle: collider.handle,
    mesh,
    ttl: 12
  });
  bridge.projectileColliderHandles.add(collider.handle);

  // Also apply a transient shock in the solver side
  bridge.forceBoost = Math.min(bridge.forceBoost + 0.75, 6.0);
  pushEvent('Projectile fired at bridge (force amplified)');
}

function spawnCuboidProjectile(world, bridge) {
  // Fire from above the bridge, aiming downward
  const scale = 1.0;

  // Add some x,z randomization to the spawn position
  const origin = new THREE.Vector3(
    (Math.random() - 0.5) * 6, // random x in [-3, 3]
    5,
    (Math.random() - 0.5) * 6  // random z in [-3, 3]
  );

  const size = { x: 1.2 * scale, y: 1.2 * scale, z: 1.2 * scale };
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(origin.x, origin.y, origin.z)
    .setCanSleep(false);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(size.x * 0.5, size.y * 0.5, size.z * 0.5)
      .setMass(100000000.0 * scale * scale)
      .setFriction(0.0)
      .setRestitution(0.0)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0.0),
    body
  );
  body.setLinvel({ x: (Math.random() - 0.5) * 5, y: 0 * -PROJECTILE_SPEED, z: (Math.random() - 0.5) * 5 }, true);

  const material = new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x552211 });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), material);
  mesh.castShadow = true; mesh.receiveShadow = true;
  mesh.position.copy(origin);
  bridge.scene.add(mesh);

  bridge.projectiles.push({
    bodyHandle: body.handle,
    colliderHandle: collider.handle,
    mesh,
    ttl: 12
  });
  bridge.projectileColliderHandles.add(collider.handle);
  if (bridge.activeContactColliders) bridge.activeContactColliders.add(collider.handle);

  // Also apply a transient shock in the solver side
  // bridge.forceBoost = Math.min(bridge.forceBoost + 0.75, 6.0);
  pushEvent('Projectile fired at bridge (force amplified)');
}

// For now, spawnProjectile is an alias for spawnCuboidProjectile
const spawnProjectile = spawnCuboidProjectile;

// visual updates kept, physics list maintenance via shared updateProjList

// --------------------------- UI / HUD ---------------------------

function setupControls(world, bridge) {
  // Gravity
  if (controlsUI.gravitySlider) {
    controlsUI.gravitySlider.value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      world.gravity = new RAPIER.Vector3(0, value, 0);
      bridge.gravity = value;
      if (HUD.gravityValue) HUD.gravityValue.textContent = value.toFixed(2);
      pushEvent(`Gravity set to ${value.toFixed(2)} m/s²`);
    });
    if (HUD.gravityValue) HUD.gravityValue.textContent = GRAVITY_DEFAULT.toFixed(2);
  }

  // Strength scale
  if (controlsUI.strengthSlider) {
    controlsUI.strengthSlider.value = bridge.strengthScale.toString();
    controlsUI.strengthSlider.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      bridge.strengthScale = Number.isFinite(value) ? value : 1.0;
      applyStrengthScale(bridge.settings, BASE_LIMITS, bridge.strengthScale);
      bridge.solver.setSettings(bridge.settings);
      updateStrengthDisplay(bridge);
      pushEvent(`Material strength scaled to ${(bridge.strengthScale * 100).toFixed(1)}%`);
    });
  }

  // Max iterations
  if (controlsUI.iterSlider) {
    controlsUI.iterSlider.value = bridge.settings.maxSolverIterationsPerFrame.toString();
    controlsUI.iterSlider.addEventListener('input', (e) => {
      const value = parseInt(e.target.value, 10);
      bridge.settings.maxSolverIterationsPerFrame = Number.isFinite(value) ? value : 32;
      bridge.solver.setSettings(bridge.settings);
      updateIterationDisplay(bridge);
    });
    updateIterationDisplay(bridge);
  }

  // Solver tolerance (10^x)
  if (controlsUI.toleranceSlider) {
    const initial = Math.log10(bridge.targetError);
    controlsUI.toleranceSlider.value = initial.toString();
    controlsUI.toleranceSlider.addEventListener('input', (e) => {
      const exponent = parseFloat(e.target.value);
      bridge.targetError = Math.pow(10, exponent);
      updateToleranceDisplay(bridge);
    });
    updateToleranceDisplay(bridge);
  }

  // console.log('bridge.settings', bridge.settings);

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
      bridge.debugEnabled = !bridge.debugEnabled;
      const enabled = bridge.debugEnabled;
      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
      bridge.debugHelper.object.visible = enabled && bridge.debugHelper.currentLines.length > 0;
      const was = bridge.debugRenderer.enabled;
      const now = enabled ? true : false;
      if (was !== now) bridge.debugRenderer.toggle();
    });
    controlsUI.debugToggle.textContent = 'Hide Debug';
  }
}

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
  const root = world.getRigidBody(bridge.body.handle);
  bridge.colliderToNode.clear();
  bridge.activeContactColliders.clear();
  bridge.pendingContactForces.clear();

  bridge.chunks.forEach((chunk) => {
    // Clean up existing collider/body state
    const oldCollider = world.getCollider(chunk.colliderHandle);
    if (oldCollider) {
      world.removeCollider(oldCollider, false);
    }

    if (chunk.isSupport) {
      const oldBody = world.getRigidBody(chunk.bodyHandle);
      if (oldBody) {
        world.removeRigidBody(oldBody);
      }

      const supportBodyDesc = RAPIER.RigidBodyDesc.fixed()
        .setTranslation(chunk.baseWorldPosition.x, chunk.baseWorldPosition.y, chunk.baseWorldPosition.z)
        .setCanSleep(false);
      const supportBody = world.createRigidBody(supportBodyDesc);

      const supportColliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
        .setTranslation(0, 0, 0)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(CONTACT_FORCE_THRESHOLD);
      const supportCollider = world.createCollider(supportColliderDesc, supportBody);

      chunk.bodyHandle = supportBody.handle;
      chunk.colliderHandle = supportCollider.handle;
      chunk.localOffset.set(0, 0, 0);
    } else {
      const oldBody = world.getRigidBody(chunk.bodyHandle);
      if (oldBody && oldBody.handle !== root.handle) {
        world.removeRigidBody(oldBody);
      }

      chunk.localOffset.copy(chunk.baseLocalOffset);

      const deckColliderDesc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setFriction(1.0)
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

  bridge.actorMap = new Map();
  bridge.solver.actors().forEach((actor) => {
    bridge.actorMap.set(actor.actorIndex, { bodyHandle: bridge.body.handle });
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
  updateOverlay(bridge, { column: -1, nodes: [] });
  if (HUD.eventLog) HUD.eventLog.innerHTML = '';
  pushEvent('Bridge reset to initial state');
}

// --------------------------- Small Utilities ---------------------------

function findChunkForNode(bridge, nodeIndex) {
  // Only deck nodes (iy >= 0) have chunks
  const coord = bridge.scenario.gridCoordinates[nodeIndex];
  if (!coord || coord.iy < 0) return null;
  // We populated chunks in the same order as deck nodes; search by nodeIndex
  return bridge.chunks.find((c) => c.nodeIndex === nodeIndex) ?? null;
}

function applyStrengthScale(settings, baseLimits, scale) {
  settings.compressionElasticLimit = baseLimits.compressionElasticLimit * scale;
  settings.compressionFatalLimit = baseLimits.compressionFatalLimit * scale;
  settings.tensionElasticLimit = baseLimits.tensionElasticLimit * scale;
  settings.tensionFatalLimit = baseLimits.tensionFatalLimit * scale;
  settings.shearElasticLimit = baseLimits.shearElasticLimit * scale;
  settings.shearFatalLimit = baseLimits.shearFatalLimit * scale;
}

function updateOverlay(bridge, contact) {
  if (!HUD.overlay) return;
  const contactLabel = contact.column >= 0 ? `${contact.column} – ${contact.column + 1}` : 'off-deck';
  HUD.overlay.innerHTML = `
    <div>Car span: ${contactLabel}</div>
    <div>Iterations: ${bridge.iterationCount}/${bridge.settings.maxSolverIterationsPerFrame}</div>
    <div>Error lin ${formatNumber(bridge.lastError.lin, 7, 4)} | ang ${formatNumber(bridge.lastError.ang, 7, 4)}</div>
    <div>Active bonds: ${bridge.bonds.filter((b) => b.active).length}/${bridge.bonds.length}</div>
    <div>Force multiplier: ${bridge.forceBoost.toFixed(2)}x</div>
    <div>Status: ${bridge.failureDetected ? 'FAILED' : 'Active'}</div>
  `;
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

function pushEvent(message) {
  return; // Disabled

  console.log('pushEvent', message);
  if (!HUD.eventLog) return;
  const li = document.createElement('li');
  li.textContent = message;
  HUD.eventLog.prepend(li);
  while (HUD.eventLog.children.length > MAX_EVENTS) {
    HUD.eventLog.removeChild(HUD.eventLog.lastChild);
  }
}

// Configurable threshold value for stress severity coloring.
// "Medium" is the severity at which the color is fully blended to STRESS_COLOR_MEDIUM.
// "High" is the severity at which the color starts blending toward STRESS_COLOR_HIGH.
// In this model, both thresholds are the same, as the transition to "high" begins where "medium" ends.
const STRESS_SEVERITY_MEDIUM_THRESHOLD = 0.5;
const STRESS_SEVERITY_HIGH_THRESHOLD = STRESS_SEVERITY_MEDIUM_THRESHOLD;

function chunkColorForSeverity(chunk, severity) {
  if (severity <= 0) {
    return chunk.baseColor;
  }
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
  bonds.forEach((bond) => {
    if (bond) {
      bond.severity = 0;
    }
  });

  if (!Array.isArray(debugLines) || debugLines.length === 0) {
    bridge.chunkStress = chunkStress;
    chunks.forEach((chunk) => {
      chunk.stressSeverity = 0;
    });
    return;
  }

  debugLines.forEach((line) => {
    const bondIndex = nearestBondIndexFast(bondCentroids, line);
    if (bondIndex < 0) {
      return;
    }

    const bond = bonds[bondIndex];
    if (!bond || !bond.active) {
      return;
    }

    const severity = stressSeverityFromLine(line);
    if (!(severity > 0)) {
      return;
    }

    bond.severity = Math.max(bond.severity ?? 0, severity);
  });

  const severitySumByNode = new Map();
  const severityCountByNode = new Map();

  bonds.forEach((bond) => {
    if (!bond || !bond.active) {
      return;
    }
    const severity = bond.severity ?? 0;
    const node0Sum = severitySumByNode.get(bond.node0) ?? 0;
    const node1Sum = severitySumByNode.get(bond.node1) ?? 0;
    severitySumByNode.set(bond.node0, node0Sum + severity);
    severitySumByNode.set(bond.node1, node1Sum + severity);

    const node0Count = severityCountByNode.get(bond.node0) ?? 0;
    const node1Count = severityCountByNode.get(bond.node1) ?? 0;
    severityCountByNode.set(bond.node0, node0Count + 1);
    severityCountByNode.set(bond.node1, node1Count + 1);
  });

  chunks.forEach((chunk) => {
    const sum = severitySumByNode.get(chunk.nodeIndex) ?? 0;
    const count = severityCountByNode.get(chunk.nodeIndex) ?? 0;
    const avgSeverity = count > 0 ? sum / Math.max(count, 1) : 0;
    chunk.stressSeverity = avgSeverity;
    chunkStress.set(chunk.nodeIndex, chunk.stressSeverity);
  });

  bridge.chunkStress = chunkStress;
}

function resetChunkColors(bridge) {
  if (bridge.chunkStress) {
    bridge.chunkStress.clear();
  }
  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) {
      return;
    }
    chunk.mesh.material.color.copy(chunk.baseColor);
    chunk.detached = false;
    chunk.stressSeverity = 0;
  });
  bridge.bonds.forEach((bond) => {
    if (bond) {
      bond.severity = 0;
    }
  });
}

function nearestBondIndexFast(centroids, line) {
  let best = -1;
  let bestDistSq = Infinity;
  const cx = (line.p0.x + line.p1.x) * 0.5;
  const cy = (line.p0.y + line.p1.y) * 0.5;
  const cz = (line.p0.z + line.p1.z) * 0.5;

  for (let i = 0; i < centroids.length; ++i) {
    const centroid = centroids[i];
    if (!centroid) {
      continue;
    }
    const dx = centroid.x - cx;
    const dy = centroid.y - cy;
    const dz = centroid.z - cz;
    const distSq = dx * dx + dy * dy + dz * dz;
    if (distSq < bestDistSq) {
      bestDistSq = distSq;
      best = i;
    }
  }

  return best;
}

function colorIntensity(rgb) {
  const r = (rgb >> 16) & 0xff;
  const g = (rgb >> 8) & 0xff;
  const b = rgb & 0xff;
  const max = Math.max(r, g, b);
  return max / 255;
}

function colorForSeverity(baseColor, severity) {
  if (severity <= 0) {
    return baseColor.clone();
  }
  const color = baseColor.clone();
  const mediumBlend = Math.min(severity, 1);
  color.lerp(STRESS_COLOR_MEDIUM, mediumBlend);
  if (severity >= 0.65) {
    const highBlend = Math.min((severity - 0.65) / 0.35, 1);
    color.lerp(STRESS_COLOR_HIGH, highBlend);
  }
  return color;
}

function updateBondStressFromSolver(bridge) {
  if (!bridge?.solver || !bridge?.bonds) {
    return;
  }
  updateBondTable(bridge.bonds);
}

function solverSeverityScale(bridge) {
  const scale = bridge?.strengthScale;
  if (!scale || !Number.isFinite(scale) || scale <= 0) {
    return 1.0;
  }
  return 1.0 / scale;
}

function gatherSolverForcesFromRapier(bridge) {
  const {
    world,
    body,
    colliderToNode,
    activeContactColliders,
    pendingContactForces,
    contactForceScratch
  } = bridge;

  if (!world || !body || activeContactColliders.size === 0) {
    return [];
  }

  const rootBody = world.getRigidBody(body.handle);
  if (!rootBody) {
    return [];
  }

  const qInv = new THREE.Quaternion(
    rootBody.rotation().x,
    rootBody.rotation().y,
    rootBody.rotation().z,
    rootBody.rotation().w
  ).invert();

  const results = contactForceScratch ?? [];
  results.length = 0;

  activeContactColliders.forEach((handle) => {
    const contact = pendingContactForces.get(handle);
    const nodeIndex = colliderToNode.get(handle);
    if (!contact || !Number.isInteger(nodeIndex) || nodeIndex < 0) {
      return;
    }

    const localForce = new THREE.Vector3(contact.force.x, contact.force.y, contact.force.z).applyQuaternion(qInv);
    const localPoint = new THREE.Vector3(contact.point.x, contact.point.y, contact.point.z).applyQuaternion(qInv);
    const impulseVec = contact.impulse
      ? new THREE.Vector3(contact.impulse.x, contact.impulse.y, contact.impulse.z).applyQuaternion(qInv)
      : undefined;

    results.push({
      nodeIndex,
      localForce: vec3(localForce.x, localForce.y, localForce.z),
      localPoint: vec3(localPoint.x, localPoint.y, localPoint.z),
      impulse: impulseVec ? vec3(impulseVec.x, impulseVec.y, impulseVec.z) : undefined,
      mode: ExtForceMode.Force
    });
  });

  pendingContactForces.clear();
  return results;
}

// visual updates kept, physics list maintenance via shared updateProjList
function updateProjectiles(world, bridge, delta) {
  // physics list maintenance
  bridge.projectiles = updateProjList(world, bridge.projectiles, delta);
  // visuals
  bridge.projectiles.forEach((proj) => {
    const body = world.getRigidBody(proj.bodyHandle);
    if (!body || !proj.mesh) return;
    const t = body.translation();
    const q = body.rotation();
    proj.mesh.position.set(t.x, t.y, t.z);
    proj.mesh.quaternion.set(q.x, q.y, q.z, q.w);
  });
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
