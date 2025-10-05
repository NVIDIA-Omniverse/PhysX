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
import RAPIER from '@dimforge/rapier3d-compat';
import RapierDebugRenderer from './rapier-debug-renderer.js';

import {
  loadStressSolver,
  vec3,
  formatNumber,
  ExtDebugMode
} from './stress.js';
import { buildBridgeScenario } from './extBridgeScenario.js';

// --------------------------- Constants & UI ---------------------------

const GRAVITY_DEFAULT = -9.81;
const PROJECTILE_SPEED = 35;
const MAX_EVENTS = 8;
const IMPACT_MULTIPLIER = 3.5;

const CAR_PROPS = {
  mass: 5200,
  length: 3.6,
  width: 1.8,
  height: 1.2,
  speed: 4.5,
  idleDuration: 0.75
};

const BASE_LIMITS = {
  compressionElasticLimit: 0.05,
  compressionFatalLimit: 0.1,
  tensionElasticLimit: 0.05,
  tensionFatalLimit: 0.1,
  shearElasticLimit: 0.05,
  shearFatalLimit: 0.1
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

// --------------------------- Helpers / Types ---------------------------

class BridgeChunk {
  constructor({ nodeIndex, centroid, size, material }) {
    this.nodeIndex = nodeIndex;
    this.localOffset = new THREE.Vector3(centroid.x, centroid.y, centroid.z);
    this.localQuat = new THREE.Quaternion(0, 0, 0, 1);
    this.size = size; // { x, y, z } (box)
    this.mesh = createVoxelMesh(size, material);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.active = true;
  }
}

class BridgeBond {
  constructor({ index, node0, node1, centroid }) {
    this.index = index;
    this.node0 = node0;
    this.node1 = node1;
    this.centroid = centroid; // local (bridge frame)
    this.active = true;
    this.key = `${node0}-${node1}`;
  }
}

function createVoxelMesh(size, material) {
  const geom = new THREE.BoxGeometry(size.x, size.y, size.z);
  const mat = material.clone();
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function colorFromUint24(u24) {
  const r = ((u24 >> 16) & 0xff) / 255;
  const g = ((u24 >> 8) & 0xff) / 255;
  const b = (u24 & 0xff) / 255;
  return { r, g, b };
}

function formatTolerance(value) {
  return Number(value).toExponential(2);
}

// --------------------------- Main Entry ---------------------------

async function init() {
  await RAPIER.init();
  const runtime = await loadStressSolver();

  const { scene, renderer, camera, controls } = initThree();
  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));

  const bridge = buildBridge(scene, world, runtime);

  setupControls(world, bridge);

  const clock = new THREE.Clock();
  function loop() {
    const delta = clock.getDelta();

    // 1) Run Blast Ext solver and fracture if needed (bridge-local frame)
    updateBridgeLogical(bridge, delta);

    // 2) Step Rapier world & sync visuals
    world.step();
    if (bridge.debugRenderer?.enabled) bridge.debugRenderer.update();

    syncMeshes(world, bridge);
    updateProjectiles(world, bridge, delta);
    updateLoadVehicle(world, bridge, delta);

    renderDebugLines(bridge); // show Blast lines in world space

    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
  }
  loop();
}

init().catch((err) => {
  console.error('Failed to initialize bridge demo', err);
  pushEvent(`Initialization failed: ${err?.message ?? err}`);
});

// --------------------------- Scene / Build ---------------------------

function initThree() {
  const canvas = document.getElementById('bridge-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;

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

  window.addEventListener('resize', () => {
    const width = canvas.clientWidth || window.innerWidth;
    const height = window.innerHeight - 1;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  });

  return { scene, renderer, camera, controls };
}

function buildBridge(scene, world, runtime) {
  // --- Build the new scenario (grid deck + supports) ---
  const scenario = buildBridgeScenario(); // defaults are fine; tune as needed
  const settings = runtime.defaultExtSettings();
  // a bit more iterations by default
  settings.maxSolverIterationsPerFrame = 32;
  settings.graphReductionLevel = 0;
  applyStrengthScale(settings, BASE_LIMITS, 1.0);

  // Ext solver instance
  let solver = runtime.createExtSolver({
    nodes: scenario.nodes,
    bonds: scenario.bonds,
    settings
  });

  // --- Rapier: one dynamic parent body that holds all deck voxels initially ---
  const bridgeBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(0, 0, 0)
    .setCanSleep(false)
    .setLinearDamping(0.8)
    .setAngularDamping(1.2);
  const bridgeBody = world.createRigidBody(bridgeBodyDesc);

  const deckMaterial = new THREE.MeshStandardMaterial({
    color: 0x486fe3, roughness: 0.35, metalness: 0.45
  });
  const topMaterial = new THREE.MeshStandardMaterial({
    color: 0x5b86ff, roughness: 0.28, metalness: 0.55
  });

  // Use scenario spacing to size voxels similar to your ext visualization
  const spacing = scenario.spacing;
  const chunks = [];
  const meshByNode = new Map();

  // Create colliders/meshes for deck nodes (iy >= 0). Supports get visual columns only.
  scenario.nodes.forEach((node, nodeIndex) => {
    const coord = scenario.gridCoordinates[nodeIndex];
    if (!coord) return;

    if (coord.iy >= 0) {
      const sizeX = Math.max(spacing.x * 0.9, 0.25);
      const sizeY = Math.max(spacing.y * 0.9, 0.2);
      const sizeZ = scenario.widthSegments > 1
        ? Math.max(spacing.z * 0.9, 0.25)
        : Math.max(scenario.parameters.deckWidth * 0.9, 0.8);
      const size = { x: sizeX, y: sizeY, z: sizeZ };

      const mat = (coord.iy === scenario.thicknessLayers - 1) ? topMaterial : deckMaterial;
      const chunk = new BridgeChunk({ nodeIndex, centroid: node.centroid, size, material: mat });
      chunks.push(chunk);
      meshByNode.set(nodeIndex, chunk.mesh);
      scene.add(chunk.mesh);

      // Rapier collider on the single parent body (offset to node centroid)
      const colliderDesc = RAPIER.ColliderDesc.cuboid(size.x * 0.5, size.y * 0.5, size.z * 0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setFriction(1.0)
        .setRestitution(0.05);
      const collider = world.createCollider(colliderDesc, bridgeBody);
      // Optional: give mass to voxels based on scenario.mass (Rapier sums densities)
      // collider.setMass(node.mass ?? 1.0); // uncomment if you want heavier deck

      chunk.colliderHandle = collider.handle;
      chunk.bodyHandle = bridgeBody.handle;
    }
  });

  // Visual supports (no Rapier colliders; the bridge is free-floating like your original demo)
  const supportMaterial = new THREE.MeshStandardMaterial({
    color: 0x2f3e56, roughness: 0.6, metalness: 0.25
  });
  scenario.supportLinks.forEach((link) => {
    const deckNode = scenario.nodes[link.deckIndex];
    if (!deckNode) return;
    const columnHeight = scenario.parameters.pierHeight;
    const columnWidth = Math.max(spacing.x * 0.6, 0.4);
    const columnDepth = scenario.widthSegments > 1
      ? Math.max(spacing.z * 0.6, 0.4)
      : Math.max(scenario.parameters.deckWidth * 0.4, 0.4);
    const geo = new THREE.BoxGeometry(columnWidth, columnHeight, columnDepth);
    const mesh = new THREE.Mesh(geo, supportMaterial.clone());
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    const topY = deckNode.centroid.y - spacing.y * 0.5;
    mesh.position.set(deckNode.centroid.x, topY - columnHeight * 0.5, deckNode.centroid.z);
    scene.add(mesh);
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
  world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setTranslation(0, -4, 0), groundBody);

  // Rapier debug overlay
  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: false });

  // Blast debug helper (line segments)
  const debugHelper = createDebugLineHelper();
  scene.add(debugHelper.object);

  // Load vehicle (Rapier)
  const car = spawnLoadVehicle(world, scene);

  // Projectile list
  const projectiles = [];

  // Overlays
  pushEvent(`Bridge scenario loaded with ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`);
  updateBondTable([]); // start empty

  // Core bridge state
  return {
    runtime,
    world,
    scene,
    settings,
    solver,
    scenario,
    chunks,
    bonds,
    meshByNode,
    body: bridgeBody,
    car,
    projectiles,
    gravity: GRAVITY_DEFAULT,
    strengthScale: 1.0,
    targetError: 1.0e-6,
    iterationCount: 0,
    lastError: { lin: 0, ang: 0 },
    overstressed: 0,
    failureDetected: false,
    forceBoost: 1.0,
    debugEnabled: true,
    debugHelper,
    debugRenderer,
    // caches for mapping lines -> bonds quickly
    bondCentroids: scenario.bonds.map((b) => new THREE.Vector3(b.centroid.x, b.centroid.y, b.centroid.z))
  };
}

// --------------------------- Solver Update / Destruction ---------------------------

function updateBridgeLogical(bridge, delta) {
  advanceCarLogic(bridge, delta);

  const contact = computeContactNodesBridgeLocal(bridge);
  applyForcesAndSolve(bridge, contact);

  // Choose/perform fractures if solver reports overstress
  if (bridge.overstressed > 0) {
    attemptFractureFromDebug(bridge);
  }

  // Decay transient shock
  if (bridge.forceBoost > 1.0) {
    bridge.forceBoost = THREE.MathUtils.lerp(bridge.forceBoost, 1.0, Math.min(delta * 0.75, 1.0));
  }

  // UI updates
  updateOverlay(bridge, contact);
  updateIterationDisplay(bridge);
  updateToleranceDisplay(bridge);
  updateStrengthDisplay(bridge);
}

function advanceCarLogic(bridge, delta) {
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

function applyForcesAndSolve(bridge, contact) {
  const { solver } = bridge;
  solver.reset();

  // Gravity (solver-local frame)
  solver.addGravity(vec3(0.0, bridge.gravity, 0.0));

  // Car load distributed to contact nodes (solver-local)
  const contactNodes = contact.nodes ?? [];
  if (contactNodes.length > 0 && bridge.car.active) {
    const baseForcePerNode = (CAR_PROPS.mass * 9.81) / contactNodes.length;
    const severity = contact.column >= 0 ? 1.0 + contact.column * 1.4 : 1.0;
    const half = Math.floor(contactNodes.length / 2);
    contactNodes.forEach((nodeIndex, idx) => {
      const dynamicBoost = idx < half ? 1.0 : IMPACT_MULTIPLIER;
      const downwardForce = baseForcePerNode * severity * dynamicBoost * bridge.forceBoost;
      solver.addForce(nodeIndex, vec3(), vec3(0.0, -downwardForce, 0.0));
    });
  }

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
}

function attemptFractureFromDebug(bridge) {
  // Pull Blast debug lines in bridge-local frame
  const linesMax = bridge.solver.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 });
  const linesComp = bridge.solver.fillDebugRender({ mode: ExtDebugMode.Compression, scale: 1.0 });
  const linesTen = bridge.solver.fillDebugRender({ mode: ExtDebugMode.Tension, scale: 1.0 });
  const linesShear = bridge.solver.fillDebugRender({ mode: ExtDebugMode.Shear, scale: 1.0 });

  // Build candidate map: bondIndex -> {score, mode}
  const candidateByBond = new Map();

  const consider = (line, mode) => {
    const center = {
      x: 0.5 * (line.p0.x + line.p1.x),
      y: 0.5 * (line.p0.y + line.p1.y),
      z: 0.5 * (line.p0.z + line.p1.z)
    };
    const bondIndex = nearestBondIndex(bridge, center);
    if (bondIndex < 0) return;

    const length = lineMagnitude(line);
    const color = colorFromUint24(line.color0);
    const redNorm = color.r / Math.max(color.r + color.g + color.b, 1e-6); // favor "hot" lines
    const spacingX = Math.max(bridge.scenario.spacing.x, 1e-3);
    const lenNorm = THREE.MathUtils.clamp(length / (spacingX * 1.25), 0, 2);

    const score = 0.5 * lenNorm + 0.5 * redNorm;

    const curr = candidateByBond.get(bondIndex);
    if (!curr || score > curr.score) {
      candidateByBond.set(bondIndex, { score, mode });
    }
  };

  linesMax.forEach((l) => consider(l, 'max'));
  linesComp.forEach((l) => consider(l, 'compression'));
  linesTen.forEach((l) => consider(l, 'tension'));
  linesShear.forEach((l) => consider(l, 'shear'));

  if (candidateByBond.size === 0) return;

  // Pick best not-yet-broken bond that has both chunks existing
  const sorted = [...candidateByBond.entries()].sort((a, b) => b[1].score - a[1].score);
  for (const [bondIndex, info] of sorted) {
    const bond = bridge.bonds[bondIndex];
    if (!bond || !bond.active) continue;

    const chunkA = findChunkForNode(bridge, bond.node0);
    const chunkB = findChunkForNode(bridge, bond.node1);
    if (!chunkA || !chunkB) continue; // skip support-only bonds

    fractureBond(bridge, bondIndex, info.mode);
    break;
  }
}

function nearestBondIndex(bridge, center) {
  // naive nearest lookup (bonds count is modest in typical scenarios)
  let best = -1;
  let bestDistSq = Infinity;
  for (let i = 0; i < bridge.bonds.length; ++i) {
    const b = bridge.bonds[i];
    if (!b.active) continue;
    const c = bridge.bondCentroids[i];
    const dx = c.x - center.x;
    const dy = c.y - center.y;
    const dz = c.z - center.z;
    const d2 = dx * dx + dy * dy + dz * dz;
    if (d2 < bestDistSq) {
      bestDistSq = d2;
      best = i;
    }
  }
  // small tolerance to avoid accidental mapping across the bridge
  const tol = Math.pow(Math.max(bridge.scenario.spacing.x, 0.5) * 0.75, 2);
  return bestDistSq <= tol ? best : -1;
}

function fractureBond(bridge, bondIndex, mode = 'overstress') {
  const { world } = bridge;
  const bond = bridge.bonds[bondIndex];
  if (!bond || !bond.active) return;

  const chunkA = findChunkForNode(bridge, bond.node0);
  const chunkB = findChunkForNode(bridge, bond.node1);
  if (!chunkA || !chunkB) return;

  bond.active = false;
  pushEvent(`Bond ${bond.node0}-${bond.node1} failed due to ${mode}`);

  // Split node1 voxel into its own rigid body
  splitChunk(world, chunkB);

  // Rebuild Blast solver with bond removed
  const remainingBonds = [];
  for (let i = 0; i < bridge.scenario.bonds.length; ++i) {
    if (bridge.bonds[i].active) remainingBonds.push(bridge.scenario.bonds[i]);
  }
  bridge.solver.destroy();
  bridge.solver = bridge.runtime.createExtSolver({
    nodes: bridge.scenario.nodes,
    bonds: remainingBonds,
    settings: bridge.settings
  });

  // Clear lines immediately
  bridge.debugHelper.currentLines = [];
  updateDebugLineObject(bridge.debugHelper, [], false);
}

function splitChunk(world, chunk) {
  if (!chunk.active) return;

  const parent = world.getRigidBody(chunk.bodyHandle);
  const oldCollider = world.getCollider(chunk.colliderHandle);
  if (!parent || !oldCollider) return;

  // Remove old collider from parent
  world.removeCollider(oldCollider, /* wakeUpBodies: */ false);

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
    .setRestitution(0.05);

  const newCollider = world.createCollider(colliderDesc, newBody);

  chunk.bodyHandle = newBody.handle;
  chunk.colliderHandle = newCollider.handle;
  chunk.active = true;
}

// --------------------------- Rendering / Sync ---------------------------

function syncMeshes(world, bridge) {
  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) return;
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) return;

    const p = body.translation();
    const r = body.rotation();

    chunk.mesh.position.set(p.x, p.y, p.z);
    quat.set(r.x, r.y, r.z, r.w);
    chunk.mesh.quaternion.copy(quat);

    // apply local offset (bridge-local -> world)
    tmp.copy(chunk.localOffset).applyQuaternion(chunk.mesh.quaternion);
    chunk.mesh.position.add(tmp);
  });

  // Sync car mesh to its body
  const car = bridge.car;
  if (car) {
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
  if (!bridge.debugEnabled) {
    updateDebugLineObject(bridge.debugHelper, [], false);
    return;
  }
  // Fetch fresh lines from solver (bridge-local frame)
  const debugLines = bridge.solver.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 });
  bridge.debugHelper.currentLines = debugLines;

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

  // Update right panel summary with top lines (world lengths shown)
  const summary = summarizeLines(worldLines, 6);
  updateBondTable(summary);
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

function summarizeLines(lines, limit = 6) {
  const sorted = [...lines].sort((a, b) => lineMagnitude(b) - lineMagnitude(a));
  return sorted.slice(0, limit).map((line) => {
    const length = lineMagnitude(line);
    return {
      length,
      centerX: (line.p0.x + line.p1.x) * 0.5,
      centerY: (line.p0.y + line.p1.y) * 0.5,
      centerZ: (line.p0.z + line.p1.z) * 0.5,
      color: `#${(line.color0 & 0xffffff).toString(16).padStart(6, '0')}`
    };
  });
}

function lineMagnitude(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// --------------------------- Contact Mapping (Bridge-local) ---------------------------

function computeContactNodesBridgeLocal(bridge) {
  const { car, scenario } = bridge;

  // Car body world -> bridge-local (relative to current root body transform)
  const body = bridge.world.getRigidBody(bridge.body.handle);
  if (!body) return { column: -1, nodes: [] };

  const brPos = body.translation();
  const brRot = body.rotation();
  const qInv = new THREE.Quaternion(brRot.x, brRot.y, brRot.z, brRot.w).invert();

  const carBody = bridge.world.getRigidBody(car.bodyHandle);
  if (!carBody) return { column: -1, nodes: [] };
  const ct = carBody.translation();
  const carWorld = new THREE.Vector3(ct.x, ct.y, ct.z);

  const local = carWorld.sub(new THREE.Vector3(brPos.x, brPos.y, brPos.z)).applyQuaternion(qInv);

  const deckLength = scenario.origins.x + scenario.spacing.x * (scenario.spanSegments - 1) - scenario.origins.x;
  const minX = scenario.origins.x;
  const maxX = scenario.origins.x + scenario.spacing.x * (scenario.spanSegments - 1);

  if (deckLength <= 0) return { column: -1, nodes: [] };

  const progress = (local.x - minX) / (maxX - minX);
  if (progress <= 0 || progress >= 1) return { column: -1, nodes: [] };

  const segments = scenario.spanSegments;
  const clamped = THREE.MathUtils.clamp(progress, 0, 0.999);
  const column = Math.min(Math.floor(clamped * (segments - 1)), segments - 2);
  const nodes = [
    ...scenario.topColumnNodes[column],
    ...scenario.topColumnNodes[column + 1]
  ];
  return { column, nodes };
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
  const margin = CAR_PROPS.length * 0.5;

  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(minX - margin, 2.0 + CAR_PROPS.height * 0.5 + 0.1, 0)
    .setLinearDamping(0.4)
    .setAngularDamping(1.0);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(CAR_PROPS.length * 0.5, CAR_PROPS.height * 0.5, CAR_PROPS.width * 0.5)
      .setFriction(1.0)
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

function updateLoadVehicle(world, bridge, delta) {
  const car = bridge.car;
  if (!car) return;
  const body = world.getRigidBody(car.bodyHandle);
  if (!body) return;

  // keep some forward velocity (world), let physics handle contacts
  const v = body.linvel();
  body.setLinvel({ x: car.speed * car.direction, y: v.y, z: v.z }, true);
}

function spawnProjectile(world, bridge) {
  const origin = new THREE.Vector3(-20, 4, 0);
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(origin.x, origin.y, origin.z)
    .setCanSleep(false);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.6), body);
  body.setLinvel({ x: PROJECTILE_SPEED, y: -1, z: (Math.random() - 0.5) * 5 }, true);

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

  // Also apply a transient shock in the solver side
  bridge.forceBoost = Math.min(bridge.forceBoost + 0.75, 6.0);
  pushEvent('Projectile fired at bridge (force amplified)');
}

function updateProjectiles(world, bridge, delta) {
  const remaining = [];
  bridge.projectiles.forEach((proj) => {
    const body = world.getRigidBody(proj.bodyHandle);
    if (!body) {
      proj.mesh?.removeFromParent();
      return;
    }
    proj.ttl -= delta;
    const t = body.translation();
    if (proj.ttl <= 0 || t.y < -20) {
      world.removeRigidBody(body);
      proj.mesh?.removeFromParent();
      return;
    }
    proj.mesh.position.set(t.x, t.y, t.z);
    const q = body.rotation();
    proj.mesh.quaternion.set(q.x, q.y, q.z, q.w);
    remaining.push(proj);
  });
  bridge.projectiles = remaining;
}

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

  // Fire projectile
  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => spawnProjectile(world, bridge));
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

  // Re-attach all voxels to the single body (remove split bodies)
  const root = world.getRigidBody(bridge.body.handle);
  const rootPos = root.translation();
  const rootRot = root.rotation();

  bridge.chunks.forEach((chunk) => {
    if (!chunk.active) return;
    const body = world.getRigidBody(chunk.bodyHandle);
    const collider = world.getCollider(chunk.colliderHandle);
    if (body && body.handle !== root.handle && collider) {
      world.removeCollider(collider, false);
      const desc = RAPIER.ColliderDesc.cuboid(chunk.size.x * 0.5, chunk.size.y * 0.5, chunk.size.z * 0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setFriction(1.0)
        .setRestitution(0.05);
      const newCollider = world.createCollider(desc, root);
      chunk.bodyHandle = root.handle;
      chunk.colliderHandle = newCollider.handle;
    }
  });

  // Restore solver with all bonds
  bridge.bonds.forEach((b) => (b.active = true));
  bridge.solver.destroy();
  bridge.solver = bridge.runtime.createExtSolver({
    nodes: bridge.scenario.nodes,
    bonds: bridge.scenario.bonds,
    settings: bridge.settings
  });

  // Clear debug/overlay
  bridge.iterationCount = 0;
  bridge.lastError = { lin: 0, ang: 0 };
  bridge.overstressed = 0;
  bridge.failureDetected = false;
  bridge.forceBoost = 1.0;

  bridge.debugHelper.currentLines = [];
  updateDebugLineObject(bridge.debugHelper, [], bridge.debugEnabled);

  updateBondTable([]);
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

function updateBondTable(highlights) {
  if (!HUD.bondTable) return;
  HUD.bondTable.innerHTML = '';

  if (!highlights || highlights.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'bond-row';
    empty.textContent = 'No stressed bonds reported';
    HUD.bondTable.appendChild(empty);
    return;
  }

  highlights.forEach((item, idx) => {
    const row = document.createElement('div');
    row.className = 'bond-row';

    const title = document.createElement('div');
    title.textContent = `Line ${idx + 1}`;
    row.appendChild(title);

    const value = document.createElement('div');
    value.textContent = `${formatNumber(item.length, 6, 3)} m`;
    row.appendChild(value);

    const info = document.createElement('div');
    info.className = 'bond-info';
    info.textContent = `center (${formatNumber(item.centerX, 6, 2)}, ${formatNumber(item.centerY, 6, 2)}, ${formatNumber(item.centerZ, 6, 2)})`;
    row.appendChild(info);

    const colorLabel = document.createElement('div');
    colorLabel.className = 'bond-color';
    colorLabel.style.backgroundColor = item.color;
    row.appendChild(colorLabel);

    HUD.bondTable.appendChild(row);
  });
}

function updateOverlay(bridge, contact) {
  if (!HUD.overlay) return;
  const contactLabel = contact.column >= 0 ? `${contact.column} – ${contact.column + 1}` : 'off-deck';
  HUD.overlay.innerHTML = `
    <div>Car span: ${contactLabel}</div>
    <div>Iterations: ${bridge.iterationCount}/${bridge.settings.maxSolverIterationsPerFrame}</div>
    <div>Error lin ${formatNumber(bridge.lastError.lin, 7, 4)} | ang ${formatNumber(bridge.lastError.ang, 7, 4)}</div>
    <div>Overstressed bonds: ${bridge.overstressed}</div>
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
  if (!HUD.eventLog) return;
  const li = document.createElement('li');
  li.textContent = message;
  HUD.eventLog.prepend(li);
  while (HUD.eventLog.children.length > MAX_EVENTS) {
    HUD.eventLog.removeChild(HUD.eventLog.lastChild);
  }
}
