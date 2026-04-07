import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import RapierDebugRenderer from './rapier-debug-renderer.js';

import { loadStressSolver, vec3, formatNumber, ExtDebugMode } from './stress.js';
import { buildBridgeScenario } from './extBridgeScenario.js';

const GRAVITY_DEFAULT = -9.81;
const PROJECTILE_SPEED = 35;
const MAX_EVENTS = 8;
const IMPACT_MULTIPLIER = 3.5;

const BASE_LIMITS = {
  compressionElasticLimit: 0.05,
  compressionFatalLimit: 0.1,
  tensionElasticLimit: 0.05,
  tensionFatalLimit: 0.1,
  shearElasticLimit: 0.05,
  shearFatalLimit: 0.1
};

function createSeverity() {
  return {
    compression: 0,
    tension: 0,
    shear: 0,
    max: 0
  };
}

class BridgeChunk {
  constructor(desc) {
    Object.assign(this, desc);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.mesh = null;
    this.active = true;
    this.pendingForce = { x: 0, y: 0, z: 0 };
    this.severity = createSeverity();
    this.localOffset = new THREE.Vector3(this.localPosition.x, this.localPosition.y, this.localPosition.z);
    this.localQuat = desc.localRotation instanceof THREE.Quaternion
      ? desc.localRotation.clone()
      : new THREE.Quaternion(
          this.localRotation.x ?? 0,
          this.localRotation.y ?? 0,
          this.localRotation.z ?? 0,
          this.localRotation.w ?? 1
        );
  }
}

class BridgeBond {
  constructor(desc) {
    Object.assign(this, desc);
    this.active = true;
    this.severity = createSeverity();
    this.key = `${this.node0}-${this.node1}`;
  }
}

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

async function init() {
  try {
  await RAPIER.init();
    const runtime = await loadStressSolver();
  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));
  const { scene, renderer, camera, controls } = initThree();
    const bridge = buildBridge(scene, world, runtime);

    setupControls(bridge);

  const clock = new THREE.Clock();

  function loop() {
    const delta = clock.getDelta();
      updateBridge(bridge, delta);
      bridge.world.step();
    if (bridge.debugRenderer?.enabled) {
      bridge.debugRenderer.update();
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
  }

  loop();
  } catch (err) {
    console.error('Failed to initialize bridge demo', err);
    pushEvent(`Initialization failed: ${err?.message ?? err}`);
  }
}

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
  const scenario = buildBridgeScenario();
  const solverNodes = scenario.nodes.map((node) => ({
    centroid: vec3(node.centroid.x, node.centroid.y, node.centroid.z),
    mass: node.mass,
    volume: node.volume
  }));
  const solverBonds = scenario.bonds.map((bond) => ({
    centroid: vec3(bond.centroid.x, bond.centroid.y, bond.centroid.z),
    normal: vec3(bond.normal.x, bond.normal.y, bond.normal.z),
    area: bond.area,
      node0: bond.node0,
      node1: bond.node1
  }));

  const chunks = createChunksFromScenario(scenario);
  const bonds = createBondsFromScenario(scenario, chunks);

  const bridgeBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(0, 0, 0)
    .setLinearDamping(0.8)
    .setAngularDamping(1.2)
    .setCanSleep(false);
  const bridgeBody = world.createRigidBody(bridgeBodyDesc);

  const materials = {
    deck: new THREE.MeshStandardMaterial({ color: 0x486fe3, roughness: 0.42, metalness: 0.5 }),
    top: new THREE.MeshStandardMaterial({ color: 0x5b86ff, roughness: 0.3, metalness: 0.6 }),
    support: new THREE.MeshStandardMaterial({ color: 0x2f3e56, roughness: 0.6, metalness: 0.25 }),
    base: new THREE.MeshStandardMaterial({ color: 0x1d2533, roughness: 0.75, metalness: 0.2 })
  };

  const chunkMeshes = [];
  chunks.forEach((chunk) => {
    const colliderDesc = createColliderForChunk(chunk);
    colliderDesc.setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z);
    const collider = world.createCollider(colliderDesc, bridgeBody);
    if (chunk.mass > 0) {
      collider.setMass(chunk.mass);
    }
    chunk.colliderHandle = collider.handle;
    chunk.bodyHandle = bridgeBody.handle;

    let material = materials.deck;
    if (chunk.kind === 'support-top') {
      material = materials.top;
    } else if (chunk.kind === 'support-column') {
      material = materials.support;
    } else if (chunk.kind === 'support-base') {
      material = materials.base;
    }

    const mesh = createMeshForChunk(chunk, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.position.copy(chunk.localOffset);
    mesh.quaternion.copy(chunk.localQuat);
    scene.add(mesh);
    chunk.mesh = mesh;
    chunkMeshes.push(mesh);
  });

  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setTranslation(0, -2, 0), groundBody);
  const ground = new THREE.Mesh(
    new THREE.BoxGeometry(40, 1, 40),
    new THREE.MeshStandardMaterial({ color: 0x1a1e2f, roughness: 0.82, metalness: 0.12 })
  );
  ground.position.set(0, -2, 0);
  ground.receiveShadow = true;
  scene.add(ground);

  const settings = runtime.defaultExtSettings();
  settings.maxSolverIterationsPerFrame = 32;
  settings.graphReductionLevel = 0;
  applyStrengthScale(settings, BASE_LIMITS, 1.0);

  const bridge = {
    world,
    scene,
    runtime,
    scenario,
    solverNodes,
    baseBonds: solverBonds,
    chunks,
    bonds,
    meshes: chunkMeshes,
    rootBodyHandle: bridgeBody.handle,
    solver: null,
    solverBondMap: [],
    settings,
    baseLimits: { ...BASE_LIMITS },
    strengthScale: 1.0,
    gravity: GRAVITY_DEFAULT,
    targetError: 1.0e-6,
    highlightSummary: [],
    overstressed: 0,
    iterationCount: 0,
    lastError: { lin: 0, ang: 0 },
    forceBoost: 1.0,
    failureDetected: false,
    projectiles: [],
    debugRenderer: new RapierDebugRenderer(scene, world, { enabled: false }),
    loadVehicle: null,
    contactInfo: { column: -1, nodes: [] }
  };

  bridge.maxBondLength = computeMaxBondLength(bridge);
  createExtSolverForBridge(bridge);

  bridge.loadVehicle = spawnLoadVehicle(world, bridge);

  pushEvent(`Bridge initialized with ${chunks.length} chunks and ${bridge.solverBondMap.length} bonds`);
  updateBondTable(bridge);
  updateOverlay(bridge, bridge.contactInfo);

  return bridge;
}

function createChunksFromScenario(scenario) {
  const chunks = [];
  const spacing = scenario.spacing;

  scenario.nodes.forEach((node, index) => {
    const coord = scenario.gridCoordinates[index];
    const isDeckLayer = coord.iy >= 0;
    const isTopLayer = coord.iy === scenario.thicknessLayers - 1;
    const kind = isDeckLayer ? (isTopLayer ? 'support-top' : 'deck') : coord.iy === -1 ? 'support-column' : 'support-base';

    let hx;
    let hy;
    let hz;
    if (isDeckLayer) {
      hx = Math.max(spacing.x * 0.45, 0.25);
      hy = Math.max(spacing.y * 0.45, 0.2);
      hz = Math.max((scenario.widthSegments > 1 ? spacing.z : scenario.parameters.deckWidth) * 0.45, 0.3);
    } else {
      hx = Math.max(spacing.x * 0.35, 0.25);
      hy = Math.max(scenario.parameters.pierHeight * 0.45, 0.8);
      hz = Math.max((scenario.widthSegments > 1 ? spacing.z : scenario.parameters.deckWidth) * 0.35, 0.35);
    }

    const chunk = new BridgeChunk({
      id: `node-${index}`,
      nodeId: index,
      shape: { type: 'box', hx, hy, hz },
      localPosition: vec3(node.centroid.x, node.centroid.y, node.centroid.z),
      localRotation: { x: 0, y: 0, z: 0, w: 1 },
      mass: node.mass,
      inertia: Math.max(node.mass * 0.18, 0.01),
      kind,
      coord
    });

    chunks.push(chunk);
  });

  return chunks;
}

function createBondsFromScenario(scenario, chunks) {
  return scenario.bonds.map((bond, index) => {
    const chunkA = chunks[bond.node0];
    const chunkB = chunks[bond.node1];
    const label = makeBondLabel(chunkA, chunkB);
    const length = bondLength(chunks, bond);
    return new BridgeBond({
      id: `bond-${bond.node0}-${bond.node1}`,
      node0: bond.node0,
      node1: bond.node1,
      area: bond.area,
      centroid: vec3(bond.centroid.x, bond.centroid.y, bond.centroid.z),
      normal: vec3(bond.normal.x, bond.normal.y, bond.normal.z),
      name: label,
      length,
      scenarioIndex: index
    });
  });
}

function makeBondLabel(chunkA, chunkB) {
  if (!chunkA || !chunkB) {
    return 'Unknown bond';
  }
  const labelA = makeChunkLabel(chunkA);
  const labelB = makeChunkLabel(chunkB);
  return `${labelA} ↔ ${labelB}`;
}

function makeChunkLabel(chunk) {
  const { coord } = chunk;
  if (!coord) {
    return chunk.id;
  }
  if (coord.iy >= 0) {
    return `Deck[${coord.ix},${coord.iz},L${coord.iy}]`;
  }
  return `Support[${coord.ix},${coord.iz}]`;
}

function bondLength(chunks, bond) {
  const a = chunks[bond.node0]?.localPosition ?? vec3();
  const b = chunks[bond.node1]?.localPosition ?? vec3();
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function computeMaxBondLength(bridge) {
  let maxLength = 0;
  bridge.bonds.forEach((bond) => {
    maxLength = Math.max(maxLength, bond.length ?? 0);
  });
  return Math.max(maxLength, 1.0);
}

function createExtSolverForBridge(bridge) {
  if (bridge.solver) {
    bridge.solver.destroy();
    bridge.solver = null;
  }

  const activeBonds = [];
  const bondMap = [];
  bridge.bonds.forEach((bond, index) => {
    if (bond.active) {
      activeBonds.push(bridge.baseBonds[bond.scenarioIndex]);
      bondMap.push(index);
    }
  });

  if (activeBonds.length === 0) {
    bridge.solverBondMap = [];
    return;
  }

  bridge.solver = bridge.runtime.createExtSolver({
    nodes: bridge.solverNodes,
    bonds: activeBonds,
    settings: bridge.settings
  });
  bridge.solverBondMap = bondMap;
}

function updateBridge(bridge, delta) {
  advanceCar(bridge, delta);
  driveStressSolver(bridge);
  syncMeshes(bridge.world, bridge);
  updateProjectiles(bridge.world, bridge, delta);
  updateLoadVehicle(bridge.world, bridge, delta);
  updateBondTable(bridge);
  updateOverlay(bridge, bridge.contactInfo ?? { column: -1, nodes: [] });

  if (bridge.forceBoost > 1.0) {
    bridge.forceBoost = THREE.MathUtils.lerp(bridge.forceBoost, 1.0, Math.min(delta * 0.8, 1.0));
  }
}

function advanceCar(bridge, delta) {
  const vehicle = bridge.loadVehicle;
  if (!vehicle) {
    return;
  }
  const body = bridge.world.getRigidBody(vehicle.bodyHandle);
  if (!body) {
    return;
  }
  const translation = body.translation();
  const maxX = bridge.scenario.origins.x + bridge.scenario.spacing.x * (bridge.scenario.spanSegments - 1);
  const minX = bridge.scenario.origins.x;
  if (translation.x > maxX + vehicle.margin) {
    body.setTranslation({ x: maxX + vehicle.margin, y: translation.y, z: translation.z }, true);
    const velocity = body.linvel();
    body.setLinvel({ x: -Math.abs(velocity.x), y: velocity.y, z: velocity.z }, true);
  } else if (translation.x < minX - vehicle.margin) {
    body.setTranslation({ x: minX - vehicle.margin, y: translation.y, z: translation.z }, true);
    const velocity = body.linvel();
    body.setLinvel({ x: Math.abs(velocity.x), y: velocity.y, z: velocity.z }, true);
  }
}

function driveStressSolver(bridge) {
  const solver = bridge.solver;
  if (!solver) {
      return;
    }

  solver.reset();
  solver.addGravity(vec3(0.0, bridge.gravity, 0.0));

  const contact = computeCarContact(bridge);
  bridge.contactInfo = contact;
  applyCarLoads(bridge, contact);

  let iterations = 0;
  let error = { lin: 0, ang: 0 };
  const maxIterations = bridge.settings.maxSolverIterationsPerFrame;
  for (; iterations < maxIterations; ++iterations) {
    solver.update();
    error = solver.stressError();
    if (solver.converged()) {
      break;
    }
    if (error.lin <= bridge.targetError && error.ang <= bridge.targetError) {
      break;
    }
  }

  const overstressed = solver.overstressedBondCount();
  const debugLines = solver.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 });

  bridge.iterationCount = iterations + 1;
  bridge.lastError = error;
  bridge.overstressed = overstressed;

  updateBondHighlights(bridge, debugLines, overstressed);

  if (overstressed > 0) {
    handleBondFailures(bridge);
  }
}

function computeCarContact(bridge) {
  const vehicle = bridge.loadVehicle;
  if (!vehicle) {
    return { column: -1, nodes: [] };
  }
  const body = bridge.world.getRigidBody(vehicle.bodyHandle);
    if (!body) {
    return { column: -1, nodes: [] };
  }
  const translation = body.translation();
  const minX = bridge.scenario.origins.x;
  const maxX = bridge.scenario.origins.x + bridge.scenario.spacing.x * (bridge.scenario.spanSegments - 1);
  const deckRange = maxX - minX;
  if (deckRange <= 0) {
    return { column: -1, nodes: [] };
  }
  const progress = THREE.MathUtils.clamp((translation.x - minX) / deckRange, 0, 0.999);
  const segments = bridge.scenario.spanSegments;
  const column = Math.min(Math.floor(progress * (segments - 1)), segments - 2);
  const nodes = [
    ...bridge.scenario.topColumnNodes[column],
    ...bridge.scenario.topColumnNodes[column + 1]
  ];
  return { column, nodes };
}

function applyCarLoads(bridge, contact) {
  const nodes = contact.nodes ?? [];
  if (nodes.length === 0) {
      return;
    }

  const vehicle = bridge.loadVehicle;
  const body = bridge.world.getRigidBody(vehicle.bodyHandle);
  const linvel = body?.linvel();
  const speed = body ? Math.abs(linvel.x) : 0;
  const dynamicBoost = 1.0 + Math.min(speed * 0.15, 2.0) + (bridge.forceBoost - 1.0);

  const baseForce = vehicle.mass * Math.abs(bridge.gravity);
  const perNode = baseForce / nodes.length;

  nodes.forEach((nodeIndex, idx) => {
    const multiplier = idx < nodes.length / 2 ? 1.0 : IMPACT_MULTIPLIER;
    const force = perNode * multiplier * dynamicBoost;
    bridge.solver.addForce(nodeIndex, vec3(), vec3(0.0, -force, 0.0));
  });
}

function updateBondHighlights(bridge, debugLines, overstressed) {
  const highlights = [];
  const severityByBond = new Map();
  const chunkSeverity = new Array(bridge.chunks.length).fill(0);
  const activeCount = Math.min(bridge.solverBondMap.length, debugLines.length);

  for (let i = 0; i < activeCount; ++i) {
    const bondIndex = bridge.solverBondMap[i];
    const line = debugLines[i];
    if (!line) {
      continue;
    }
    const length = lineMagnitude(line);
    const severity = Math.min(length / bridge.maxBondLength, 1.0);
    severityByBond.set(bondIndex, severity);

    const center = {
      x: (line.p0.x + line.p1.x) * 0.5,
      y: (line.p0.y + line.p1.y) * 0.5,
      z: (line.p0.z + line.p1.z) * 0.5
    };

    highlights.push({
      bondIndex,
      severity,
      length,
      center,
      color: toColorString(line.color0),
      line
    });
  }

  highlights.sort((a, b) => b.severity - a.severity);
  const overstressedCount = Math.min(overstressed, highlights.length);
  for (let i = 0; i < overstressedCount; ++i) {
    highlights[i].overstressed = true;
  }

  bridge.chunks.forEach((chunk) => {
    chunk.severity = createSeverity();
  });

  bridge.bonds.forEach((bond, index) => {
    const severity = severityByBond.get(index) ?? 0;
    bond.severity.max = severity;
    bond.severity.compression = severity;
    bond.severity.tension = severity;
    bond.severity.shear = severity;
    if (severity > 0) {
      chunkSeverity[bond.node0] = Math.max(chunkSeverity[bond.node0], severity);
      chunkSeverity[bond.node1] = Math.max(chunkSeverity[bond.node1], severity);
    }
  });

  chunkSeverity.forEach((value, nodeIndex) => {
    const chunk = bridge.chunks[nodeIndex];
    if (chunk) {
      chunk.severity.max = value;
      chunk.severity.compression = value;
      chunk.severity.tension = value;
      chunk.severity.shear = value;
    }
  });

  bridge.highlightSummary = highlights;
}

function handleBondFailures(bridge) {
  const failing = bridge.highlightSummary.filter(
    (item) => item.overstressed && bridge.bonds[item.bondIndex]?.active
  );
  const candidates = failing.length > 0 ? failing : bridge.highlightSummary.slice(0, 1);

  candidates.forEach((item) => {
    const bond = bridge.bonds[item.bondIndex];
    if (!bond || !bond.active) {
      return;
    }
    pushEvent(`${bond.name} failed at severity ${(item.severity * 100).toFixed(1)}%`);
    fractureBond(bridge.world, bridge, item.bondIndex, bond, 'stress solver');
    bond.active = false;
    bridge.failureDetected = true;
  });

  if (candidates.length > 0) {
    createExtSolverForBridge(bridge);
  }
}

function syncMeshes(world, bridge) {
  const tempColor = new THREE.Color();
  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh || !chunk.active) {
      return;
    }
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return;
    }
    const position = body.translation();
    const rotation = body.rotation();
    chunk.mesh.position.set(position.x, position.y, position.z);
    chunk.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    chunk.mesh.position.add(chunk.localOffset.clone().applyQuaternion(chunk.mesh.quaternion));

    const severity = chunk.severity?.max ?? 0;
    tempColor.setHSL(Math.max(0.02, 0.62 - severity * 0.5), 0.75, 0.55);
    chunk.mesh.material.color.lerp(tempColor, 0.25);
  });
}

function updateLoadVehicle(world, bridge, delta) {
  const vehicle = bridge.loadVehicle;
  if (!vehicle) {
    return;
  }
  const body = world.getRigidBody(vehicle.bodyHandle);
  if (!body) {
    vehicle.mesh.removeFromParent();
    bridge.loadVehicle = undefined;
    return;
  }
  const translation = body.translation();
  vehicle.mesh.position.set(translation.x, translation.y, translation.z);
  const rotation = body.rotation();
  vehicle.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
}

function updateBondTable(bridge) {
  if (!HUD.bondTable) {
    return;
  }
  HUD.bondTable.innerHTML = '';

  if (!bridge.highlightSummary || bridge.highlightSummary.length === 0) {
      const row = document.createElement('div');
      row.className = 'bond-row';
    row.textContent = 'No stressed bonds reported yet';
    HUD.bondTable.appendChild(row);
    return;
  }

  bridge.highlightSummary.slice(0, 6).forEach((item, index) => {
    const bond = bridge.bonds[item.bondIndex];
    const row = document.createElement('div');
    row.className = 'bond-row';

      const title = document.createElement('div');
    title.textContent = bond ? bond.name : `Line ${index + 1}`;
      row.appendChild(title);

      const value = document.createElement('div');
    value.textContent = `${(item.severity * 100).toFixed(0)}%`;
      row.appendChild(value);

    const info = document.createElement('div');
    info.className = 'bond-info';
    info.textContent = `center (${formatNumber(item.center.x, 6, 2)}, ${formatNumber(
      item.center.y,
      6,
      2
    )}, ${formatNumber(item.center.z, 6, 2)})`;
    row.appendChild(info);

    const colorLabel = document.createElement('div');
    colorLabel.className = 'bond-color';
    colorLabel.style.backgroundColor = item.color;
    row.appendChild(colorLabel);

      HUD.bondTable.appendChild(row);
    });
}

function updateOverlay(bridge, contact) {
  if (!HUD.overlay) {
    return;
  }
  const contactLabel = contact.column >= 0 ? `${contact.column} – ${contact.column + 1}` : 'off-deck';
  HUD.overlay.innerHTML = `
    <div>Car span: ${contactLabel}</div>
    <div>Iterations: ${bridge.iterationCount}/${bridge.settings.maxSolverIterationsPerFrame}</div>
    <div>Error lin ${formatNumber(bridge.lastError.lin, 7, 4)} | ang ${formatNumber(bridge.lastError.ang, 7, 4)}</div>
    <div>Overstressed bonds: ${bridge.overstressed}</div>
    <div>Force multiplier: ${bridge.forceBoost.toFixed(2)}x</div>
    <div>Status: ${bridge.failureDetected ? 'FAILED' : 'Stable'}</div>
  `;
}

function pushEvent(message) {
  if (!HUD.eventLog) {
    return;
  }
  const item = document.createElement('li');
  item.textContent = message;
  HUD.eventLog.prepend(item);
  while (HUD.eventLog.children.length > MAX_EVENTS) {
    HUD.eventLog.removeChild(HUD.eventLog.lastChild);
  }
}

function setupControls(bridge) {
  if (controlsUI.gravitySlider) {
    controlsUI.gravitySlider.value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      bridge.gravity = Number.isFinite(value) ? value : GRAVITY_DEFAULT;
      bridge.world.gravity = new RAPIER.Vector3(0, bridge.gravity, 0);
      if (HUD.gravityValue) {
        HUD.gravityValue.textContent = bridge.gravity.toFixed(2);
      }
      pushEvent(`Gravity set to ${bridge.gravity.toFixed(2)} m/s²`);
    });
    if (HUD.gravityValue) {
      HUD.gravityValue.textContent = GRAVITY_DEFAULT.toFixed(2);
    }
  }

  if (controlsUI.strengthSlider) {
    controlsUI.strengthSlider.value = bridge.strengthScale.toString();
    controlsUI.strengthSlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      bridge.strengthScale = Number.isFinite(value) ? value : 1.0;
      applyStrengthScale(bridge.settings, bridge.baseLimits, bridge.strengthScale);
      bridge.solver.setSettings(bridge.settings);
      if (HUD.strengthValue) {
        HUD.strengthValue.textContent = `${(bridge.strengthScale * 100).toFixed(1)}%`;
      }
      pushEvent(`Material strength scaled to ${(bridge.strengthScale * 100).toFixed(1)}%`);
    });
    if (HUD.strengthValue) {
      HUD.strengthValue.textContent = '100.0%';
    }
  }

  if (controlsUI.iterSlider) {
    controlsUI.iterSlider.value = bridge.settings.maxSolverIterationsPerFrame.toString();
    controlsUI.iterSlider.addEventListener('input', (event) => {
      const value = parseInt(event.target.value, 10);
      bridge.settings.maxSolverIterationsPerFrame = Number.isFinite(value) ? value : 32;
      bridge.solver.setSettings(bridge.settings);
      if (HUD.iterValue) {
        HUD.iterValue.textContent = bridge.settings.maxSolverIterationsPerFrame.toString();
      }
    });
    if (HUD.iterValue) {
      HUD.iterValue.textContent = bridge.settings.maxSolverIterationsPerFrame.toString();
    }
  }

  if (controlsUI.toleranceSlider) {
    const initial = Math.log10(bridge.targetError);
    controlsUI.toleranceSlider.value = initial.toString();
    controlsUI.toleranceSlider.addEventListener('input', (event) => {
      const exponent = parseFloat(event.target.value);
      bridge.targetError = Math.pow(10, exponent);
      if (HUD.toleranceValue) {
        HUD.toleranceValue.textContent = formatTolerance(bridge.targetError);
      }
    });
    if (HUD.toleranceValue) {
      HUD.toleranceValue.textContent = formatTolerance(bridge.targetError);
    }
  }

  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => {
      spawnProjectile(bridge.world, bridge);
    });
  }

  if (controlsUI.resetButton) {
    controlsUI.resetButton.addEventListener('click', () => {
      window.location.reload();
    });
  }

  if (controlsUI.debugToggle && bridge.debugRenderer) {
    controlsUI.debugToggle.addEventListener('click', () => {
      const enabled = bridge.debugRenderer.toggle();
      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
    });
    controlsUI.debugToggle.textContent = 'Hide Debug';
  }
}

function spawnLoadVehicle(world, bridge) {
  const props = {
    mass: 5200,
    length: 3.6,
    width: 1.8,
    height: 1.2,
    speed: 4.5
  };
  const startX = bridge.scenario.origins.x - bridge.scenario.spacing.x;
  const deckTop = bridge.scenario.spacing.y * (bridge.scenario.thicknessLayers * 0.5);
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(startX, deckTop + props.height * 0.5 + 0.05, 0)
    .setLinearDamping(0.4)
    .setAngularDamping(1.0);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(props.length * 0.5, props.height * 0.5, props.width * 0.5).setMass(props.mass),
    body
  );
  const material = new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.6 });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(props.length, props.height, props.width), material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  bridge.scene.add(mesh);
  body.setLinvel({ x: props.speed, y: 0, z: 0 }, true);

  return {
    bodyHandle: body.handle,
    colliderHandle: collider.handle,
    mesh,
    mass: props.mass,
    margin: props.length * 0.5
  };
}

function spawnProjectile(world, bridge) {
  const origin = new THREE.Vector3(-20, 4, 0);
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(origin.x, origin.y, origin.z)
    .setCanSleep(false);
  const projectileBody = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.6), projectileBody);
  projectileBody.setLinvel({ x: PROJECTILE_SPEED, y: -1, z: (Math.random() - 0.5) * 5 }, true);

  const projectileMaterial = new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x552211 });
  const projectileMesh = new THREE.Mesh(new THREE.SphereGeometry(0.6, 20, 20), projectileMaterial);
  projectileMesh.castShadow = true;
  projectileMesh.receiveShadow = true;
  projectileMesh.position.copy(origin);
  bridge.scene.add(projectileMesh);

  bridge.projectiles.push({
    bodyHandle: projectileBody.handle,
    colliderHandle: collider.handle,
    mesh: projectileMesh,
    ttl: 12
  });
  pushEvent('Projectile fired at bridge');
}

function updateProjectiles(world, bridge, delta) {
  const remaining = [];
  bridge.projectiles.forEach((projectile) => {
    const body = world.getRigidBody(projectile.bodyHandle);
    if (!body) {
      projectile.mesh?.removeFromParent();
      return;
    }
    projectile.ttl -= delta;
    if (projectile.ttl <= 0 || body.translation().y < -10) {
      world.removeRigidBody(body);
      projectile.mesh?.removeFromParent();
      return;
    }
    const translation = body.translation();
    projectile.mesh.position.set(translation.x, translation.y, translation.z);
    const rotation = body.rotation();
    projectile.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    remaining.push(projectile);
  });
  bridge.projectiles = remaining;
}

function applyStrengthScale(settings, baseLimits, scale) {
  settings.compressionElasticLimit = baseLimits.compressionElasticLimit * scale;
  settings.compressionFatalLimit = baseLimits.compressionFatalLimit * scale;
  settings.tensionElasticLimit = baseLimits.tensionElasticLimit * scale;
  settings.tensionFatalLimit = baseLimits.tensionFatalLimit * scale;
  settings.shearElasticLimit = baseLimits.shearElasticLimit * scale;
  settings.shearFatalLimit = baseLimits.shearFatalLimit * scale;
}

function createColliderForChunk(chunk) {
  switch (chunk.shape.type) {
    case 'box':
      return RAPIER.ColliderDesc.cuboid(chunk.shape.hx, chunk.shape.hy, chunk.shape.hz);
    case 'capsule':
      return RAPIER.ColliderDesc.capsule(chunk.shape.halfHeight, chunk.shape.radius);
    default:
      return RAPIER.ColliderDesc.ball(0.6);
  }
}

function createMeshForChunk(chunk, baseMaterial) {
  const material = baseMaterial.clone();
  let geometry;
  switch (chunk.shape.type) {
    case 'box':
      geometry = new THREE.BoxGeometry(chunk.shape.hx * 2, chunk.shape.hy * 2, chunk.shape.hz * 2);
      break;
    case 'capsule':
      geometry = new THREE.CapsuleGeometry(chunk.shape.radius, chunk.shape.halfHeight * 2, 18, 36);
      break;
    default:
      geometry = new THREE.SphereGeometry(0.6, 20, 20);
  }
  return new THREE.Mesh(geometry, material);
}

function toColorString(value) {
  const hex = (value & 0xffffff).toString(16).padStart(6, '0');
  return `#${hex}`;
}

function lineMagnitude(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function formatTolerance(value) {
  return Number(value).toExponential(2);
}

function fractureBond(world, bridge, bondIndex, bond, mode) {
  const chunkA = bridge.chunks.find((chunk) => chunk.nodeId === bond.node0);
  const chunkB = bridge.chunks.find((chunk) => chunk.nodeId === bond.node1);
  if (!chunkA || !chunkB) {
    return;
  }
  bond.active = false;
  pushEvent(`${bond.name} failed due to ${mode}`);
  splitChunk(world, bridge, chunkB);
}

function splitChunk(world, bridge, chunk) {
  if (!chunk.active) {
    return;
  }
  chunk.active = false;
  const parentBody = world.getRigidBody(chunk.bodyHandle);
  const collider = world.getCollider(chunk.colliderHandle);
  if (!parentBody || !collider) {
    return;
  }

  const parentTranslation = parentBody.translation();
  const parentRotation = parentBody.rotation();
  const parentQuat = new THREE.Quaternion(parentRotation.x, parentRotation.y, parentRotation.z, parentRotation.w);
  const offsetWorld = chunk.localOffset.clone().applyQuaternion(parentQuat);
  const worldPos = new THREE.Vector3(parentTranslation.x, parentTranslation.y, parentTranslation.z).add(offsetWorld);
  const worldQuat = parentQuat.clone().multiply(chunk.localQuat);

  world.removeCollider(collider, false);

  const velocity = parentBody.linvel();
  const angularVelocity = parentBody.angvel();

  const newBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(worldPos.x, worldPos.y, worldPos.z)
    .setRotation(new RAPIER.Quaternion(worldQuat.x, worldQuat.y, worldQuat.z, worldQuat.w));
  if (velocity && Number.isFinite(velocity.x) && Number.isFinite(velocity.y) && Number.isFinite(velocity.z)) {
    newBodyDesc.setLinvel(velocity.x, velocity.y, velocity.z);
  }
  if (
    angularVelocity &&
    Number.isFinite(angularVelocity.x) &&
    Number.isFinite(angularVelocity.y) &&
    Number.isFinite(angularVelocity.z)
  ) {
    newBodyDesc.setAngvel(angularVelocity.x, angularVelocity.y, angularVelocity.z);
  }
  const newBody = world.createRigidBody(newBodyDesc);

  const newColliderDesc = createColliderForChunk(chunk)
    .setTranslation(0, 0, 0)
    .setRotation(new RAPIER.Quaternion(0, 0, 0, 1));
  const newCollider = world.createCollider(newColliderDesc, newBody);

  chunk.bodyHandle = newBody.handle;
  chunk.colliderHandle = newCollider.handle;
  chunk.active = true;
  chunk.localOffset.set(0, 0, 0);
  chunk.localQuat.set(0, 0, 0, 1);
  chunk.mesh.position.set(worldPos.x, worldPos.y, worldPos.z);
  chunk.mesh.quaternion.copy(worldQuat);
}

function colliderDescFromShape(collider) {
  const shape = collider.shape;
  if (!shape) {
    return RAPIER.ColliderDesc.ball(0.5);
  }
  switch (shape.type) {
    case 'cuboid': {
      const halfExtents = shape.halfExtents;
      return RAPIER.ColliderDesc.cuboid(halfExtents.x, halfExtents.y, halfExtents.z);
    }
    case 'ball':
      return RAPIER.ColliderDesc.ball(shape.radius ?? 0.5);
    case 'capsule':
      return RAPIER.ColliderDesc.capsule(shape.halfHeight ?? 0.5, shape.radius ?? 0.25);
    default:
      return RAPIER.ColliderDesc.ball(0.5);
  }
}

init().catch((err) => {
  console.error(err);
});
