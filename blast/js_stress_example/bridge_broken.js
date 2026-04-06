function fractureBond(world, bridge, bondIndex, bond, meta, mode) {
  const chunkA = bridge.chunks[bond.node0];
  const chunkB = bridge.chunks[bond.node1];
  if (!chunkA || !chunkB) {
    return;
  }
  if (meta) {
    pushEvent(`${meta.name ?? `${bond.node0}→${bond.node1}`} failed due to ${mode}`);
  }
  bridge.stressProcessor.removeBond(bondIndex);
  if (meta) {
    bridge.bonds.splice(bondIndex, 1);
  }
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

  world.removeCollider(collider, false);

  const translation = parentBody.translation();
  const rotation = parentBody.rotation();
  const velocity = parentBody.linvel();
  const angularVelocity = parentBody.angvel();

  const newBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(translation.x, translation.y, translation.z)
    .setRotation(rotation)
    .setCanSleep(false);
  const newBody = world.createRigidBody(newBodyDesc);
  newBody.setLinvel({ x: velocity.x, y: velocity.y, z: velocity.z }, true);
  newBody.setAngvel({ x: angularVelocity.x, y: angularVelocity.y, z: angularVelocity.z }, true);

  const newColliderDesc = createColliderForChunk(chunk, { global: true });
  const newCollider = world.createCollider(newColliderDesc, newBody);
  chunk.bodyHandle = newBody.handle;
  chunk.colliderHandle = newCollider.handle;
  chunk.active = true;
}
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import RapierDebugRenderer from './rapier-debug-renderer.js';

import {
  loadStressSolver,
  vec3,
  computeBondStress,
  StressLimits
} from './stress_broken.js';

const DEBUG_LOGGING = true;

const GRAVITY_DEFAULT = -9.81;
const PROJECTILE_SPEED = 35;
const MAX_EVENTS = 8;

const colorScale = new THREE.Color();

const BRIDGE_DIMENSIONS = {
  deckSegmentLength: 9,
  deckWidth: 8,
  deckThickness: 0.6,
  deckElevation: 2.2,
  pierWidth: 1.4,
  pierDepth: 2.6,
  pierHeight: 2.2
};

const MATERIAL_DENSITIES = {
  deck: 450, // kg/m^3
  pier: 600,
  base: 750
};

const VEHICLE_CONFIG = {
  mass: 2400,
  length: 3.4,
  width: 1.7,
  height: 1.2,
  speed: 2.8,
  startOffsetX: -0.3,
  startOffsetZ: 0.6,
  loopExtentMultiplier: 0.6
};

class BridgeChunk {
  constructor(desc) {
    Object.assign(this, desc);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.mesh = null;
    this.active = true;
    this.severity = createSeverity();
    this.pendingForce = { x: 0, y: 0, z: 0 };

    const baseOffset = desc.localPosition
      ? new THREE.Vector3(desc.localPosition.x, desc.localPosition.y, desc.localPosition.z)
      : new THREE.Vector3();
    const baseQuat = desc.localRotation instanceof THREE.Quaternion
      ? desc.localRotation.clone()
      : new THREE.Quaternion(
          desc.localRotation?.x ?? 0,
          desc.localRotation?.y ?? 0,
          desc.localRotation?.z ?? 0,
          desc.localRotation?.w ?? 1
        );

    this.structureOffset = baseOffset.clone();
    this.structureQuat = baseQuat.clone();
    this.localOffset = baseOffset.clone();
    this.localQuat = baseQuat.clone();
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

function createSeverity() {
  return {
    compression: 0,
    tension: 0,
    shear: 0,
    max: 0
  };
}

function computeBoxMassAndInertia(hx, hy, hz, density) {
  const volume = (hx * 2) * (hy * 2) * (hz * 2);
  const mass = density * volume;
  const ix = (1.0 / 12.0) * mass * ((hy * 2) ** 2 + (hz * 2) ** 2);
  const iy = (1.0 / 12.0) * mass * ((hx * 2) ** 2 + (hz * 2) ** 2);
  const iz = (1.0 / 12.0) * mass * ((hx * 2) ** 2 + (hy * 2) ** 2);
  const inertia = (ix + iy + iz) / 3;
  return { mass, inertia };
}

const HUD = {
  gravityValue: document.getElementById('gravity-value'),
  bondTable: document.getElementById('bond-table'),
  eventLog: document.getElementById('event-log'),
  overlay: document.getElementById('overlay'),
  strengthValue: document.getElementById('strength-value')
};

const controlsUI = {
  gravitySlider: document.getElementById('gravity-slider'),
  fireButton: document.getElementById('fire-projectile'),
  resetButton: document.getElementById('reset-bridge'),
  debugToggle: document.getElementById('toggle-debug'),
  strengthSlider: document.getElementById('strength-slider'),
  strengthValue: document.getElementById('strength-value')
};

const solverParamsUI = {
  iterationsSlider: document.getElementById('iter-slider'),
  toleranceSlider: document.getElementById('tolerance-slider'),
  iterationsValue: document.getElementById('iter-value'),
  toleranceValue: document.getElementById('tolerance-value')
};

const storageKey = (name) => `blast-bridge-${name}`;

function loadSetting(name, fallback) {
  const raw = localStorage.getItem(storageKey(name));
  if (raw === null) {
    return fallback;
  }
  const parsed = parseFloat(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function saveSetting(name, value) {
  localStorage.setItem(storageKey(name), String(value));
}

async function init() {
  await RAPIER.init();
  const stressRuntime = await loadStressSolver();

  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));
  const { scene, renderer, camera, controls } = initThree();
  const bridge = buildBridge(scene, world, stressRuntime);
  bridge.debugRenderer = new RapierDebugRenderer(scene, world, { enabled: loadSetting('debug', 1) === 1 });

  setupControls(world, bridge);

  const clock = new THREE.Clock();

  function loop() {
    const delta = clock.getDelta();
    updateBridge(world, bridge, stressRuntime, delta);
    world.step(bridge.eventQueue);
    if (bridge.debugRenderer?.enabled) {
      bridge.debugRenderer.update();
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
  }

  loop();
}

function initThree() {
  const canvas = document.getElementById('bridge-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  const initialWidth = canvas.clientWidth || window.innerWidth;
  const initialHeight = canvas.clientHeight || window.innerHeight;
  renderer.setSize(initialWidth, initialHeight, false);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x05070a);

  const camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 500);
  camera.position.set(0, 8, 18);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 0, 0);

  const ambient = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.85);
  dir.position.set(12, 18, 10);
  dir.castShadow = true;
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

function buildBridge(scene, world, stressRuntime) {
  const chunks = createBridgeChunks();
  const nodeIndexById = new Map();
  chunks.forEach((chunk, index) => {
    chunk.nodeId = index;
    nodeIndexById.set(chunk.id, index);
  });
  const bonds = createBridgeBonds(nodeIndexById);
  const bondMeta = bonds.map((bond) => ({ ...bond }));

  const stressProcessor = stressRuntime.createProcessor({
    nodes: chunks.map((chunk) => ({
      com: chunk.structureOffset ?? chunk.localOffset ?? vec3(),
      mass: chunk.mass ?? 0,
      inertia: chunk.inertia ?? 0
    })),
    bonds: bonds.map((bond) => ({
      centroid: getBondCentroid(bond, chunks),
      node0: bond.node0,
      node1: bond.node1
    })),
    dataParams: { equalizeMasses: 1, centerBonds: 1 }
  });

  const limits = scaledLimits(1.0);
  const solverParams = loadSolverParams();
  stressProcessor.setSolverParams(solverParams);

  const bridgeBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(0, 0, 0)
    .setGravityScale(0)
    .setCanSleep(false)
    .setLinearDamping(0.8)
    .setAngularDamping(1.2);
  const bridgeBody = world.createRigidBody(bridgeBodyDesc);

  const deckMaterial = new THREE.MeshStandardMaterial({ color: 0x486fe3, roughness: 0.35, metalness: 0.45 });
  const pierMaterial = new THREE.MeshStandardMaterial({ color: 0x2f3e56, roughness: 0.6, metalness: 0.25 });
  const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x1d2533, roughness: 0.75, metalness: 0.18 });

  const chunkMeshes = [];
  const colliderToChunk = new Map();
  chunks.forEach((chunk) => {
    const colliderDesc = createColliderForChunk(chunk)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0);
    const collider = world.createCollider(colliderDesc, bridgeBody);
    chunk.colliderHandle = collider.handle;
    chunk.bodyHandle = bridgeBody.handle;

    let material = deckMaterial;
    if (chunk.id.startsWith('pier-base')) {
      material = baseMaterial;
    } else if (chunk.id.startsWith('pier')) {
      material = pierMaterial;
    }
    const mesh = createMeshForChunk(chunk, material);
    chunk.mesh = mesh;
    chunkMeshes.push(mesh);
    mesh.position.copy(chunk.structureOffset);
    mesh.quaternion.copy(chunk.structureQuat);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    scene.add(mesh);
    colliderToChunk.set(collider.handle, chunk);
  });

  const groundPosition = new THREE.Vector3(0, -2, 0);
  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(20, 0.5, 20)
      .setTranslation(groundPosition.x, groundPosition.y, groundPosition.z)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0),
    groundBody
  );
  const ground = new THREE.Mesh(new THREE.BoxGeometry(40, 1, 40), new THREE.MeshStandardMaterial({
    color: 0x1a1e2f,
    roughness: 0.8,
    metalness: 0.1
  }));
  ground.position.set(groundPosition.x, groundPosition.y, groundPosition.z);
  ground.receiveShadow = true;
  scene.add(ground);

  updateBondTable(bonds);

  return {
    world,
    scene,
    body: bridgeBody,
    chunks,
    bonds: bondMeta,
    stressProcessor,
    limits,
    projectiles: [],
    splitBodies: [],
    activeGravity: GRAVITY_DEFAULT,
    eventQueue: new RAPIER.EventQueue(true),
    colliderToChunk,
    limitsScale: 1,
    solverParams
  };
}

function updateBridge(world, bridge, stressRuntime, delta) {
  const { stressProcessor, chunks, limits, eventQueue } = bridge;

  chunks.forEach((chunk) => {
    chunk.severity = createSeverity();
  });

  drainContactForces(world, bridge);

  const velocities = chunks.map((chunk) => {
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return { lin: vec3(), ang: vec3() };
    }
    const linVel = body.linvel();
    const angVel = body.angvel();
    if (DEBUG_LOGGING && (linVel.x || linVel.y || linVel.z || angVel.x || angVel.y || angVel.z)) {
      console.log('[bridge][velocity]', {
        chunkId: chunk.id,
        nodeId: chunk.nodeId,
        linVel,
        angVel
      });
    }
    return {
      lin: vec3(linVel.x, linVel.y, linVel.z),
      ang: vec3(angVel.x, angVel.y, angVel.z)
    };
  });
  // console.log('[bridge][updateBridge]', {
  //   chunks: chunks.map((chunk) => chunk.id),
  //   velocities,
  // });

  let solveResult;
  try {
    solveResult = stressProcessor.solve({ velocities, resume: false });
  } catch (err) {
    pushEvent(`Stress solver failed: ${err.message ?? err}`);
    return;
  }
  const { iterations, error, impulses } = solveResult;
  if (!Number.isFinite(iterations)) {
    pushEvent('Stress solver failed: non-finite iteration count');
    return;
  }
  const solverNodes = stressProcessor.getNodes();
  const bonds = stressProcessor.getBonds();

  bonds.forEach((bond, index) => {
    const meta = bridge.bonds[index];
    if (!bond.active) {
      return;
    }
    const stress = computeBondStress(
      stressProcessor.bondDesc(index),
      impulses[index],
      solverNodes,
      bond.area
    );
    if (DEBUG_LOGGING && (stress.compression > 0 || stress.tension > 0 || stress.shear > 0)) {
      console.log('[bridge][bondStress]', {
        bond: meta?.id ?? index,
        stress,
        impulses: impulses[index]
      });
    }
    const severity = limits.severity(stress);
    severity.max = Math.max(severity.compression, severity.tension, severity.shear);
    if (meta) {
      meta.severity = severity;
    }
    const chunkSeverityA = chunks[bond.node0].severity ?? createSeverity();
    const chunkSeverityB = chunks[bond.node1].severity ?? createSeverity();
    chunkSeverityA.max = Math.max(chunkSeverityA.max, severity.max);
    chunkSeverityA.compression = Math.max(chunkSeverityA.compression ?? 0, severity.compression);
    chunkSeverityA.tension = Math.max(chunkSeverityA.tension ?? 0, severity.tension);
    chunkSeverityA.shear = Math.max(chunkSeverityA.shear ?? 0, severity.shear);
    chunkSeverityB.max = Math.max(chunkSeverityB.max, severity.max);
    chunkSeverityB.compression = Math.max(chunkSeverityB.compression ?? 0, severity.compression);
    chunkSeverityB.tension = Math.max(chunkSeverityB.tension ?? 0, severity.tension);
    chunkSeverityB.shear = Math.max(chunkSeverityB.shear ?? 0, severity.shear);
    chunks[bond.node0].severity = chunkSeverityA;
    chunks[bond.node1].severity = chunkSeverityB;
    const mode = limits.failureMode(stress);
    if (mode) {
      fractureBond(world, bridge, index, bond, meta, mode);
    }
  });

  syncMeshes(world, bridge);
  updateProjectiles(world, bridge, delta);
  updateLoadVehicle(world, bridge, delta);
  updateBondTable(bridge.bonds.slice(0, bonds.length));
  pushEvent(`Bridge solve: iter ${iterations}, error=(${error.lin.toFixed(3)}, ${error.ang.toFixed(3)})`);
}

function drainContactForces(world, bridge) {
  const { eventQueue, stressProcessor, colliderToChunk } = bridge;
  if (!eventQueue) {
    return;
  }
  const toVec3 = (v) => vec3(v.x ?? 0, v.y ?? 0, v.z ?? 0);
  eventQueue.drainContactForceEvents((event) => {
    const collider1 = event.collider1();
    const collider2 = event.collider2();
    const chunk = colliderToChunk.get(collider1) || colliderToChunk.get(collider2);
    if (!chunk || !chunk.active) {
      if (DEBUG_LOGGING) {
        console.debug('[bridge][contact][skip]', { collider1, collider2 });
      }
      return;
    }
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return;
    }
    const dir = event.maxForceDirection();
    const magnitude = event.maxForceMagnitude();
    if (!Number.isFinite(magnitude) || magnitude === 0) {
      return;
    }
    const worldForce = new RAPIER.Vector3(dir.x * magnitude, dir.y * magnitude, dir.z * magnitude);
    const localForce = toVec3(body.worldVectorToLocal(worldForce));
    const worldPoint = event.worldContactPoint ? event.worldContactPoint() : body.translation();
    const localPoint = chunk.isRootChunk && chunk.structureOffset
      ? chunk.structureOffset
      : toVec3(body.worldPointToLocal(worldPoint));
    if (DEBUG_LOGGING) {
      console.log('[bridge][contact]', {
        collider1,
        collider2,
        chunkId: chunk.id,
        nodeId: chunk.nodeId,
        magnitude,
        direction: dir,
        localForce,
        localPoint,
        root: chunk.isRootChunk
      });
    }
    // TODO: forward structural loads once the solver supports per-node force accumulation
  });
}

function updateLoadVehicle(world, bridge, delta) {
  if (!bridge.loadVehicle) {
    return;
  }
  const vehicle = bridge.loadVehicle;
  const body = world.getRigidBody(vehicle.bodyHandle);
  if (!body) {
    vehicle.mesh.removeFromParent();
    bridge.loadVehicle = undefined;
    return;
  }
  const translation = body.translation();
  const loopExtent = BRIDGE_DIMENSIONS.deckSegmentLength * VEHICLE_CONFIG.loopExtentMultiplier;
  if (translation.x > loopExtent) {
    body.setTranslation({ x: -loopExtent, y: translation.y, z: translation.z }, true);
  }
  vehicle.mesh.position.set(translation.x, translation.y, translation.z);
  const rotation = body.rotation();
  vehicle.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
}

function syncMeshes(world, bridge) {
  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) {
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

    if (chunk.bodyHandle === bridge.body.handle && chunk.structureOffset && typeof chunk.structureOffset.clone === 'function') {
      const offset = chunk.structureOffset.clone().applyQuaternion(chunk.mesh.quaternion);
      chunk.mesh.position.add(offset);
    }

    const severity = chunk.severity?.max ?? 0;
    colorScale.setHSL(Math.max(0.05, 0.62 - severity * 0.5), 0.75, 0.55);
    chunk.mesh.material.color.lerp(colorScale, 0.18);
  });
}

function createBridgeChunks() {
  const {
    deckSegmentLength,
    deckWidth,
    deckThickness,
    deckElevation,
    pierWidth,
    pierDepth,
    pierHeight
  } = BRIDGE_DIMENSIONS;

  const deckHalfLength = deckSegmentLength * 0.5;
  const pierHalfHeight = pierHeight * 0.5;
  const pierBaseHeight = 0.6;

  const chunks = [];

  for (let i = -1; i <= 1; i++) {
    chunks.push(
      new BridgeChunk({
        id: `deck-${i + 1}`,
        shape: { type: 'box', hx: deckHalfLength, hy: deckThickness * 0.5, hz: deckWidth * 0.5 },
        localPosition: vec3(i * deckSegmentLength, deckElevation, 0),
        localRotation: { x: 0, y: 0, z: 0, w: 1 },
        density: MATERIAL_DENSITIES.deck
      })
    );
  }

  const pierOffsets = [-deckSegmentLength, 0, deckSegmentLength];
  pierOffsets.forEach((offset, index) => {
    const pierId = index + 5;
    chunks.push(
      new BridgeChunk({
        id: `pier-${index}`,
        shape: { type: 'box', hx: pierWidth * 0.5, hy: pierHeight * 0.5, hz: pierDepth * 0.5 },
        localPosition: vec3(offset, pierHalfHeight, 0),
        localRotation: { x: 0, y: 0, z: 0, w: 1 },
        density: MATERIAL_DENSITIES.pier
      })
    );
    chunks.push(
      new BridgeChunk({
        id: `pier-base-${index}`,
        shape: { type: 'box', hx: pierWidth * 0.8, hy: pierBaseHeight * 0.5, hz: pierDepth * 0.8 },
        localPosition: vec3(offset, pierBaseHeight * 0.5, 0),
        localRotation: { x: 0, y: 0, z: 0, w: 1 },
        density: MATERIAL_DENSITIES.base
      })
    );
  });

  chunks.forEach((chunk) => {
    if (chunk.shape.type === 'box') {
      const { mass, inertia } = computeBoxMassAndInertia(chunk.shape.hx, chunk.shape.hy, chunk.shape.hz, chunk.density ?? 500);
      chunk.mass = mass;
      chunk.inertia = inertia;
    }
  });

  return chunks;
}

function createBridgeBonds(nodeIndexById) {
  const deckSpanArea = 3.5;
  const deckPierArea = 5.5;
  const pierBaseArea = 6.5;

  return [
    new BridgeBond({
      id: 'deck-0-1',
      node0: nodeIndexById.get('deck-0'),
      node1: nodeIndexById.get('deck-1'),
      area: deckSpanArea,
      name: 'Deck west → Deck mid'
    }),
    new BridgeBond({
      id: 'deck-1-2',
      node0: nodeIndexById.get('deck-1'),
      node1: nodeIndexById.get('deck-2'),
      area: deckSpanArea,
      name: 'Deck mid → Deck east'
    }),
    new BridgeBond({
      id: 'deck-west-pier-west',
      node0: nodeIndexById.get('deck-0'),
      node1: nodeIndexById.get('pier-0'),
      area: deckPierArea,
      name: 'Deck west → Pier west'
    }),
    new BridgeBond({
      id: 'deck-mid-pier-west',
      node0: nodeIndexById.get('deck-1'),
      node1: nodeIndexById.get('pier-0'),
      area: deckPierArea * 0.8,
      name: 'Deck mid → Pier west'
    }),
    new BridgeBond({
      id: 'deck-mid-pier-mid',
      node0: nodeIndexById.get('deck-1'),
      node1: nodeIndexById.get('pier-1'),
      area: deckPierArea,
      name: 'Deck mid → Pier mid'
    }),
    new BridgeBond({
      id: 'deck-mid-pier-east',
      node0: nodeIndexById.get('deck-1'),
      node1: nodeIndexById.get('pier-2'),
      area: deckPierArea * 0.8,
      name: 'Deck mid → Pier east'
    }),
    new BridgeBond({
      id: 'deck-east-pier-east',
      node0: nodeIndexById.get('deck-2'),
      node1: nodeIndexById.get('pier-2'),
      area: deckPierArea,
      name: 'Deck east → Pier east'
    }),
    new BridgeBond({
      id: 'pier-west-base',
      node0: nodeIndexById.get('pier-0'),
      node1: nodeIndexById.get('pier-base-0'),
      area: pierBaseArea,
      name: 'Pier west → Base west'
    }),
    new BridgeBond({
      id: 'pier-mid-base',
      node0: nodeIndexById.get('pier-1'),
      node1: nodeIndexById.get('pier-base-1'),
      area: pierBaseArea,
      name: 'Pier mid → Base mid'
    }),
    new BridgeBond({
      id: 'pier-east-base',
      node0: nodeIndexById.get('pier-2'),
      node1: nodeIndexById.get('pier-base-2'),
      area: pierBaseArea,
      name: 'Pier east → Base east'
    })
  ];
}

function getBondCentroid(bond, chunks) {
  const chunkA = chunks.find((c) => c.nodeId === bond.node0);
  const chunkB = chunks.find((c) => c.nodeId === bond.node1);
  if (!chunkA || !chunkB) {
    return vec3(0, 0, 0);
  }
  return vec3(
    (chunkA.localOffset.x + chunkB.localOffset.x) * 0.5,
    (chunkA.localOffset.y + chunkB.localOffset.y) * 0.5,
    (chunkA.localOffset.z + chunkB.localOffset.z) * 0.5
  );
}

function createColliderForChunk(chunk, { global = false } = {}) {
  const rotation = chunk.structureQuat ?? new THREE.Quaternion();
  const offset = chunk.structureOffset ?? new THREE.Vector3();
  if (global) {
    const body = chunk.bodyHandle ? chunk.world.getRigidBody(chunk.bodyHandle) : null;
    if (body) {
      const translation = body.translation();
      const quat = body.rotation();
      const offsetWorld = offset.clone().applyQuaternion(new THREE.Quaternion(quat.x, quat.y, quat.z, quat.w));
      chunk.localOffset = new THREE.Vector3(translation.x, translation.y, translation.z).add(offsetWorld);
      chunk.localQuat = new THREE.Quaternion(quat.x, quat.y, quat.z, quat.w).multiply(rotation);
    }
  } else {
    chunk.localOffset = offset.clone();
    chunk.localQuat = rotation.clone();
  }

  const rot = chunk.localQuat;
  const pos = chunk.localOffset;
  switch (chunk.shape.type) {
    case 'box':
      return RAPIER.ColliderDesc.cuboid(chunk.shape.hx, chunk.shape.hy, chunk.shape.hz)
        .setTranslation(pos.x, pos.y, pos.z)
        .setRotation(new RAPIER.Quaternion(rot.x, rot.y, rot.z, rot.w));
    case 'capsule':
      return RAPIER.ColliderDesc.capsule(chunk.shape.halfHeight, chunk.shape.radius)
        .setTranslation(pos.x, pos.y, pos.z)
        .setRotation(new RAPIER.Quaternion(rot.x, rot.y, rot.z, rot.w));
    default:
      return RAPIER.ColliderDesc.ball(1).setTranslation(pos.x, pos.y, pos.z);
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

function updateBondTable(bonds) {
  if (!HUD.bondTable) {
    return;
  }
  HUD.bondTable.innerHTML = '';
  bonds.forEach((bond) => {
    if (!bond) {
      return;
    }
    const row = document.createElement('div');
    row.className = 'bond-row';
    const title = document.createElement('div');
    title.textContent = bond.name ?? `${bond.node0} → ${bond.node1}`;
    row.appendChild(title);
    const value = document.createElement('div');
    value.textContent = `${(bond.severity.max * 100).toFixed(0)}%`;
    row.appendChild(value);
    const bar = document.createElement('div');
    bar.className = 'bars';
    const fill = document.createElement('span');
    fill.style.width = `${Math.min(1, bond.severity.max) * 100}%`;
    bar.appendChild(fill);
    row.appendChild(bar);
    HUD.bondTable.appendChild(row);
  });
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

function setupControls(world, bridge) {
  if (controlsUI.gravitySlider) {
    const gravityValue = loadSetting('gravity', GRAVITY_DEFAULT);
    controlsUI.gravitySlider.value = gravityValue.toString();
    world.gravity = new RAPIER.Vector3(0, gravityValue, 0);
    bridge.activeGravity = gravityValue;
    if (HUD.gravityValue) {
      HUD.gravityValue.textContent = gravityValue.toFixed(2);
    }
    controlsUI.gravitySlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      world.gravity = new RAPIER.Vector3(0, value, 0);
      bridge.activeGravity = value;
      saveSetting('gravity', value);
      if (HUD.gravityValue) {
        HUD.gravityValue.textContent = value.toFixed(2);
      }
      pushEvent(`Gravity set to ${value.toFixed(2)} m/s²`);
    });
  }
  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => {
      spawnProjectile(world, bridge);
    });
  }
  if (controlsUI.resetButton) {
    controlsUI.resetButton.addEventListener('click', () => {
      window.location.reload();
    });
  }
  if (controlsUI.debugToggle && bridge.debugRenderer) {
    const initial = bridge.debugRenderer.enabled;
    controlsUI.debugToggle.textContent = initial ? 'Hide Debug' : 'Show Debug';
    controlsUI.debugToggle.addEventListener('click', () => {
      const enabled = bridge.debugRenderer.toggle();
      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
      saveSetting('debug', enabled ? 1 : 0);
    });
  }
  if (controlsUI.strengthSlider) {
    const storedStrength = loadSetting('strength', 1);
    controlsUI.strengthSlider.value = storedStrength.toString();
    setMaterialStrength(bridge, storedStrength);
    if (controlsUI.strengthValue) {
      controlsUI.strengthValue.textContent = `${Math.round(storedStrength * 100)}%`;
    }
    controlsUI.strengthSlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      setMaterialStrength(bridge, value);
      if (controlsUI.strengthValue) {
        controlsUI.strengthValue.textContent = `${Math.round(value * 100)}%`;
      }
      saveSetting('strength', value);
      pushEvent(`Material strength set to ${Math.round(value * 100)}%`);
    });
  }
  setupSolverControls(bridge);
  spawnLoadVehicle(world, bridge);
}

function setupSolverControls(bridge) {
  const params = bridge.solverParams;
  if (solverParamsUI.iterationsSlider) {
    solverParamsUI.iterationsSlider.value = params.maxIterations.toString();
    solverParamsUI.iterationsValue.textContent = params.maxIterations.toString();
    solverParamsUI.iterationsSlider.addEventListener('input', (event) => {
      const value = Math.max(1, Math.round(parseFloat(event.target.value)));
      solverParamsUI.iterationsValue.textContent = value.toString();
      bridge.solverParams.maxIterations = value;
      bridge.stressProcessor.setSolverParams(bridge.solverParams);
      saveSetting('iterations', value);
      pushEvent(`Solver iterations set to ${value}`);
    });
  }
  if (solverParamsUI.toleranceSlider) {
    solverParamsUI.toleranceSlider.value = params.tolerance.toExponential(2);
    solverParamsUI.toleranceValue.textContent = params.tolerance.toExponential(2);
    solverParamsUI.toleranceSlider.addEventListener('input', (event) => {
      const exponent = parseFloat(event.target.value);
      const tolerance = Math.pow(10, exponent);
      solverParamsUI.toleranceValue.textContent = tolerance.toExponential(2);
      bridge.solverParams.tolerance = tolerance;
      bridge.stressProcessor.setSolverParams(bridge.solverParams);
      saveSetting('tolerance', tolerance);
      pushEvent(`Solver tolerance set to ${tolerance.toExponential(2)}`);
    });
  }
}

function spawnProjectile(world, bridge) {
  const origin = new THREE.Vector3(-20, 4, 0);
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(origin.x, origin.y, origin.z)
    .setCanSleep(false);
  const projectileBody = world.createRigidBody(bodyDesc);
  const colliderDesc = RAPIER.ColliderDesc.ball(0.6)
    .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
    .setContactForceEventThreshold(0);
  const collider = world.createCollider(colliderDesc, projectileBody);
  projectileBody.setLinvel({ x: PROJECTILE_SPEED, y: -1, z: (Math.random() - 0.5) * 5 }, true);

  // Position the projectile above the center of the bridge with a little randomness, but ensure it still hits the bridge
  const maxOffsetX = BRIDGE_DIMENSIONS.deckSegmentLength * 0.4; // stay well within bridge span
  const maxOffsetZ = BRIDGE_DIMENSIONS.deckWidth * 0.4;
  const bridgeMidX = (Math.random() - 0.5) * 2 * maxOffsetX;
  const bridgeMidZ = (Math.random() - 0.5) * 2 * maxOffsetZ;
  const bridgeMidY = BRIDGE_DIMENSIONS.deckElevation + 6; // 6 units above deck
  origin.set(bridgeMidX, bridgeMidY, bridgeMidZ);
  projectileBody.setTranslation({ x: bridgeMidX, y: bridgeMidY, z: bridgeMidZ }, true);
  // Add a tiny bit of horizontal velocity so it's not perfectly vertical, but still hits the bridge
  const xVel = (Math.random() - 0.5) * 2; // up to +/-1 unit/sec
  const zVel = (Math.random() - 0.5) * 2;
  projectileBody.setLinvel({ x: xVel, y: -PROJECTILE_SPEED, z: zVel }, true);


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

function spawnLoadVehicle(world, bridge) {
  const { length, width, height, mass, speed, startOffsetX, startOffsetZ } = VEHICLE_CONFIG;
  const startX = BRIDGE_DIMENSIONS.deckSegmentLength * startOffsetX;
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(startX, BRIDGE_DIMENSIONS.deckElevation + height * 0.5 + 0.05, startOffsetZ)
    .setLinearDamping(0.35)
    .setAngularDamping(1.0);
  const body = world.createRigidBody(bodyDesc);
  const colliderDesc = RAPIER.ColliderDesc.cuboid(length * 0.5, height * 0.5, width * 0.5)
    .setMass(mass)
    .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
    .setContactForceEventThreshold(0);
  const collider = world.createCollider(colliderDesc, body);
  const material = new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.6 });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(length, height, width), material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  bridge.scene.add(mesh);
  bridge.loadVehicle = { bodyHandle: body.handle, colliderHandle: collider.handle, mesh, speed };
  body.setLinvel({ x: speed, y: 0, z: 0 }, true);
}

function updateProjectiles(world, bridge, delta) {
  const remaining = [];
  bridge.projectiles.forEach((projectile) => {
    const body = world.getRigidBody(projectile.bodyHandle);
    if (!body) {
      if (projectile.mesh) {
        projectile.mesh.removeFromParent();
      }
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

function resetChunkTransform(chunk) {
  chunk.localOffset.copy(chunk.originalOffset);
  chunk.localQuat.copy(chunk.originalQuat);
  if (chunk.mesh) {
    chunk.mesh.position.copy(chunk.originalOffset);
    chunk.mesh.quaternion.copy(chunk.originalQuat);
  }
}

const DEFAULT_LIMITS = {
  compressionElasticLimit: 3.5e5,
  compressionFatalLimit: 6.5e5,
  tensionElasticLimit: 2.8e5,
  tensionFatalLimit: 4.5e5,
  shearElasticLimit: 1.8e5,
  shearFatalLimit: 3.5e5
};

const DEFAULT_SOLVER_PARAMS = {
  maxIterations: 32,
  tolerance: 1.0e-6,
  warmStart: false
};

function scaledLimits(scale) {
  const s = scale;
  return new StressLimits(DEFAULT_LIMITS).scaledBy(s);
}

function setMaterialStrength(bridge, factor) {
  const clamped = Math.max(0, factor);
  bridge.limitsScale = factor;
  bridge.limits = scaledLimits(clamped);
  pushEvent(`Limits updated: ×${clamped.toFixed(2)}`);
  updateLegendThresholds(bridge.limits);
}

function updateLegendThresholds(limits) {
  const overlay = HUD.overlay;
  if (!overlay) {
    return;
  }
  const lines = [
    `Compression fatal: ${(limits.compressionFatalThreshold() / 1e5).toFixed(2)}e5 Pa`,
    `Tension fatal: ${(limits.tensionFatalThreshold() / 1e5).toFixed(2)}e5 Pa`,
    `Shear fatal: ${(limits.shearFatalThreshold() / 1e5).toFixed(2)}e5 Pa`
  ];
  overlay.innerHTML = lines.map((line) => `<div>${line}</div>`).join('');
}

function loadSolverParams() {
  return {
    maxIterations: loadSetting('iterations', DEFAULT_SOLVER_PARAMS.maxIterations),
    tolerance: loadSetting('tolerance', DEFAULT_SOLVER_PARAMS.tolerance),
    warmStart: DEFAULT_SOLVER_PARAMS.warmStart
  };
}

init().catch((err) => {
  console.error('Failed to initialize bridge demo', err);
});
