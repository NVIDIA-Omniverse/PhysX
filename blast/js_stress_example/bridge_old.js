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

  const colliderShape = collider.shape;
  world.removeCollider(collider, false);

  const translation = parentBody.translation();
  const rotation = parentBody.rotation();
  const velocity = parentBody.linvel();
  const angularVelocity = parentBody.angvel();

  const newBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(translation.x, translation.y, translation.z)
    .setRotation(rotation)
    .setLinearVelocity(velocity)
    .setAngularVelocity(angularVelocity);
  const newBody = world.createRigidBody(newBodyDesc);
  const newCollider = world.createCollider(colliderShape, newBody);
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
} from './stress_old.js';

const GRAVITY_DEFAULT = -9.81;
const PROJECTILE_SPEED = 35;
const MAX_EVENTS = 8;

const colorScale = new THREE.Color();

const BRIDGE_DIMENSIONS = {
  deckSegmentLength: 9,
  deckWidth: 4,
  deckThickness: 0.6,
  deckElevation: 2.2,
  pierWidth: 1.4,
  pierDepth: 2.6,
  pierHeight: 2.2
};

const CAR_PROPS = {
  mass: 2400,
  length: 3.6,
  width: 1.8,
  height: 1.2,
  speed: 4.5
};

class BridgeChunk {
  constructor(desc) {
    Object.assign(this, desc);
    this.colliderHandle = null;
    this.bodyHandle = null;
    this.mesh = null;
    this.active = true;
    this.severity = 0;
    this.pendingForce = { x: 0, y: 0, z: 0 };
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

function createSeverity() {
  return {
    compression: 0,
    tension: 0,
    shear: 0,
    max: 0
  };
}

const HUD = {
  gravityValue: document.getElementById('gravity-value'),
  bondTable: document.getElementById('bond-table'),
  eventLog: document.getElementById('event-log'),
  overlay: document.getElementById('overlay')
};

const controlsUI = {
  gravitySlider: document.getElementById('gravity-slider'),
  fireButton: document.getElementById('fire-projectile'),
  resetButton: document.getElementById('reset-bridge'),
  debugToggle: document.getElementById('toggle-debug')
};

async function init() {
  await RAPIER.init();
  const stressRuntime = await loadStressSolver();

  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));
  const { scene, renderer, camera, controls } = initThree();
  const bridge = buildBridge(scene, world, stressRuntime);
  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: false });
  bridge.debugRenderer = debugRenderer;

  setupControls(world, bridge);

  const clock = new THREE.Clock();

  function loop() {
    const delta = clock.getDelta();
    updateBridge(world, bridge, stressRuntime, delta);
    world.step();
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

  const stressProcessor = stressRuntime.createProcessor({
    nodes: chunks.map((chunk) => ({
      com: chunk.localPosition,
      mass: chunk.mass,
      inertia: chunk.inertia
    })),
    bonds: bonds.map((bond) => ({
      centroid: getBondCentroid(bond, chunks),
      node0: bond.node0,
      node1: bond.node1
    })),
    dataParams: { equalizeMasses: 1, centerBonds: 1 }
  });

  const limits = new StressLimits({
    compressionElasticLimit: 3.5e5,
    compressionFatalLimit: 6.5e5,
    tensionElasticLimit: 2.8e5,
    tensionFatalLimit: 4.5e5,
    shearElasticLimit: 1.8e5,
    shearFatalLimit: 3.5e5
  });

  const bridgeBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(0, 0, 0)
    .setCanSleep(false)
    .setLinearDamping(0.8)
    .setAngularDamping(1.2);
  const bridgeBody = world.createRigidBody(bridgeBodyDesc);

  const deckMaterial = new THREE.MeshStandardMaterial({ color: 0x486fe3, roughness: 0.35, metalness: 0.45 });
  const pierMaterial = new THREE.MeshStandardMaterial({ color: 0x2f3e56, roughness: 0.6, metalness: 0.25 });
  const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x1d2533, roughness: 0.75, metalness: 0.18 });

  const chunkMeshes = [];
  chunks.forEach((chunk) => {
    const colliderDesc = createColliderForChunk(chunk);
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
    mesh.position.copy(chunk.localOffset);
    mesh.quaternion.copy(chunk.localQuat);
    scene.add(mesh);
  });

  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setTranslation(0, -2, 0), groundBody);
  const ground = new THREE.Mesh(new THREE.BoxGeometry(40, 1, 40), new THREE.MeshStandardMaterial({
    color: 0x1a1e2f,
    roughness: 0.8,
    metalness: 0.1
  }));
  ground.position.set(0, -2, 0);
  scene.add(ground);

  updateBondTable(bonds);

  return {
    world,
    scene,
    body: bridgeBody,
    chunks,
    bonds,
    meshes: chunkMeshes,
    stressProcessor,
    limits,
    projectiles: [],
    splitBodies: [],
    activeGravity: GRAVITY_DEFAULT,
    debugRenderer: new RapierDebugRenderer(scene, world)
  };
}

function updateBridge(world, bridge, stressRuntime, delta) {
  const { stressProcessor, chunks, bonds, limits } = bridge;

  const velocities = chunks.map((chunk) => {
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return { lin: vec3(), ang: vec3() };
    }
    const linVel = body.linvel();
    const angVel = body.angvel();
    return {
      lin: vec3(linVel.x, linVel.y, linVel.z),
      ang: vec3(angVel.x, angVel.y, angVel.z)
    };
  });

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

  bonds.forEach((bond, index) => {
    if (!bond.active) {
      return;
    }
    const stress = computeBondStress(
      stressProcessor.bondDesc(index),
      impulses[index],
      solverNodes,
      bond.area
    );
    const severity = limits.severity(stress);
    bond.severity = severity;
    severity.max = Math.max(severity.compression, severity.tension, severity.shear);
    const mode = limits.failureMode(stress);
    if (mode) {
      fractureBond(world, bridge, index, bond, mode);
    }
  });

  syncMeshes(world, bridge);
  updateProjectiles(world, bridge, delta);
  updateLoadVehicle(world, bridge, delta);
  updateBondTable(bonds);
  pushEvent(`Bridge solve: iter ${iterations}, error=(${error.lin.toFixed(3)}, ${error.ang.toFixed(3)})`);
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
  if (translation.x > BRIDGE_DIMENSIONS.deckSegmentLength * 1.6) {
    body.setTranslation({ x: -BRIDGE_DIMENSIONS.deckSegmentLength * 1.6, y: translation.y, z: translation.z }, true);
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
    chunk.mesh.position.add(chunk.localOffset.clone().applyQuaternion(chunk.mesh.quaternion));

    const severity = Math.max(
      chunk.severity?.max ?? 0,
      ...bridge.bonds
        .filter((bond) => bond.node0 === chunk.nodeId || bond.node1 === chunk.nodeId)
        .map((bond) => bond.severity.max)
    );
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

  const deckMass = 18000;
  const pierMass = 32000;
  const pierBaseMass = 15000;

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
        mass: deckMass,
        inertia: deckMass * 0.15
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
        mass: pierMass,
        inertia: pierMass * 0.18
      })
    );
    chunks.push(
      new BridgeChunk({
        id: `pier-base-${index}`,
        shape: { type: 'box', hx: pierWidth * 0.8, hy: pierBaseHeight * 0.5, hz: pierDepth * 0.8 },
        localPosition: vec3(offset, pierBaseHeight * 0.5, 0),
        localRotation: { x: 0, y: 0, z: 0, w: 1 },
        mass: pierBaseMass,
        inertia: pierBaseMass * 0.12
      })
    );
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

function createColliderForChunk(chunk) {
  const rotation = chunk.localQuat ?? new THREE.Quaternion();
  switch (chunk.shape.type) {
    case 'box':
      return RAPIER.ColliderDesc.cuboid(chunk.shape.hx, chunk.shape.hy, chunk.shape.hz)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setRotation(new RAPIER.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
    case 'capsule':
      return RAPIER.ColliderDesc.capsule(chunk.shape.halfHeight, chunk.shape.radius)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setRotation(new RAPIER.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
    default:
      return RAPIER.ColliderDesc.ball(1).setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z);
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
  bonds
    .filter((bond) => bond.active)
    .forEach((bond) => {
      const row = document.createElement('div');
      row.className = 'bond-row';
      const title = document.createElement('div');
      title.textContent = bond.name;
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
    controlsUI.gravitySlider.value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      world.gravity = new RAPIER.Vector3(0, value, 0);
      bridge.activeGravity = value;
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
    controlsUI.debugToggle.addEventListener('click', () => {
      const enabled = bridge.debugRenderer.toggle();
      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
    });
  }
  spawnLoadVehicle(world, bridge);
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

function spawnLoadVehicle(world, bridge) {
  const { length, width, height, mass, speed } = CAR_PROPS;
  const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(-BRIDGE_DIMENSIONS.deckSegmentLength * 1.5, BRIDGE_DIMENSIONS.deckElevation + height * 0.5 + 0.05, 0)
    .setLinearDamping(0.4)
    .setAngularDamping(1.0);
  const body = world.createRigidBody(bodyDesc);
  const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(length * 0.5, height * 0.5, width * 0.5).setMass(mass), body);
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

init().catch((err) => {
  console.error('Failed to initialize bridge demo', err);
});
