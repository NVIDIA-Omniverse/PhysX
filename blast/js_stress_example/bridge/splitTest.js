import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';

import { fractureBond, splitChunk } from './simulation.js';
import { BridgeChunk, BridgeBond } from './models.js';
import { createColliderForChunk, createMeshForChunk } from './factory.js';
import { pushEvent } from './ui.js';
import { resolveParentBody, computeWorldPose } from './transformUtils.js';
import { vec3, StressLimits } from '../stress.js';

const debugLog = document.getElementById('debug-log');

const eventLog = document.getElementById('event-log');

function setDebugText(text) {
  debugLog.textContent = text;
}

let renderer;
let scene;
let camera;
let controls;
let world;
let bridge;
let clock;
let animationId;

async function init() {
  await RAPIER.init();
  setupThree();
  createWorld();
  createTestBridge();
  setupUI();

  clock = new THREE.Clock();
  loop();
}

function setupThree() {
  const canvas = document.getElementById('split-canvas');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(canvas.clientWidth || window.innerWidth, canvas.clientHeight || window.innerHeight, false);

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x05070a);

  camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 200);
  camera.position.set(0, 5, 10);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 0.5, 0);

  const ambient = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 1.0);
  dir.position.set(5, 10, 8);
  scene.add(dir);

  const ground = new THREE.Mesh(
    new THREE.BoxGeometry(40, 1, 40),
    new THREE.MeshStandardMaterial({ color: 0x11151f, roughness: 0.8 })
  );
  ground.position.set(0, -0.5, 0);
  ground.receiveShadow = true;
  scene.add(ground);
}

function createWorld() {
  world = new RAPIER.World(new RAPIER.Vector3(0, -9.81, 0));
  const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5, 20).setTranslation(0, -0.5, 0), groundBody);
}

function createTestBridge() {
  // Build a minimal bridge description reusing the same structures as the full demo.
  const chunk = new BridgeChunk({
    id: 'test-deck',
    shape: { type: 'box', hx: 2, hy: 0.3, hz: 1 },
    localPosition: vec3(0, 1.5, 0),
    localRotation: { x: 0, y: 0, z: 0, w: 1 },
    mass: 5000,
    inertia: 5000 * 0.2
  });
  chunk.nodeId = 0;

  const support = new BridgeChunk({
    id: 'test-support',
    shape: { type: 'box', hx: 1.6, hy: 0.4, hz: 1.2 },
    localPosition: vec3(0, 0.4, 0),
    localRotation: { x: 0, y: 0, z: 0, w: 1 },
    mass: 12000,
    inertia: 12000 * 0.18
  });
  support.nodeId = 1;

  const bond = new BridgeBond({
    id: 'test-bond',
    node0: 0,
    node1: 1,
    area: 4.5,
    name: 'Deck → Support'
  });

  const bridgeBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(0, 0, 0)
      .setLinearDamping(0.8)
      .setAngularDamping(1.0)
      .setCanSleep(false)
  );

  const deckCollider = world.createCollider(createColliderForChunk(chunk), bridgeBody);
  chunk.bodyHandle = bridgeBody.handle;
  chunk.parentBodyHandle = bridgeBody.handle;
  chunk.colliderHandle = deckCollider.handle;

  const supportBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0)
  );
  const supportCollider = world.createCollider(createColliderForChunk(support), supportBody);
  support.bodyHandle = supportBody.handle;
  support.parentBodyHandle = supportBody.handle;
  support.colliderHandle = supportCollider.handle;

  const deckMaterial = new THREE.MeshStandardMaterial({ color: 0x4a73ff, roughness: 0.35, metalness: 0.25 });
  const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.75 });

  const deckMesh = createMeshForChunk(chunk, deckMaterial);
  chunk.mesh = deckMesh;
  scene.add(deckMesh);

  const supportMesh = createMeshForChunk(support, baseMaterial);
  support.mesh = supportMesh;
  scene.add(supportMesh);

  bridge = {
    chunks: [chunk, support],
    bonds: [bond],
    limits: new StressLimits({
      compressionElasticLimit: 1e5,
      compressionFatalLimit: 2e5,
      tensionElasticLimit: 1e5,
      tensionFatalLimit: 2e5,
      shearElasticLimit: 1e5,
      shearFatalLimit: 2e5
    }),
    stressProcessor: {
      bondDesc: () => ({ node0: bond.node0, node1: bond.node1 }),
      getNodes: () => bridge.chunks.map((c) => ({ com: c.localOffset }))
    },
    splitBodies: []
  };

  updateMeshes();
  logBridgeState('Initialised test bridge');
  pushEvent('Test bridge initialised');
}

function setupUI() {
  document.getElementById('btn-fracture').addEventListener('click', () => {
    const bond = bridge.bonds[0];
    fractureBond(world, bridge, 0, bond, 'tension');
    logBridgeState('fractureBond invoked');
    pushEvent('fractureBond() call completed');
  });

  document.getElementById('btn-split').addEventListener('click', () => {
    splitChunk(world, bridge, bridge.chunks[0]);
    logBridgeState('splitChunk invoked directly');
    pushEvent('splitChunk() call completed');
  });

  document.getElementById('btn-reload').addEventListener('click', () => {
    reset();
  });
}

function loop() {
  const delta = clock.getDelta();
  controls.update();
  world.step();
  updateMeshes();
  renderer.render(scene, camera);
  animationId = requestAnimationFrame(loop);
}

function updateMeshes() {
  bridge.chunks.forEach((chunk) => {
    const parentBody = resolveParentBody(world, chunk);
    if (!parentBody) {
      return;
    }
    const pose = computeWorldPose(parentBody, chunk);
    if (!pose) {
      return;
    }
    chunk.mesh.position.copy(pose.position);
    chunk.mesh.quaternion.copy(pose.rotation);
  });
}

function logBridgeState(message) {
  const chunk = bridge.chunks[0];
  const parent = resolveParentBody(world, chunk);
  let snapshot = `${message}\n`;
  if (parent) {
    const pose = computeWorldPose(parent, chunk);
    const velocity = parent.linvel();
    const ang = parent.angvel();
    snapshot += ` position=(${pose.position.x.toFixed(2)}, ${pose.position.y.toFixed(2)}, ${pose.position.z.toFixed(2)})\n`;
    snapshot += ` linvel=(${velocity.x.toFixed(2)}, ${velocity.y.toFixed(2)}, ${velocity.z.toFixed(2)})\n`;
    snapshot += ` angvel=(${ang.x.toFixed(2)}, ${ang.y.toFixed(2)}, ${ang.z.toFixed(2)})\n`;
  } else {
    snapshot += ' chunk body missing\n';
  }
  setDebugText(snapshot);
}

function reset() {
  cancelAnimationFrame(animationId);
  bridge.chunks.forEach((chunk) => {
    if (chunk.mesh) {
      scene.remove(chunk.mesh);
    }
  });
  world.free();
  while (eventLog?.firstChild) {
    eventLog.removeChild(eventLog.firstChild);
  }
  setDebugText('');

  createWorld();
  createTestBridge();
  clock = new THREE.Clock();
  loop();
}

init().catch((err) => {
  console.error('Failed to initialise split test', err);
  setDebugText(`Init failure: ${err.message ?? err}`);
});

