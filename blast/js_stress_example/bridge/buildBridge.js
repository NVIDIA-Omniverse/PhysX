import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import RapierDebugRenderer from '../rapier-debug-renderer.js';

import { BRIDGE_DIMENSIONS, CAR_PROPS, GRAVITY_DEFAULT, LIMITS_DEFAULTS, STRENGTH_DEFAULT, SOLVER_DEFAULTS } from './constants.js';
import { BridgeChunk, BridgeBond } from './models.js';
import { vec3, StressLimits } from '../stress.js';
import { createColliderForChunk, createMeshForChunk } from './factory.js';
import { updateBondTable } from './ui.js';
import { scaleStressLimits } from './utils.js';

export function buildBridge(scene, world, stressRuntime) {
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
    chunk.parentBodyHandle = bridgeBody.handle;
    chunk.localOffset.copy(chunk.restOffset);
    chunk.localQuat.copy(chunk.restQuat);

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

  const baseLimits = { ...LIMITS_DEFAULTS };
  const strengthScale = STRENGTH_DEFAULT;
  const limitsCtor = StressLimits;
  const limits = new limitsCtor(scaleStressLimits(baseLimits, strengthScale));

  return {
    world,
    scene,
    body: bridgeBody,
    chunks,
    bonds,
    meshes: chunkMeshes,
    stressProcessor,
    baseLimits,
    limits,
    limitsCtor,
    strengthScale,
    solverSettings: {
      maxIterations: SOLVER_DEFAULTS.maxIterations,
      toleranceExponent: SOLVER_DEFAULTS.toleranceExponent
    },
    projectiles: [],
    splitBodies: [],
    activeGravity: GRAVITY_DEFAULT,
    debugRenderer: new RapierDebugRenderer(scene, world, { enabled: true })
  };
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

export function spawnLoadVehicle(world, bridge) {
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

