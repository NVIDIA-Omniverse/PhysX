import RAPIER from '@dimforge/rapier3d-compat';
import { loadStressSolver } from '../stress.js';
import { buildBridgeScenario } from '../extBridgeScenario.js';

export function createBridgeCore({ runtime, world, scenario, gravity = -9.81, strengthScale = 0.03 }) {
  const settings = runtime.defaultExtSettings();
  settings.maxSolverIterationsPerFrame = 32;
  settings.graphReductionLevel = 0;

  const BASE_LIMITS = {
    compressionElasticLimit: 0.001,
    compressionFatalLimit: 0.002,
    tensionElasticLimit: 0.001,
    tensionFatalLimit: 0.002,
    shearElasticLimit: 0.001,
    shearFatalLimit: 0.002
  };
  settings.compressionElasticLimit = BASE_LIMITS.compressionElasticLimit * strengthScale;
  settings.compressionFatalLimit = BASE_LIMITS.compressionFatalLimit * strengthScale;
  settings.tensionElasticLimit = BASE_LIMITS.tensionElasticLimit * strengthScale;
  settings.tensionFatalLimit = BASE_LIMITS.tensionFatalLimit * strengthScale;
  settings.shearElasticLimit = BASE_LIMITS.shearElasticLimit * strengthScale;
  settings.shearFatalLimit = BASE_LIMITS.shearFatalLimit * strengthScale;

  const solver = runtime.createExtSolver({ nodes: scenario.nodes, bonds: scenario.bonds, settings });

  const bridgeBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0));

  const spacing = scenario.spacing;
  const chunks = [];
  const colliderToNode = new Map();
  const activeContactColliders = new Set();
  const pendingContactForces = new Map();
  const eventQueue = new RAPIER.EventQueue(true);

  const nodeSizeScale = 1.0;
  scenario.nodes.forEach((node, nodeIndex) => {
    const coord = scenario.gridCoordinates[nodeIndex];
    if (!coord) return;
    if (coord.iy >= 0) {
      const sizeX = Math.max(spacing.x * nodeSizeScale, 0.25);
      const sizeY = Math.max(spacing.y * nodeSizeScale, 0.2);
      const sizeZ = scenario.widthSegments > 1 ? Math.max(spacing.z * nodeSizeScale, 0.25) : Math.max(scenario.parameters.deckWidth * nodeSizeScale, 0.8);
      const chunk = {
        nodeIndex,
        size: { x: sizeX, y: sizeY, z: sizeZ },
        isSupport: false,
        baseLocalOffset: { x: node.centroid.x, y: node.centroid.y, z: node.centroid.z },
        localOffset: { x: node.centroid.x, y: node.centroid.y, z: node.centroid.z, copy(v) { this.x=v.x; this.y=v.y; this.z=v.z; } },
        mesh: null,
        colliderHandle: null,
        bodyHandle: null,
        active: true,
        detached: false,
        baseColor: { r: 0, g: 0, b: 0 },
        stressSeverity: 0
      };
      chunks.push(chunk);
      const colliderDesc = RAPIER.ColliderDesc.cuboid(sizeX*0.5, sizeY*0.5, sizeZ*0.5)
        .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0);
      const collider = world.createCollider(colliderDesc, bridgeBody);
      chunk.colliderHandle = collider.handle;
      chunk.bodyHandle = bridgeBody.handle;
      colliderToNode.set(collider.handle, nodeIndex);
      activeContactColliders.add(collider.handle);
    } else if (coord.iy === -1) {
      const sizeX = Math.max(spacing.x * 0.6, 0.4);
      const sizeZ = scenario.widthSegments > 1 ? Math.max(spacing.z * 0.6, 0.4) : Math.max(scenario.parameters.deckWidth * 0.4, 0.4);
      const sizeY = scenario.parameters.pierHeight;
      const chunk = {
        nodeIndex,
        size: { x: sizeX, y: sizeY, z: sizeZ },
        isSupport: true,
        baseLocalOffset: { x: 0, y: 0, z: 0 },
        localOffset: { x: 0, y: 0, z: 0, copy(v) { this.x=v.x; this.y=v.y; this.z=v.z; } },
        mesh: null,
        colliderHandle: null,
        bodyHandle: null,
        active: true,
        detached: false,
        baseColor: { r: 0, g: 0, b: 0 },
        stressSeverity: 0
      };
      chunks.push(chunk);
      const supportBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(node.centroid.x, node.centroid.y, node.centroid.z));
      const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(sizeX*0.5, sizeY*0.5, sizeZ*0.5), supportBody);
      chunk.bodyHandle = supportBody.handle;
      chunk.colliderHandle = collider.handle;
      colliderToNode.set(collider.handle, nodeIndex);
      activeContactColliders.add(collider.handle);
    }
  });

  const actorMap = new Map();
  solver.actors().forEach((actor) => actorMap.set(actor.actorIndex, { bodyHandle: bridgeBody.handle }));

  const topLayerNodeIndices = scenario.topColumnNodes.reduce((acc, column) => {
    if (!Array.isArray(column)) return acc;
    column.forEach((nodeIndex) => { if (Number.isInteger(nodeIndex)) acc.push(nodeIndex); });
    return acc;
  }, []);

  return {
    runtime,
    world,
    solver,
    settings,
    scenario,
    body: bridgeBody,
    chunks,
    colliderToNode,
    activeContactColliders,
    pendingContactForces,
    contactForceScratch: [],
    eventQueue,
    actorMap,
    gravity,
    targetError: 1.0e-6,
    iterationCount: 0,
    lastError: { lin: 0, ang: 0 },
    overstressed: 0,
    topLayerNodeIndices
  };
}

export async function buildBridgeShared({ gravity = -9.81, strengthScale = 0.03 } = {}) {
  await RAPIER.init();
  const runtime = await loadStressSolver();
  const scenario = buildBridgeScenario();
  const world = new RAPIER.World({ x: 0, y: gravity, z: 0 });
  world.RAPIER = RAPIER;
  return createBridgeCore({ runtime, world, scenario, gravity, strengthScale });
}



