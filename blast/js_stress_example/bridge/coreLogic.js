// Shared, headless-friendly core logic used by the web demo and tests.
// No DOM/Three dependencies.

import { ExtForceMode, vec3 } from '../stress.js';
import { handleSplitEvents } from './handleSplitEvents.js';

export function applyForcesAndSolve(bridge) {
  const { solver } = bridge;
  const gravity = bridge.gravity ?? -9.81;

  // Gravity (solver-local frame), slightly higher scale to ensure timely overstress in headless tests
  solver.addGravity(vec3(0.0, gravity * 0.2, 0.0));

  // Contact forces collected into solver-space
  const solverForces = gatherSolverForcesFromRapier(bridge);
//   console.log('    applyForcesAndSolve: solverForces', solverForces.length, solverForces);
  (solverForces ?? []).forEach((force) => {
    solver.addForce(force.nodeIndex, force.localPoint, force.localForce, force.mode ?? ExtForceMode.Force);
  });

  // Iterate solver
  let iterations = 0;
  let error = { lin: 0, ang: 0 };
  const maxIterations = bridge.settings?.maxSolverIterationsPerFrame ?? 25;
  for (; iterations < maxIterations; ++iterations) {
    solver.update();
    error = solver.stressError();
    if (solver.converged()) break;
    if (error.lin <= (bridge.targetError ?? 1.0e-6) && error.ang <= (bridge.targetError ?? 1.0e-6)) break;
  }

  bridge.iterationCount = iterations + 1;
  bridge.lastError = error;
  bridge.overstressed = solver.overstressedBondCount();
}

export function processSolverFractures(bridge, RAPIER, contactForceThreshold = 0.0) {
  const { solver } = bridge;
  if (!solver) return false;

  const fractureSets = solver.generateFractureCommandsPerActor();
  if (!Array.isArray(fractureSets) || fractureSets.length === 0) {
    return false;
  }

  const splitResults = solver.applyFractureCommands(fractureSets);
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return false;
  }

  bridge._handleSplitEventsCount = (bridge._handleSplitEventsCount ?? 0) + 1;
  // Normalize and record the split results for determinism/assertions in tests
  try {
    console.log('    processSolverFractures: splitResults', splitResults.length, splitResults);
    console.log('    splitResults (json):', JSON.stringify(splitResults));
    console.log('    fractureSets (json):', JSON.stringify(fractureSets));
  
    const normalized = normalizeSplitResults(splitResults);
    if (!bridge._splitLog) bridge._splitLog = [];
    bridge._splitLog.push(normalized);
  } catch (_) {}

  handleSplitEvents(bridge, splitResults, RAPIER, contactForceThreshold); // FIXME: re-enabling this line causes Rapier panic

  // After splits, sweep and disable any colliders that lost a parent/body (parry BVH safety)
  try {
    if (bridge.activeContactColliders && bridge.colliderToNode) {
      const toDisable = [];
      bridge.activeContactColliders.forEach((h) => {
        const col = bridge.world.getCollider(h);
        if (!col || !col.parent()) toDisable.push(h);
      });
      toDisable.forEach((h) => {
        const col = bridge.world.getCollider(h);
        if (col) col.setEnabled(false);
        bridge.activeContactColliders.delete(h);
        bridge.colliderToNode.delete(h);
      });
    }
  } catch (_) {}

  // Optional: full Rapier rebuild after split to avoid BVH edge cases
  if (bridge._rebuildOnSplit) {
    try { rebuildRapierAfterSplit(bridge); } catch (e) { console.warn('rebuildRapierAfterSplit failed', e); }
  }
  // After any split, run safe physics steps without events to let BVH settle fully
  bridge._justSplitFrames = Math.max(bridge._justSplitFrames ?? 0, 3);
  // Reset EventQueue to drop stale events referencing old handles
  try {
    const R = bridge.world?.RAPIER;
    if (R) bridge.eventQueue = new R.EventQueue(true);
  } catch (_) {}
  // Clear any pending contact forces accumulated with stale handles
  try { bridge.pendingContactForces?.clear?.(); } catch (_) {}
  // Aggressively clear contact tracking; will be rebuilt by migrations
  try {
    bridge.activeContactColliders?.clear?.();
    bridge.colliderToNode?.clear?.();
  } catch (_) {}
  // Stricter invariant sweep for colliders/actorMap
  try {
    const world = bridge.world;
    if (Array.isArray(bridge.chunks)) {
      for (const chunk of bridge.chunks) {
        const h = chunk?.colliderHandle;
        if (h == null) continue;
        const c = world.getCollider(h);
        const parent = c ? c.parent() : undefined;
        const body = parent != null ? world.getRigidBody(parent) : undefined;
        if (!c || (c.isEnabled && !c.isEnabled()) || !body) {
          chunk.colliderHandle = undefined;
          if (chunk.active !== undefined) chunk.active = false;
        }
      }
    }
    if (bridge.actorMap && typeof bridge.actorMap.forEach === 'function') {
      for (const [actorIndex, entry] of Array.from(bridge.actorMap.entries())) {
        const body = world.getRigidBody(entry?.bodyHandle);
        if (!body) bridge.actorMap.delete(actorIndex);
      }
    }
  } catch (_) {}
  rebuildActorsFromSolver(bridge);
  return true;
}

export function rebuildRapierAfterSplit(bridge) {
  const R = bridge.world.RAPIER;
  const oldWorld = bridge.world;
  const gravity = oldWorld.gravity ?? { x: 0, y: -9.81, z: 0 };
  const newWorld = new R.World(gravity);
  newWorld.RAPIER = R;
  const newEventQueue = new R.EventQueue(true);

  const colliderToNode = new Map();
  const activeContactColliders = new Set();

  // Root fixed body for any non-detached deck chunks
  const rootBody = newWorld.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(0, 0, 0).setCanSleep(false));

  // Helper to create collider on body
  const createChunkCollider = (chunk, body) => {
    const halfX = (chunk.size?.x ?? 1) * 0.5;
    const halfY = (chunk.size?.y ?? 1) * 0.5;
    const halfZ = (chunk.size?.z ?? 1) * 0.5;
    const tx = chunk.baseLocalOffset?.x ?? 0;
    const ty = chunk.baseLocalOffset?.y ?? 0;
    const tz = chunk.baseLocalOffset?.z ?? 0;
    const col = newWorld.createCollider(
      R.ColliderDesc.cuboid(halfX, halfY, halfZ)
        .setTranslation(tx, ty, tz)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(R.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0),
      body
    );
    chunk.bodyHandle = body.handle;
    chunk.colliderHandle = col.handle;
    colliderToNode.set(col.handle, chunk.nodeIndex);
    activeContactColliders.add(col.handle);
  };

  // Create supports first (fixed at their world position)
  (bridge.chunks ?? []).forEach((chunk) => {
    if (!chunk || !chunk.isSupport) return;
    const pos = chunk.baseWorldPosition ?? { x: 0, y: 0, z: 0 };
    const body = newWorld.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(pos.x, pos.y, pos.z).setCanSleep(false));
    // supports have local offset at 0,0,0
    const save = chunk.baseLocalOffset; chunk.baseLocalOffset = { x: 0, y: 0, z: 0 };
    createChunkCollider(chunk, body);
    // restore
    chunk.baseLocalOffset = save;
  });

  // Create deck chunks: detached → dynamic body at old body's pose; else attach to root
  (bridge.chunks ?? []).forEach((chunk) => {
    if (!chunk || chunk.isSupport) return;
    if (chunk.detached) {
      const oldBody = oldWorld.getRigidBody(chunk.bodyHandle);
      const t = oldBody?.translation?.();
      const r = oldBody?.rotation?.();
      const bodyDesc = R.RigidBodyDesc.dynamic()
        .setTranslation(t?.x ?? 0, t?.y ?? 0, t?.z ?? 0)
        .setRotation(r ?? { x: 0, y: 0, z: 0, w: 1 })
        .setLinearDamping(oldBody?.linearDamping?.() ?? 0.4)
        .setAngularDamping(oldBody?.angularDamping?.() ?? 1.0);
      const body = newWorld.createRigidBody(bodyDesc);
      if (oldBody) {
        const lv = oldBody.linvel?.(); const av = oldBody.angvel?.();
        if (lv) body.setLinvel({ x: lv.x, y: lv.y, z: lv.z }, true);
        if (av) body.setAngvel({ x: av.x, y: av.y, z: av.z }, true);
      }
      createChunkCollider(chunk, body);
    } else {
      createChunkCollider(chunk, rootBody);
    }
  });

  // Swap in new world and event queue
  bridge.world = newWorld;
  bridge.eventQueue = newEventQueue;
  bridge.activeContactColliders = activeContactColliders;
  bridge.colliderToNode = colliderToNode;
  // Update root body reference used by other systems
  bridge.bodyHandle = rootBody.handle;
  // Reset pending contact forces (handles from old world are invalid)
  bridge.pendingContactForces = new Map();
  // Reset actorMap body handles to safe root (will be refined by future splits)
  if (bridge.actorMap) {
    const remap = new Map();
    bridge.actorMap.forEach((entry, key) => {
      remap.set(key, { bodyHandle: rootBody.handle });
    });
    bridge.actorMap = remap;
  }
}

export function normalizeSplitResults(splitResults) {
  if (!Array.isArray(splitResults)) return [];
  return splitResults.map((evt) => ({
    parentActorIndex: evt?.parentActorIndex ?? -1,
    children: (evt?.children ?? [])
      .map((c) => ({
        actorIndex: c?.actorIndex ?? -1,
        nodes: Array.isArray(c?.nodes) ? [...c.nodes].sort((a, b) => a - b) : []
      }))
      .sort((a, b) => a.actorIndex - b.actorIndex)
  }));
}

export function rebuildActorsFromSolver(bridge) {
  const solverActors = bridge.solver.actors();
  bridge.actorMap = bridge.actorMap ?? new Map();

  const knownActorIndices = new Set();
  solverActors.forEach((actor) => knownActorIndices.add(actor.actorIndex));

  // Clean up stale entries in the actor map.
  [...bridge.actorMap.keys()].forEach((key) => {
    if (!knownActorIndices.has(key)) {
      bridge.actorMap.delete(key);
    }
  });
}

export function drainContactForces(bridge) {
  const { eventQueue, colliderToNode, activeContactColliders, pendingContactForces, world } = bridge;
  if (!eventQueue || typeof eventQueue.drainContactForceEvents !== 'function' || !world) {
    return;
  }

  eventQueue.drainContactForceEvents((event) => {
    if (!event) return;

    const totalForce = event.totalForce?.();
    const magnitude = event.totalForceMagnitude?.();
    if (!totalForce || !(magnitude > 0)) return;

    const collider1 = event.collider1?.();
    const collider2 = event.collider2?.();
    const worldPoint = event.worldContactPoint ? event.worldContactPoint() : undefined;
    const worldPoint2 = event.worldContactPoint2 ? event.worldContactPoint2() : undefined;
    const totalImpulse = event.totalImpulse ? event.totalImpulse() : undefined;

    const register = (handle, direction) => {
      if (handle == null) return;
      if (!colliderToNode.has(handle)) return; // only care about deck/support chunk colliders

      const collider = world.getCollider(handle);
      if (!collider) {
        // Prune stale handle
        activeContactColliders.delete(handle);
        colliderToNode.delete(handle);
        return;
      }
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
        ? { x: (totalImpulse.x ?? 0) * direction, y: (totalImpulse.y ?? 0) * direction, z: (totalImpulse.z ?? 0) * direction }
        : undefined;

      const existing = pendingContactForces.get(handle);
      if (existing) {
        existing.force.x += forceVec.x; existing.force.y += forceVec.y; existing.force.z += forceVec.z;
        existing.point = point;
        if (impulse) {
          if (existing.impulse) {
            existing.impulse.x += impulse.x; existing.impulse.y += impulse.y; existing.impulse.z += impulse.z;
          } else {
            existing.impulse = { ...impulse };
          }
        }
      } else {
        pendingContactForces.set(handle, { force: forceVec, point, impulse: impulse || { x: 0, y: 0, z: 0 } });
      }
    };

    register(collider1, 1);
    register(collider2, -1);
  });
}

export function gatherSolverForcesFromRapier(bridge) {
  const { world, bodyHandle, colliderToNode, activeContactColliders, pendingContactForces, contactForceScratch } = bridge;
  if (!world || !bodyHandle || activeContactColliders.size === 0) {
    return [];
  }

  const rootBody = world.getRigidBody(bodyHandle);
  if (!rootBody) return [];

  // Convert world vectors to root-body local
  const qInv = {
    x: rootBody.rotation().x,
    y: rootBody.rotation().y,
    z: rootBody.rotation().z,
    w: rootBody.rotation().w
  };

  const quatApply = (v) => {
    // minimal quaternion-vector rotation using THREE-like conventions
    const x = qInv.x, y = qInv.y, z = qInv.z, w = qInv.w;
    const vx = v.x, vy = v.y, vz = v.z;
    // q * v
    const ix =  w * vx + y * vz - z * vy;
    const iy =  w * vy + z * vx - x * vz;
    const iz =  w * vz + x * vy - y * vx;
    const iw = -x * vx - y * vy - z * vz;
    // (q * v) * q^-1
    return {
      x: ix * w + iw * -x + iy * -z - iz * -y,
      y: iy * w + iw * -y + iz * -x - ix * -z,
      z: iz * w + iw * -z + ix * -y - iy * -x
    };
  };

  const results = contactForceScratch ?? [];
  results.length = 0;

  activeContactColliders.forEach((handle) => {
    const contact = pendingContactForces.get(handle);
    const nodeIndex = colliderToNode.get(handle);
    if (!contact || !Number.isInteger(nodeIndex) || nodeIndex < 0) return;

    const localForce = quatApply(contact.force);
    const localPoint = quatApply(contact.point);
    const impulseVec = contact.impulse ? quatApply(contact.impulse) : undefined;

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


// Drop invalid collider handles from tracking collections before an eventful step
export function sweepBeforeEventfulStep(bridge) {
  try {
    const world = bridge.world;
    if (!world) return;

    // Invariant: No pending disabled colliders remaining
    if (bridge.disabledCollidersToRemove && bridge.disabledCollidersToRemove.size > 0) {
      for (const h of Array.from(bridge.disabledCollidersToRemove)) {
        const c = world.getCollider(h);
        if (c) world.removeCollider(c, false);
        bridge.disabledCollidersToRemove.delete(h);
      }
    }

    if (bridge.activeContactColliders && typeof bridge.activeContactColliders.forEach === 'function') {
      for (const h of Array.from(bridge.activeContactColliders)) {
        const c = world.getCollider(h);
        const parent = c ? c.parent() : undefined;
        const body = parent != null ? world.getRigidBody(parent) : undefined;
        if (!c || (c.isEnabled && !c.isEnabled()) || !body) {
          bridge.activeContactColliders.delete(h);
          bridge.colliderToNode?.delete?.(h);
          bridge.pendingContactForces?.delete?.(h);
        }
      }
    }

    if (bridge.colliderToNode && typeof bridge.colliderToNode.forEach === 'function') {
      for (const [h] of Array.from(bridge.colliderToNode.entries())) {
        const c = world.getCollider(h);
        const parent = c ? c.parent() : undefined;
        const body = parent != null ? world.getRigidBody(parent) : undefined;
        if (!c || (c.isEnabled && !c.isEnabled()) || !body) {
          bridge.colliderToNode.delete(h);
          bridge.activeContactColliders?.delete?.(h);
          bridge.pendingContactForces?.delete?.(h);
        }
      }
    }

    // Invariant: all chunk collider handles must be live and enabled
    if (Array.isArray(bridge.chunks)) {
      for (const chunk of bridge.chunks) {
        const h = chunk?.colliderHandle;
        if (h == null) continue;
        const c = world.getCollider(h);
        const parent = c ? c.parent() : undefined;
        const body = parent != null ? world.getRigidBody(parent) : undefined;
        if (!c || (c.isEnabled && !c.isEnabled()) || !body) {
          chunk.colliderHandle = null;
          if (chunk.active !== undefined) chunk.active = false;
        }
      }
    }
  } catch (_) {}
}

