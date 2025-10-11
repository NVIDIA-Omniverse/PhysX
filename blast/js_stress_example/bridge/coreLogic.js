// Shared, headless-friendly core logic used by the web demo and tests.
// No DOM/Three dependencies.

import { ExtForceMode, vec3 } from '../stress.js';
import { handleSplitEvents } from './handleSplitEvents.js';

export function applyForcesAndSolve(bridge) {
  const { solver } = bridge;
  const gravity = bridge.gravity ?? -9.81;

  // Gravity (solver-local frame), scaled like the demo
  solver.addGravity(vec3(0.0, gravity * 0.1, 0.0));

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

  // console.log('    processSolverFractures: splitResults', splitResults.length, splitResults);

  handleSplitEvents(bridge, splitResults, RAPIER, contactForceThreshold);
  rebuildActorsFromSolver(bridge);
  return true;
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
      if (handle == null || !activeContactColliders.has(handle)) return;

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
  const { world, body, colliderToNode, activeContactColliders, pendingContactForces, contactForceScratch } = bridge;
  if (!world || !body || activeContactColliders.size === 0) {
    console.log('    gatherSolverForcesFromRapier: no world or body or activeContactColliders', { world, body, activeContactColliders });
    return [];
  }

  const rootBody = world.getRigidBody(body.handle);
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


