// Shared, headless-friendly core logic used by the web demo and tests.
// No DOM/Three dependencies.
import { ExtForceMode, vec3 } from '../stress.js';
export function applyForcesAndSolve(bridge) {
    const { solver } = bridge;
    const gravity = bridge.gravity ?? -9.81;
    // Gravity in solver-local frame (slightly scaled for timely overstress in tests)
    solver.addGravity(vec3(0.0, gravity * 0.2, 0.0));
    // Contact forces from Rapier → solver space
    const solverForces = gatherSolverForcesFromRapier(bridge);
    for (const force of solverForces) {
        solver.addForce(force.nodeIndex, force.localPoint, force.localForce, force.mode ?? ExtForceMode.Force);
    }
    // Iterate solver to convergence
    let iterations = 0;
    const maxIterations = bridge.settings?.maxSolverIterationsPerFrame ?? 25;
    let error = { lin: 0, ang: 0 };
    for (; iterations < maxIterations; ++iterations) {
        solver.update();
        error = solver.stressError();
        if (solver.converged())
            break;
        if (error.lin <= bridge.targetError && error.ang <= bridge.targetError)
            break;
    }
    bridge.iterationCount = iterations + 1;
    bridge.lastError = error;
    bridge.overstressed = solver.overstressedBondCount();
}
export function processSolverFractures(bridge, _RAPIER, _contactForceThreshold = 0.0) {
    const { solver } = bridge;
    if (!solver)
        return null;
    const fractureSets = solver.generateFractureCommandsPerActor();
    if (!Array.isArray(fractureSets) || fractureSets.length === 0) {
        return null;
    }
    const splitResults = solver.applyFractureCommands(fractureSets);
    if (!Array.isArray(splitResults) || splitResults.length === 0) {
        return null;
    }
    bridge._handleSplitEventsCount = (bridge._handleSplitEventsCount ?? 0) + 1;
    // Log for determinism (headless tests)
    try {
        const normalized = normalizeSplitResults(splitResults);
        if (!bridge._splitLog)
            bridge._splitLog = [];
        bridge._splitLog.push(normalized);
    }
    catch (_) {
        // Ignore logging errors
    }
    // Return for caller to queue/apply during safe frames
    return { splitResults, fractureSets };
}
export function normalizeSplitResults(splitResults) {
    if (!Array.isArray(splitResults))
        return [];
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
export function drainContactForces(bridge) {
    const { eventQueue, colliderToNode, activeContactColliders, pendingContactForces, world } = bridge;
    if (typeof eventQueue.drainContactForceEvents !== 'function') {
        return;
    }
    eventQueue.drainContactForceEvents((event) => {
        if (!event)
            return;
        const totalForce = event.totalForce?.();
        const magnitude = event.totalForceMagnitude?.();
        if (!totalForce || !(magnitude > 0))
            return;
        const collider1 = event.collider1?.();
        const collider2 = event.collider2?.();
        const worldPoint = event.worldContactPoint?.();
        const worldPoint2 = event.worldContactPoint2?.();
        const totalImpulse = event.totalImpulse?.();
        const register = (handle, direction) => {
            if (handle === undefined || handle === null)
                return;
            if (!colliderToNode.has(handle))
                return;
            const collider = world.getCollider(handle);
            if (!collider) {
                activeContactColliders.delete(handle);
                colliderToNode.delete(handle);
                return;
            }
            const parentHandle = collider.parent();
            if (parentHandle === undefined || parentHandle === null)
                return;
            const body = world.getRigidBody(parentHandle);
            if (!body)
                return;
            const bodyTranslation = body.translation();
            const forceVec = {
                x: (totalForce.x ?? 0) * direction,
                y: (totalForce.y ?? 0) * direction,
                z: (totalForce.z ?? 0) * direction
            };
            const point = worldPoint
                ? { x: worldPoint.x ?? 0, y: worldPoint.y ?? 0, z: worldPoint.z ?? 0 }
                : worldPoint2
                    ? { x: worldPoint2.x ?? 0, y: worldPoint2.y ?? 0, z: worldPoint2.z ?? 0 }
                    : { x: bodyTranslation.x, y: bodyTranslation.y, z: bodyTranslation.z };
            const impulse = totalImpulse
                ? { x: (totalImpulse.x ?? 0) * direction, y: (totalImpulse.y ?? 0) * direction, z: (totalImpulse.z ?? 0) * direction }
                : { x: 0, y: 0, z: 0 };
            const existing = pendingContactForces.get(handle);
            if (existing) {
                existing.force.x += forceVec.x;
                existing.force.y += forceVec.y;
                existing.force.z += forceVec.z;
                existing.point = point;
                existing.impulse.x += impulse.x;
                existing.impulse.y += impulse.y;
                existing.impulse.z += impulse.z;
            }
            else {
                pendingContactForces.set(handle, { force: forceVec, point, impulse });
            }
        };
        register(collider1, 1);
        register(collider2, -1);
    });
}
export function collectContactForceEvents(eventQueue) {
    const out = [];
    if (!eventQueue || typeof eventQueue.drainContactForceEvents !== 'function')
        return out;
    eventQueue.drainContactForceEvents((event) => {
        if (!event)
            return;
        const totalForce = event.totalForce?.();
        const magnitude = event.totalForceMagnitude?.();
        if (!totalForce || !(magnitude > 0))
            return;
        out.push({
            c1: event.collider1?.(),
            c2: event.collider2?.(),
            totalForce: { x: totalForce.x ?? 0, y: totalForce.y ?? 0, z: totalForce.z ?? 0 },
            worldPoint: event.worldContactPoint?.(),
            worldPoint2: event.worldContactPoint2?.(),
            totalImpulse: event.totalImpulse?.()
        });
    });
    return out;
}
export function accumulateContactForcesFromEvents(bridge, events) {
    const { world, colliderToNode, activeContactColliders, pendingContactForces } = bridge;
    const rootBody = world.getRigidBody(bridge.bodyHandle);
    if (!rootBody)
        return;
    // Convert world vectors to root-body local frame
    const qInv = rootBody.rotation();
    const quatApply = (v) => {
        const x = qInv.x, y = qInv.y, z = qInv.z, w = qInv.w;
        const vx = v.x, vy = v.y, vz = v.z;
        const ix = w * vx + y * vz - z * vy;
        const iy = w * vy + z * vx - x * vz;
        const iz = w * vz + x * vy - y * vx;
        const iw = -x * vx - y * vy - z * vz;
        return {
            x: ix * w + iw * -x + iy * -z - iz * -y,
            y: iy * w + iw * -y + iz * -x - ix * -z,
            z: iz * w + iw * -z + ix * -y - iy * -x
        };
    };
    const register = (handle, dir, ev) => {
        if (handle == null)
            return;
        if (!colliderToNode.has(handle))
            return;
        const collider = world.getCollider(handle);
        if (!collider) {
            activeContactColliders.delete(handle);
            colliderToNode.delete(handle);
            return;
        }
        const parentHandle = collider.parent();
        if (parentHandle == null)
            return;
        const body = world.getRigidBody(parentHandle);
        if (!body)
            return;
        const bodyTr = body.translation();
        const force = ev.totalForce
            ? { x: ev.totalForce.x * dir, y: ev.totalForce.y * dir, z: ev.totalForce.z * dir }
            : { x: 0, y: 0, z: 0 };
        const point = ev.worldPoint
            ? ev.worldPoint
            : ev.worldPoint2
                ? ev.worldPoint2
                : { x: bodyTr.x, y: bodyTr.y, z: bodyTr.z };
        const impulse = ev.totalImpulse
            ? { x: ev.totalImpulse.x * dir, y: ev.totalImpulse.y * dir, z: ev.totalImpulse.z * dir }
            : { x: 0, y: 0, z: 0 };
        const localForce = quatApply(force);
        const localPoint = quatApply(point);
        const existing = pendingContactForces.get(handle);
        if (existing) {
            existing.force.x += localForce.x;
            existing.force.y += localForce.y;
            existing.force.z += localForce.z;
            existing.point = localPoint;
            existing.impulse.x += impulse.x;
            existing.impulse.y += impulse.y;
            existing.impulse.z += impulse.z;
        }
        else {
            pendingContactForces.set(handle, { force: localForce, point: localPoint, impulse });
        }
    };
    for (const ev of events) {
        register(ev.c1, 1, ev);
        register(ev.c2, -1, ev);
    }
}
export function gatherSolverForcesFromRapier(bridge) {
    const { world, bodyHandle, colliderToNode, activeContactColliders, pendingContactForces, contactForceScratch } = bridge;
    const rootBody = world.getRigidBody(bodyHandle);
    if (!rootBody)
        return [];
    // Convert world vectors to root-body local frame
    const qInv = rootBody.rotation();
    const quatApply = (v) => {
        const x = qInv.x, y = qInv.y, z = qInv.z, w = qInv.w;
        const vx = v.x, vy = v.y, vz = v.z;
        const ix = w * vx + y * vz - z * vy;
        const iy = w * vy + z * vx - x * vz;
        const iz = w * vz + x * vy - y * vx;
        const iw = -x * vx - y * vy - z * vz;
        return {
            x: ix * w + iw * -x + iy * -z - iz * -y,
            y: iy * w + iw * -y + iz * -x - ix * -z,
            z: iz * w + iw * -z + ix * -y - iy * -x
        };
    };
    const results = contactForceScratch;
    results.length = 0;
    for (const handle of activeContactColliders) {
        const contact = pendingContactForces.get(handle);
        const nodeIndex = colliderToNode.get(handle);
        if (!contact || nodeIndex === undefined || nodeIndex < 0)
            continue;
        const localForce = quatApply(contact.force);
        const localPoint = quatApply(contact.point);
        results.push({
            nodeIndex,
            localForce: vec3(localForce.x, localForce.y, localForce.z),
            localPoint: vec3(localPoint.x, localPoint.y, localPoint.z),
            mode: ExtForceMode.Force
        });
    }
    pendingContactForces.clear();
    return results;
}
export function sweepBeforeEventfulStep(bridge) {
    const { world, disabledCollidersToRemove, activeContactColliders, colliderToNode, pendingContactForces, chunks } = bridge;
    // Remove queued disabled colliders
    if (disabledCollidersToRemove.size > 0) {
        for (const h of Array.from(disabledCollidersToRemove)) {
            const c = world.getCollider(h);
            if (c)
                world.removeCollider(c, false);
            disabledCollidersToRemove.delete(h);
        }
    }
    // Prune stale collider handles from tracking sets
    for (const h of Array.from(activeContactColliders)) {
        const c = world.getCollider(h);
        const parent = c?.parent();
        const body = parent !== undefined ? world.getRigidBody(parent) : undefined;
        const enabled = c?.isEnabled ? c.isEnabled() : true;
        if (!c || !enabled || !body) {
            activeContactColliders.delete(h);
            colliderToNode.delete(h);
            pendingContactForces.delete(h);
        }
    }
    for (const [h] of Array.from(colliderToNode.entries())) {
        const c = world.getCollider(h);
        const parent = c?.parent();
        const body = parent !== undefined ? world.getRigidBody(parent) : undefined;
        const enabled = c?.isEnabled ? c.isEnabled() : true;
        if (!c || !enabled || !body) {
            colliderToNode.delete(h);
            activeContactColliders.delete(h);
            pendingContactForces.delete(h);
        }
    }
    // Ensure chunk collider handles are valid
    for (const chunk of chunks) {
        const h = chunk.colliderHandle;
        if (h === null)
            continue;
        const c = world.getCollider(h);
        const parent = c?.parent();
        const body = parent !== undefined ? world.getRigidBody(parent) : undefined;
        const enabled = c?.isEnabled ? c.isEnabled() : true;
        if (!c || !enabled || !body) {
            chunk.colliderHandle = null;
            chunk.active = false;
        }
    }
}
