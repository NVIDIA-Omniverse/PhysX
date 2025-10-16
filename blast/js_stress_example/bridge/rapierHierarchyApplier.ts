// Reusable, testable Rapier split-applier with two-phase migration.
// Phase 1 (enqueueSplitResults): schedule body creations and mark chunks detached without touching colliders.
// Phase 2 (applyPendingMigrations): create bodies, migrate colliders, prune disabled ones.

import type { BridgeCore, ChunkData, SplitEvent } from './types.js';

function getChunkByNode(bridge: BridgeCore, nodeIndex: number): ChunkData | null {
  for (const chunk of bridge.chunks) {
    if (chunk.nodeIndex === nodeIndex) return chunk;
  }
  return null;
}

export function enqueueSplitResults(bridge: BridgeCore, splitResults: SplitEvent[]): number {
  if (splitResults.length === 0) return 0;

  let scheduledChildren = 0;

  for (const evt of splitResults) {
    const parentActorIndex = evt.parentActorIndex;
    const parentEntry = bridge.actorMap.get(parentActorIndex);
    if (!parentEntry) continue;

    const parentBodyHandle = parentEntry.bodyHandle;
    const parentChild = evt.children.find((c) => c.actorIndex === parentActorIndex);

    for (const child of evt.children) {
      if (child.nodes.length === 0) continue;

      // Queue body creation
      bridge.pendingBodiesToCreate.push({
        actorIndex: child.actorIndex,
        inheritFromBodyHandle: parentBodyHandle,
        nodes: child.nodes.slice()
      });

      // Temporarily map child actor to parent body (safe until Phase 2)
      bridge.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });

      // Mark chunks as detached
      for (const nodeIndex of child.nodes) {
        const chunk = getChunkByNode(bridge, nodeIndex);
        if (!chunk || chunk.isSupport) continue;
        chunk.localOffset.x = chunk.baseLocalOffset.x;
        chunk.localOffset.y = chunk.baseLocalOffset.y;
        chunk.localOffset.z = chunk.baseLocalOffset.z;
        chunk.detached = true;
        chunk.active = true;
      }

      scheduledChildren += 1;
    }

    // If parent not reused, remove from actorMap
    if (!parentChild) {
      bridge.actorMap.delete(parentActorIndex);
    }
  }

  // Schedule safe frames and reset state
  bridge.safeFrames = Math.max(bridge.safeFrames ?? 0, 2);
  bridge.pendingContactForces.clear();

  return scheduledChildren;
}

export function applyPendingMigrations(bridge: BridgeCore): { bodiesCreated: number; collidersMigrated: number } {
  const { world } = bridge;
  const R = world.RAPIER;

  let bodiesCreated = 0;
  let collidersMigrated = 0;

  // Phase 1: Create deferred bodies
  if (bridge.pendingBodiesToCreate.length > 0) {
    const list = bridge.pendingBodiesToCreate;
    bridge.pendingBodiesToCreate = [];

    for (const pb of list) {
      const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
      const desc = R.RigidBodyDesc.dynamic();

      if (inherit) {
        const pt = inherit.translation();
        const pq = inherit.rotation();
        const lv = inherit.linvel();
        const av = inherit.angvel();
        desc
          .setTranslation(pt.x, pt.y, pt.z)
          .setRotation(pq)
          .setLinvel(lv.x, lv.y, lv.z)
          .setAngvel(av.x, av.y, av.z);
        
        console.log('applyPendingMigrations desc', desc);
        console.log('applyPendingMigrations pt', pt);
      }

      const body = world.createRigidBody(desc);
      bridge.actorMap.set(pb.actorIndex, { bodyHandle: body.handle });
      bodiesCreated += 1;

      // Schedule collider migrations
      for (const nodeIndex of pb.nodes) {
        bridge.pendingColliderMigrations.push({
          nodeIndex,
          targetBodyHandle: body.handle
        });
      }
    }
  }

  // Phase 2: Migrate colliders
  if (bridge.pendingColliderMigrations.length > 0) {
    const pending = bridge.pendingColliderMigrations;
    bridge.pendingColliderMigrations = [];

    for (const mig of pending) {
      const chunk = getChunkByNode(bridge, mig.nodeIndex);
      if (!chunk || chunk.isSupport) continue;

      // Disable old collider
      if (chunk.colliderHandle !== null) {
        const oldC = world.getCollider(chunk.colliderHandle);
        if (oldC) oldC.setEnabled(false);
        bridge.activeContactColliders.delete(chunk.colliderHandle);
        bridge.colliderToNode.delete(chunk.colliderHandle);
        bridge.disabledCollidersToRemove.add(chunk.colliderHandle);
        chunk.colliderHandle = null;
      }

      // Create new collider on target body
      const halfX = chunk.size.x * 0.5;
      const halfY = chunk.size.y * 0.5;
      const halfZ = chunk.size.z * 0.5;
      const tx = chunk.baseLocalOffset.x;
      const ty = chunk.baseLocalOffset.y;
      const tz = chunk.baseLocalOffset.z;

      const body = world.getRigidBody(mig.targetBodyHandle);
      if (!body) continue;

      console.log('applyPendingMigrations create new collider', chunk.isSupport, { halfX, halfY, halfZ, tx, ty, tz });

      const desc = R.ColliderDesc.cuboid(halfX, halfY, halfZ)
        .setTranslation(tx, ty, tz)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(R.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0);

      const col = world.createCollider(desc, body);
      chunk.bodyHandle = body.handle;
      chunk.colliderHandle = col.handle;
      bridge.colliderToNode.set(col.handle, chunk.nodeIndex);
      bridge.activeContactColliders.add(col.handle);
      collidersMigrated += 1;
    }
  }

  // Phase 3: Remove disabled colliders
  if (bridge.disabledCollidersToRemove.size > 0) {
    for (const h of Array.from(bridge.disabledCollidersToRemove)) {
      const c = world.getCollider(h);
      if (c) world.removeCollider(c, false);
      bridge.disabledCollidersToRemove.delete(h);
    }
  }

  return { bodiesCreated, collidersMigrated };
}

export function pruneStaleHandles(bridge: BridgeCore): void {
  const { world, activeContactColliders, colliderToNode, pendingContactForces } = bridge;

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
}

