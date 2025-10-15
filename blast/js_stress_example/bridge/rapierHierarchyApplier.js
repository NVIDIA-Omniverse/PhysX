// Reusable, testable Rapier split-applier with two-phase migration.
// Phase 1 (enqueueSplitResults): schedule body creations and mark chunks detached without touching colliders.
// Phase 2 (applyPendingMigrations): create bodies, migrate colliders, prune disabled ones.

function getChunkByNode(bridge, nodeIndex) {
  const list = bridge.chunks ?? [];
  for (let i = 0; i < list.length; i++) {
    const c = list[i];
    if (c && c.nodeIndex === nodeIndex) return c;
  }
  return null;
}

export function enqueueSplitResults(bridge, splitResults) {
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return 0;
  }

  if (!Array.isArray(bridge.pendingBodiesToCreate)) bridge.pendingBodiesToCreate = [];
  if (!Array.isArray(bridge.pendingColliderMigrations)) bridge.pendingColliderMigrations = [];

  let scheduledChildren = 0;

  splitResults.forEach((evt) => {
    const parentActorIndex = evt?.parentActorIndex;
    if (!Number.isInteger(parentActorIndex)) return;

    const parentEntry = bridge.actorMap?.get(parentActorIndex);
    if (!parentEntry) return;

    const parentBodyHandle = parentEntry.bodyHandle;
    const parentChild = (evt.children ?? []).find((c) => c && c.actorIndex === parentActorIndex);

    (evt.children ?? []).forEach((child) => {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) return;

      // Queue body creation; initially map child actor to parent body (safe until Phase 2)
      bridge.pendingBodiesToCreate.push({
        actorIndex: child.actorIndex,
        inheritFromBodyHandle: parentBodyHandle,
        nodes: child.nodes.slice()
      });
      if (bridge.actorMap) bridge.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });

      // Mark chunks detached; keep current collider for now
      child.nodes.forEach((nodeIndex) => {
        const chunk = getChunkByNode(bridge, nodeIndex);
        if (!chunk || chunk.isSupport) return;
        if (chunk.localOffset && chunk.baseLocalOffset) {
          chunk.localOffset.copy?.(chunk.baseLocalOffset);
        }
        chunk.detached = true;
        chunk.active = true;
      });

      scheduledChildren += 1;
    });

    // If parent isn't reused as a child, drop its actorMap entry (body removal is deferred)
    if (!parentChild) {
      bridge.actorMap?.delete?.(parentActorIndex);
    }
  });

  // Run at least two safe steps before next eventful step
  bridge._justSplitFrames = Math.max(bridge._justSplitFrames ?? 0, 2);

  // Drop stale contact forces/events accumulated with old handles
  try { bridge.pendingContactForces?.clear?.(); } catch (_) {}

  return scheduledChildren;
}

export function applyPendingMigrations(bridge) {
  const world = bridge.world;
  const R = world?.RAPIER;
  if (!world || !R) return { bodiesCreated: 0, collidersMigrated: 0 };

  let bodiesCreated = 0;
  let collidersMigrated = 0;

  // Create deferred bodies and schedule collider migrations
  if (Array.isArray(bridge.pendingBodiesToCreate) && bridge.pendingBodiesToCreate.length > 0) {
    const list = bridge.pendingBodiesToCreate;
    bridge.pendingBodiesToCreate = [];
    for (const pb of list) {
      const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
      const desc = R.RigidBodyDesc.dynamic();
      if (inherit) {
        const pt = inherit.translation(); const pq = inherit.rotation();
        const lv = inherit.linvel(); const av = inherit.angvel();
        desc.setTranslation(pt.x, pt.y, pt.z).setRotation(pq)
          .setLinvel(lv.x, lv.y, lv.z)
          .setAngvel(av.x, av.y, av.z);
      }
      const body = world.createRigidBody(desc);
      if (bridge.actorMap) bridge.actorMap.set(pb.actorIndex, { bodyHandle: body.handle });
      bodiesCreated += 1;

      // Schedule collider migrations for nodes of this new actor
      if (!Array.isArray(bridge.pendingColliderMigrations)) bridge.pendingColliderMigrations = [];
      for (const nodeIndex of (pb.nodes || [])) {
        bridge.pendingColliderMigrations.push({ nodeIndex, targetBodyHandle: body.handle });
      }
    }
  }

  // Apply collider migrations
  if (Array.isArray(bridge.pendingColliderMigrations) && bridge.pendingColliderMigrations.length > 0) {
    const pending = bridge.pendingColliderMigrations;
    bridge.pendingColliderMigrations = [];
    if (!bridge.disabledCollidersToRemove) bridge.disabledCollidersToRemove = new Set();

    for (const mig of pending) {
      const chunk = getChunkByNode(bridge, mig.nodeIndex);
      if (!chunk || chunk.isSupport) continue;

      // Disable old collider and queue for removal
      if (chunk.colliderHandle != null) {
        const oldC = world.getCollider(chunk.colliderHandle);
        if (oldC) oldC.setEnabled(false);
        bridge.activeContactColliders?.delete?.(chunk.colliderHandle);
        bridge.colliderToNode?.delete?.(chunk.colliderHandle);
        bridge.disabledCollidersToRemove.add(chunk.colliderHandle);
        chunk.colliderHandle = null;
      }

      // Create new collider on target body at base local offset
      const halfX = (chunk.size?.x ?? 1) * 0.5;
      const halfY = (chunk.size?.y ?? 1) * 0.5;
      const halfZ = (chunk.size?.z ?? 1) * 0.5;
      const tx = chunk.baseLocalOffset?.x ?? 0;
      const ty = chunk.baseLocalOffset?.y ?? 0;
      const tz = chunk.baseLocalOffset?.z ?? 0;
      const body = world.getRigidBody(mig.targetBodyHandle);
      if (!body) continue;
      const desc = R.ColliderDesc.cuboid(halfX, halfY, halfZ)
        .setTranslation(tx, ty, tz)
        .setFriction(1.0)
        .setRestitution(0.0)
        .setActiveEvents(R.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0);
      const col = world.createCollider(desc, body);
      chunk.bodyHandle = body.handle;
      chunk.colliderHandle = col.handle;
      bridge.colliderToNode?.set?.(col.handle, chunk.nodeIndex);
      bridge.activeContactColliders?.add?.(col.handle);
      collidersMigrated += 1;
    }
  }

  // Physically remove disabled colliders now that new ones exist
  if (bridge.disabledCollidersToRemove && bridge.disabledCollidersToRemove.size > 0) {
    for (const h of Array.from(bridge.disabledCollidersToRemove)) {
      const c = world.getCollider(h);
      if (c) world.removeCollider(c, false);
      bridge.disabledCollidersToRemove.delete(h);
    }
  }

  return { bodiesCreated, collidersMigrated };
}

export function pruneStaleHandles(bridge) {
  try {
    const world = bridge.world;
    if (!world) return;
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
  } catch (_) {}
}


