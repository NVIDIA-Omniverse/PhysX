// Core split event handler, extracted for integration testing.
// Depends only on Rapier world and plain bridge state, no DOM/Three.

export function handleSplitEvents(bridge, splitResults, RAPIER, contactForceThreshold = 0.0) {
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return;
  }

  const chunkByNode = new Map();
  (bridge.chunks ?? []).forEach((chunk) => {
    if (chunk) chunkByNode.set(chunk.nodeIndex, chunk);
  });

  const rebuildMode = !!bridge._rebuildOnSplit;
  // Initialize deferred queues when not rebuilding
  if (!rebuildMode && !Array.isArray(bridge.pendingBodiesToCreate)) bridge.pendingBodiesToCreate = [];
  if (!rebuildMode && !Array.isArray(bridge.pendingColliderMigrations)) bridge.pendingColliderMigrations = [];

  splitResults.forEach((evt) => {
    const parentActorIndex = evt?.parentActorIndex;
    if (!Number.isInteger(parentActorIndex)) {
      return;
    }

    const parentEntry = bridge.actorMap?.get(parentActorIndex);
    if (!parentEntry) {
      return;
    }

    const parentBodyHandle = parentEntry.bodyHandle;
    const parentBody = parentBodyHandle != null ? bridge.world.getRigidBody(parentBodyHandle) : null;

    const prevChunksOnParent = [];
    (bridge.chunks ?? []).forEach((c) => {
      if (c && c.bodyHandle === parentBodyHandle) prevChunksOnParent.push(c.nodeIndex);
    });

    const assignedNodes = new Set();
    const parentChild = (evt.children ?? []).find((c) => c && c.actorIndex === parentActorIndex);

    evt.children?.forEach((child) => {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) {
        return;
      }

      let bodyHandle = null;
      let body = null;

      if (!rebuildMode) {
        if (parentChild && child.actorIndex === parentActorIndex) {
          body = parentBody;
          bodyHandle = parentBodyHandle;
        } else {
          bodyHandle = bridge.actorMap?.get(child.actorIndex)?.bodyHandle ?? null;
        }

        if (bodyHandle != null) {
          body = bridge.world.getRigidBody(bodyHandle);
        }
      }

      if (!rebuildMode) {
        // Phase 1 (split frame, no RB creation/removal): queue body creation; keep mapping to parent
        bridge.pendingBodiesToCreate.push({
          actorIndex: child.actorIndex,
          inheritFromBodyHandle: parentBodyHandle,
          nodes: Array.isArray(child.nodes) ? child.nodes.slice() : []
        });
        if (bridge.actorMap) {
          bridge.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
        }
      } else {
        // In rebuild mode, we only update the logical mapping; colliders/bodies will be recreated in the new world
        if (bridge.actorMap) {
          bridge.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
        }
      }

      child.nodes.forEach((nodeIndex) => {
        if (assignedNodes.has(nodeIndex)) {
          return;
        }
        assignedNodes.add(nodeIndex);
        const chunk = chunkByNode.get(nodeIndex);
        if (!chunk) {
          return;
        }
        if (chunk.isSupport) {
          return;
        }

        if (rebuildMode) {
          // Mark as detached only; new world will own collider creation
          chunk.detached = true;
          chunk.active = true;
          // Clear old handle to avoid accidental reuse
          chunk.colliderHandle = null;
        } else {
          // Phase 1: don’t migrate colliders yet; keep them on the parent, but mark chunk as detached
          if (chunk.localOffset && chunk.baseLocalOffset) {
            chunk.localOffset.copy?.(chunk.baseLocalOffset);
          }
          chunk.detached = true;
          chunk.active = true;
          // Keep chunk.bodyHandle as-is until Phase 2 when the new body exists
        }
      });
    });

    // Do not remove the parent rigid body immediately.
    // Removing here can leave Rapier with stale contact references into the next step.
    // We rely on later cleanup (reset/rebuild) to remove orphan bodies safely.
    if (!parentChild) {
      bridge.actorMap?.delete?.(parentActorIndex);
    }
  });
}


