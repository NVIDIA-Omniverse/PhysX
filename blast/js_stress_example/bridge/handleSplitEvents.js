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
        if (!body) {
          const rapier = bridge.world?.RAPIER ?? RAPIER;
          const spawnDesc = rapier.RigidBodyDesc.dynamic();
          if (parentBody) {
            const pt = parentBody.translation();
            const pq = parentBody.rotation();
            spawnDesc
              .setTranslation(pt.x, pt.y, pt.z)
              .setRotation(pq)
              .setLinvel(parentBody.linvel().x, parentBody.linvel().y, parentBody.linvel().z)
              .setAngvel(parentBody.angvel().x, parentBody.angvel().y, parentBody.angvel().z);
          }

          body = bridge.world.createRigidBody(spawnDesc);
          bodyHandle = body.handle;
          if (bridge.actorMap) {
            bridge.actorMap.set(child.actorIndex, { bodyHandle });
          }
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
          if (chunk.colliderHandle != null) {
            const oldHandle = chunk.colliderHandle;
            const collider = bridge.world.getCollider(oldHandle);
            if (collider) {
              collider.setEnabled(false);
            }
            bridge.activeContactColliders?.delete?.(oldHandle);
            bridge.colliderToNode?.delete?.(oldHandle);
            if (!bridge.disabledCollidersToRemove) bridge.disabledCollidersToRemove = new Set();
            bridge.disabledCollidersToRemove.add(oldHandle);
            chunk.colliderHandle = null;
          }

          if (chunk.localOffset && chunk.baseLocalOffset) {
            chunk.localOffset.copy?.(chunk.baseLocalOffset);
          }

          const halfX = (chunk.size?.x ?? 1) * 0.5;
          const halfY = (chunk.size?.y ?? 1) * 0.5;
          const halfZ = (chunk.size?.z ?? 1) * 0.5;
          const tx = chunk.baseLocalOffset?.x ?? 0;
          const ty = chunk.baseLocalOffset?.y ?? 0;
          const tz = chunk.baseLocalOffset?.z ?? 0;

          const colliderDesc = RAPIER.ColliderDesc.cuboid(halfX, halfY, halfZ)
            .setTranslation(tx, ty, tz)
            .setFriction(1.0)
            .setRestitution(0.0)
            .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
            .setContactForceEventThreshold(contactForceThreshold);

          const collider = bridge.world.createCollider(colliderDesc, body);
          chunk.bodyHandle = bodyHandle;
          chunk.colliderHandle = collider.handle;
          chunk.detached = true;
          chunk.active = true;
          bridge.colliderToNode?.set?.(collider.handle, chunk.nodeIndex);
          bridge.activeContactColliders?.add?.(collider.handle);
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


