import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { enqueueSplitResults, applyPendingMigrations, pruneStaleHandles } from './rapierHierarchyApplier.js';

function makeChunk(nodeIndex: number, size = { x: 1, y: 1, z: 1 }) {
  return {
    nodeIndex,
    size,
    isSupport: false,
    baseLocalOffset: { x: (nodeIndex % 2) * 1.1 - 0.55, y: Math.floor(nodeIndex / 2) * 1.1 - 0.55, z: 0 },
    localOffset: { x: 0, y: 0, z: 0, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

describe('rapierHierarchyApplier two-phase migration', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('splits a 2x2 quad into two bodies and migrates colliders safely', () => {
    const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));

    // Build 4 chunks on the parent body
    const chunks = [0,1,2,3].map((n) => makeChunk(n));
    const colliderToNode = new Map<number, number>();
    const active = new Set<number>();
    for (const c of chunks) {
      const col = world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setTranslation(c.baseLocalOffset.x, c.baseLocalOffset.y, c.baseLocalOffset.z)
          .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(0.0),
        parent
      );
      c.bodyHandle = parent.handle; c.colliderHandle = col.handle;
      colliderToNode.set(col.handle, c.nodeIndex); active.add(col.handle);
    }

    const bridge: any = {
      world,
      chunks,
      actorMap: new Map([[0, { bodyHandle: parent.handle }]]),
      colliderToNode,
      activeContactColliders: active,
      pendingContactForces: new Map(),
    };

    // Phase 1: enqueue a split mapping [0,1] -> actor 1 and [2,3] -> actor 2
    const splitResults = [{ parentActorIndex: 0, children: [ { actorIndex: 1, nodes: [0,1] }, { actorIndex: 2, nodes: [2,3] } ] }];
    const scheduled = enqueueSplitResults(bridge, splitResults);
    expect(scheduled).toBe(2);
    // Mapping still points to parent until migrations apply
    expect(bridge.actorMap.get(1).bodyHandle).toEqual(parent.handle);
    expect(bridge.actorMap.get(2).bodyHandle).toEqual(parent.handle);

    // Safe step and apply migrations
    world.step();
    const res = applyPendingMigrations(bridge);
    pruneStaleHandles(bridge);
    expect(res.bodiesCreated).toBe(2);
    expect(res.collidersMigrated).toBe(4);

    const child1BodyHandle = bridge.actorMap.get(1).bodyHandle;
    const child2BodyHandle = bridge.actorMap.get(2).bodyHandle;
    expect(child1BodyHandle).toBeTypeOf('number');
    expect(child2BodyHandle).toBeTypeOf('number');
    expect(child1BodyHandle).not.toEqual(child2BodyHandle);

    // Chunks 0,1 moved to child1; 2,3 moved to child2; and colliders were recreated
    expect(bridge.chunks[0].bodyHandle).toEqual(child1BodyHandle);
    expect(bridge.chunks[1].bodyHandle).toEqual(child1BodyHandle);
    expect(bridge.chunks[2].bodyHandle).toEqual(child2BodyHandle);
    expect(bridge.chunks[3].bodyHandle).toEqual(child2BodyHandle);
    expect(bridge.chunks.every((c: any) => c.colliderHandle != null)).toBe(true);

    // Eventful steps should be stable
    world.step();
    world.step();
  });
});


