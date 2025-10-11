import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { handleSplitEvents } from './handleSplitEvents.js';

function makeChunk(nodeIndex: number, size = { x: 1, y: 1, z: 1 }, isSupport = false) {
  return {
    nodeIndex,
    size,
    isSupport,
    baseLocalOffset: { x: nodeIndex, y: 0, z: 0 },
    localOffset: { x: nodeIndex, y: 0, z: 0, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

describe('handleSplitEvents (Rapier integration)', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('migrates colliders from parent body to new child bodies and updates actorMap', () => {
    const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });

    // Create parent body with 4 chunk colliders
    const parentBody = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0)
    );

    const chunks = [0,1,2,3].map((n) => makeChunk(n));
    const colliderToNode = new Map<number, number>();
    const activeContactColliders = new Set<number>();

    chunks.forEach((chunk) => {
      const col = world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.5,0.5,0.5).setTranslation(chunk.baseLocalOffset.x, 0, 0),
        parentBody
      );
      chunk.bodyHandle = parentBody.handle;
      chunk.colliderHandle = col.handle;
      colliderToNode.set(col.handle, chunk.nodeIndex);
      activeContactColliders.add(col.handle);
    });

    const bridge = {
      world,
      chunks,
      actorMap: new Map<number, { bodyHandle: number }>([[0, { bodyHandle: parentBody.handle }]]),
      colliderToNode,
      activeContactColliders,
      solver: { actors: () => [] }
    };

    const splitResults = [
      { parentActorIndex: 0, children: [ { actorIndex: 1, nodes: [0,1] }, { actorIndex: 2, nodes: [2,3] } ] }
    ];

    handleSplitEvents(bridge as any, splitResults as any, RAPIER, 0.0);

    // Assert: two new bodies exist and chunks moved
    const child1BodyHandle = bridge.actorMap.get(1)!.bodyHandle;
    const child2BodyHandle = bridge.actorMap.get(2)!.bodyHandle;
    expect(child1BodyHandle).toBeTypeOf('number');
    expect(child2BodyHandle).toBeTypeOf('number');
    expect(child1BodyHandle).not.toEqual(child2BodyHandle);

    // Chunks [0,1] should now be on child1, [2,3] on child2, with new colliders
    expect(bridge.chunks[0].bodyHandle).toEqual(child1BodyHandle);
    expect(bridge.chunks[1].bodyHandle).toEqual(child1BodyHandle);
    expect(bridge.chunks[2].bodyHandle).toEqual(child2BodyHandle);
    expect(bridge.chunks[3].bodyHandle).toEqual(child2BodyHandle);

    expect(bridge.chunks[0].colliderHandle).not.toBeNull();
    expect(bridge.chunks[1].colliderHandle).not.toBeNull();
    expect(bridge.chunks[2].colliderHandle).not.toBeNull();
    expect(bridge.chunks[3].colliderHandle).not.toBeNull();

    // Parent body remains only if parent appears as child; here it does not, so parent may be removed
    const maybeParent = world.getRigidBody(parentBody.handle);
    // either null (removed) or still present if some chunk still references it
    expect([null, parentBody.handle]).toContain(maybeParent ? maybeParent.handle : null);
  });
});


