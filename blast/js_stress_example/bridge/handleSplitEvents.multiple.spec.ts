import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { handleSplitEvents } from './handleSplitEvents.js';

function makeChunk(nodeIndex: number, size = { x: 1, y: 1, z: 1 }, isSupport = false) {
  return {
    nodeIndex,
    size,
    isSupport,
    baseLocalOffset: { x: nodeIndex * 0.25, y: 0, z: 0 },
    localOffset: { x: nodeIndex * 0.25, y: 0, z: 0, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

describe('handleSplitEvents scenarios', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('keeps parent body when parent actor appears among children', () => {
    const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    const chunks = [0,1,2,3].map((n) => makeChunk(n));
    const colliderToNode = new Map<number, number>();
    const active = new Set<number>();
    chunks.forEach((c) => {
      const col = world.createCollider(RAPIER.ColliderDesc.cuboid(0.5,0.5,0.5).setTranslation(c.baseLocalOffset.x, 0, 0), parent);
      c.bodyHandle = parent.handle; c.colliderHandle = col.handle;
      colliderToNode.set(col.handle, c.nodeIndex); active.add(col.handle);
    });
    const bridge = { world, chunks, actorMap: new Map([[0, { bodyHandle: parent.handle }]]), colliderToNode, activeContactColliders: active, solver: { actors: () => [] } };
    const splitResults = [{ parentActorIndex: 0, children: [ { actorIndex: 0, nodes: [0] }, { actorIndex: 2, nodes: [1,2,3] } ] }];

    handleSplitEvents(bridge as any, splitResults as any, RAPIER, 0.0);

    // Parent should remain mapped and exist
    expect(bridge.actorMap.get(0)?.bodyHandle).toEqual(parent.handle);
    expect(world.getRigidBody(parent.handle)).not.toBeNull();
    // Child actor 2 must have a new body and own nodes 1,2,3
    const childBody = bridge.actorMap.get(2)?.bodyHandle!;
    expect(childBody).toBeTypeOf('number');
    [1,2,3].forEach((n) => expect(bridge.chunks[n].bodyHandle).toEqual(childBody));
  });

  it('skips support nodes and does not move their colliders', () => {
    const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    const chunks = [makeChunk(0), makeChunk(1, { x:1,y:1,z:1 }, true)];
    const c0 = world.createCollider(RAPIER.ColliderDesc.cuboid(0.5,0.5,0.5).setTranslation(0,0,0), parent);
    const c1 = world.createCollider(RAPIER.ColliderDesc.cuboid(0.5,0.5,0.5).setTranslation(0.25,0,0), parent);
    chunks[0].bodyHandle = parent.handle; chunks[0].colliderHandle = c0.handle;
    chunks[1].bodyHandle = parent.handle; chunks[1].colliderHandle = c1.handle; // support
    const colliderToNode = new Map<number, number>([[c0.handle,0],[c1.handle,1]]);
    const active = new Set<number>([c0.handle,c1.handle]);
    const bridge = { world, chunks, actorMap: new Map([[0,{ bodyHandle: parent.handle }]]), colliderToNode, activeContactColliders: active, solver: { actors: () => [] } };
    const splitResults = [{ parentActorIndex: 0, children: [ { actorIndex: 2, nodes: [0,1] } ] }];

    handleSplitEvents(bridge as any, splitResults as any, RAPIER, 0.0);

    // Node 0 moved to child; node 1 (support) should remain on parent
    const childBody = bridge.actorMap.get(2)?.bodyHandle!;
    expect(bridge.chunks[0].bodyHandle).toEqual(childBody);
    expect(bridge.chunks[1].bodyHandle).toEqual(parent.handle);
  });

  it('ignores empty children and duplicates', () => {
    const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    const chunks = [0,1,2].map((n) => makeChunk(n));
    chunks.forEach((c) => {
      const col = world.createCollider(RAPIER.ColliderDesc.cuboid(0.5,0.5,0.5).setTranslation(c.baseLocalOffset.x, 0, 0), parent);
      c.bodyHandle = parent.handle; c.colliderHandle = col.handle;
    });
    const bridge = { world, chunks, actorMap: new Map([[0,{ bodyHandle: parent.handle }]]), colliderToNode: new Map(), activeContactColliders: new Set(), solver: { actors: () => [] } };
    const splitResults = [{ parentActorIndex: 0, children: [ { actorIndex: 1, nodes: [] }, { actorIndex: 2, nodes: [0,0,1] } ] }];

    handleSplitEvents(bridge as any, splitResults as any, RAPIER, 0.0);

    const child2 = bridge.actorMap.get(2)?.bodyHandle!;
    expect(bridge.chunks[0].bodyHandle).toEqual(child2);
    expect(bridge.chunks[1].bodyHandle).toEqual(child2);
    // node 2 remains on parent
    expect(bridge.chunks[2].bodyHandle).toEqual(parent.handle);
  });
});


