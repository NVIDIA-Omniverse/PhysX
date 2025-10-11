import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { handleSplitEvents } from './handleSplitEvents.js';

function makeChunk(nodeIndex: number, size = { x: 1, y: 1, z: 1 }) {
  return {
    nodeIndex,
    size,
    isSupport: false,
    baseLocalOffset: { x: (nodeIndex % 16) * 0.25, y: Math.floor(nodeIndex / 16) * 0.25, z: 0 },
    localOffset: { x: 0, y: 0, z: 0, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

// This test reproduces a large split similar to the browser scenario: ~96 parent chunks -> ~32 children.
describe('handleSplitEvents large bridge scenario', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('applies many child splits and survives world.step without panic', () => {
    const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));

    // Create 104 chunks total; attach first 96 to parent to reflect prevChunksOnParent: 96
    const totalNodes = 104;
    const parentCount = 96;
    const chunks = Array.from({ length: totalNodes }, (_, i) => makeChunk(i));
    const colliderToNode = new Map<number, number>();
    const active = new Set<number>();
    for (let i = 0; i < parentCount; i++) {
      const c = chunks[i];
      const col = world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setTranslation(c.baseLocalOffset.x, c.baseLocalOffset.y, c.baseLocalOffset.z),
        parent
      );
      c.bodyHandle = parent.handle; c.colliderHandle = col.handle;
      colliderToNode.set(col.handle, c.nodeIndex); active.add(col.handle);
    }

    const bridge = { world, chunks, actorMap: new Map([[0, { bodyHandle: parent.handle }]]), colliderToNode, activeContactColliders: active, solver: { actors: () => [] } };

    // Build split results similar to the example: several groups of 4 nodes, then singletons, plus parent-actor child [0,1,2,3]
    const groups: Array<{ actorIndex: number; nodes: number[] }> = [];
    let actor = 7;
    for (let start = 4; start < 96; start += 4) {
      groups.push({ actorIndex: actor, nodes: [start, start+1, start+2, start+3] });
      actor += 4;
    }
    // singletons from 96..103
    for (let n = 96; n < 104; n++) {
      groups.push({ actorIndex: actor++, nodes: [n] });
    }
    // include parent actor as a child for [0,1,2,3]
    groups.push({ actorIndex: 0, nodes: [0,1,2,3] });

    const splitResults = [{ parentActorIndex: 0, children: groups }];

    // Apply splits and step the world a few frames to trigger BVH rebuilds
    handleSplitEvents(bridge as any, splitResults as any, RAPIER, 0.0);
    for (let i = 0; i < 3; i++) {
      world.step();
    }

    // Basic assertions: many child actors are present and chunks re-mapped
    const childActorIndices = Array.from(bridge.actorMap.keys()).filter((k) => k !== 0);
    expect(childActorIndices.length).toBeGreaterThan(10);
    // ensure some known chunk moved
    expect(bridge.chunks[4].bodyHandle).not.toEqual(parent.handle);
  });
});


