import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { enqueueSplitResults, applyPendingMigrations, pruneStaleHandles } from './rapierHierarchyApplier.js';
import { sweepBeforeEventfulStep } from './coreLogic.js';

function makeChunk(nodeIndex: number, pos: { x: number; y: number; z: number }) {
  return {
    nodeIndex,
    size: { x: 1, y: 1, z: 1 },
    isSupport: false,
    baseLocalOffset: { x: pos.x, y: pos.y, z: pos.z },
    localOffset: { x: pos.x, y: pos.y, z: pos.z, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

function buildGrid(nX: number, nY: number, spacing = 0.6) {
  const cells: Array<{ x: number; y: number; z: number }> = [];
  const halfX = (nX - 1) * spacing * 0.5;
  const halfY = (nY - 1) * spacing * 0.5;
  for (let iy = 0; iy < nY; iy++) {
    for (let ix = 0; ix < nX; ix++) {
      cells.push({ x: ix * spacing - halfX, y: iy * spacing - halfY, z: 0 });
    }
  }
  return cells;
}

describe('Eventful replay (contacts+predefined splits)', () => {
  beforeAll(async () => { await RAPIER.init(); });

  it('replays multiple splits under continuous contact events without panic for 2000 ticks', () => {
    const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    (world as any).RAPIER = RAPIER;
    const eventQueue = new RAPIER.EventQueue(true);

    // Parent dynamic body with a 4x4 = 16 chunk compound
    const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
    const grid = buildGrid(4, 4, 0.6);
    const chunks = grid.map((p, i) => makeChunk(i, p));
    const colliderToNode = new Map<number, number>();
    const active = new Set<number>();
    for (const c of chunks) {
      const col = world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.25, 0.25, 0.25).setTranslation(c.baseLocalOffset.x, c.baseLocalOffset.y, c.baseLocalOffset.z)
          .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(0.0),
        parent
      );
      c.bodyHandle = parent.handle; c.colliderHandle = col.handle;
      colliderToNode.set(col.handle, c.nodeIndex); active.add(col.handle);
    }

    // Add a projectile to hammer contacts
    const projBody = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 3, 0).setCanSleep(false));
    world.createCollider(RAPIER.ColliderDesc.ball(0.4).setFriction(1.0).setRestitution(0.0)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS).setContactForceEventThreshold(0.0), projBody);
    projBody.setLinvel({ x: 0.0, y: -1.0, z: 0.0 }, true);

    const bridge: any = {
      world,
      chunks,
      actorMap: new Map([[0, { bodyHandle: parent.handle }]]),
      colliderToNode,
      activeContactColliders: active,
      pendingContactForces: new Map(),
      pendingBodiesToCreate: [],
      pendingColliderMigrations: [],
      disabledCollidersToRemove: new Set(),
      bodiesToRemove: new Set(),
      pendingSplitResults: [],
      safeFrames: 0,
      _handleSplitEventsCount: 0,
      _splitLog: [],
      _justSplitFrames: 0
    };

    let didSplit1 = false;
    let didSplit2 = false;

    for (let i = 0; i < 2000; i++) {
      if (bridge._justSplitFrames && bridge._justSplitFrames > 0) {
        world.step();
        bridge._justSplitFrames -= 1;
        applyPendingMigrations(bridge);
        pruneStaleHandles(bridge);
      } else {
        sweepBeforeEventfulStep(bridge);
        world.step(eventQueue);

        // Drain events and trigger scripted splits upon first contacts
        let hadContact = false;
        eventQueue.drainContactForceEvents((_evt) => { hadContact = true; });

        if (!didSplit1 && i > 3) {
          // Split into four child actors over quadrants
          const q0 = [0,1,4,5];
          const q1 = [2,3,6,7];
          const q2 = [8,9,12,13];
          const q3 = [10,11,14,15];
          const split1 = [{ parentActorIndex: 0, children: [
            { actorIndex: 1001, nodes: q0 }, { actorIndex: 1002, nodes: q1 },
            { actorIndex: 1003, nodes: q2 }, { actorIndex: 1004, nodes: q3 }
          ] }];
          enqueueSplitResults(bridge, split1);
          bridge._justSplitFrames = Math.max(bridge._justSplitFrames ?? 0, 3);
          didSplit1 = true;
        } else if (didSplit1 && !didSplit2 && i > 20) {
          // Further split one quadrant
          const split2 = [{ parentActorIndex: 1002, children: [
            { actorIndex: 2001, nodes: [2,3] }, { actorIndex: 2002, nodes: [6,7] }
          ] }];
          enqueueSplitResults(bridge, split2);
          bridge._justSplitFrames = Math.max(bridge._justSplitFrames ?? 0, 3);
          didSplit2 = true;
        }
      }
    }

    // Assertions after long run
    const actorKeys = Array.from(bridge.actorMap.keys());
    expect(actorKeys.includes(1001)).toBe(true);
    // 1002 was used as a second-stage parent; it is replaced by 2001/2002
    expect(actorKeys.includes(1003)).toBe(true);
    expect(actorKeys.includes(1004)).toBe(true);
    expect(actorKeys.includes(2001)).toBe(true);
    expect(actorKeys.includes(2002)).toBe(true);
    // All chunks have colliders and valid body handles
    for (const c of bridge.chunks) {
      expect(typeof c.bodyHandle).toBe('number');
      expect(typeof c.colliderHandle).toBe('number');
    }
  });
});


