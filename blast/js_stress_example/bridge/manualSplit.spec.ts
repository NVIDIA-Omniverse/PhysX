import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { enqueueSplitResults, applyPendingMigrations, pruneStaleHandles } from './rapierHierarchyApplier.js';

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

function buildInitialWorld(grid: Array<{ x: number; y: number; z: number }>) {
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  (world as any).RAPIER = RAPIER;
  const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
  const chunks = grid.map((p, i) => makeChunk(i, p));
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
  return { world, parent, bridge };
}

async function runTicks(world: any, ticks: number, postStep?: () => void) {
  for (let i = 0; i < ticks; i++) {
    world.step();
    if (postStep) postStep();
  }
}

describe('Manual split fixtures (solver-free)', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('cube4_single: one split into two children and remains stable for 600 ticks', async () => {
    const __filename = fileURLToPath(import.meta.url);
    const __dirname = dirname(__filename);
    const fx = JSON.parse(readFileSync(resolve(__dirname, '../tests/fixtures/manual_splits/cube4_single.json'), 'utf-8'));

    const { world, parent, bridge } = buildInitialWorld(fx.initial.grid);
    const scheduled = enqueueSplitResults(bridge, fx.splits);
    expect(scheduled).toBe(2);

    // Phase 2 after safe step
    await runTicks(world, 1);
    const res = applyPendingMigrations(bridge);
    pruneStaleHandles(bridge);
    expect(res.bodiesCreated).toBe(2);
    expect(res.collidersMigrated).toBe(4);

    // Assert children exist with distinct bodies
    const childActors = Array.from(bridge.actorMap.keys()).filter((k) => k !== 0);
    expect(childActors.length).toBe(2);
    const h1 = bridge.actorMap.get(childActors[0]).bodyHandle;
    const h2 = bridge.actorMap.get(childActors[1]).bodyHandle;
    expect(h1).toBeTypeOf('number'); expect(h2).toBeTypeOf('number');
    expect(h1).not.toEqual(h2);
    expect(bridge.chunks.every((c: any) => c.colliderHandle != null)).toBe(true);

    // Let it run long enough to catch BVH panics
    await runTicks(world, 600, () => pruneStaleHandles(bridge));
  });

  it('cube4_multi: multiple sequential splits and remains stable for 900 ticks', async () => {
    const __filename = fileURLToPath(import.meta.url);
    const __dirname = dirname(__filename);
    const fx = JSON.parse(readFileSync(resolve(__dirname, '../tests/fixtures/manual_splits/cube4_multi.json'), 'utf-8'));

    const { world, bridge } = buildInitialWorld(fx.initial.grid);

    // Apply each split with proper two-phase handling
    for (const evt of fx.splits) {
      const scheduled = enqueueSplitResults(bridge, [evt]);
      expect(scheduled).toBeGreaterThan(0);
      await runTicks(world, 1);
      applyPendingMigrations(bridge);
      pruneStaleHandles(bridge);
    }

    // Validate unique bodies per child actor (final children: 1,3,5,6)
    const childActors = Array.from(bridge.actorMap.keys()).filter((k) => k !== 0);
    expect(childActors.length).toBe(4);
    const bodyHandles = childActors.map((a) => bridge.actorMap.get(a).bodyHandle);
    const unique = new Set(bodyHandles);
    expect(unique.size).toBe(4);
    expect(bridge.chunks.every((c: any) => c.colliderHandle != null)).toBe(true);

    await runTicks(world, 900, () => pruneStaleHandles(bridge));
  });
});


