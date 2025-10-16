import { beforeAll, describe, expect, it } from 'vitest';
import { buildBridgeShared } from '../bridge/buildBridge.headless.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces, sweepBeforeEventfulStep } from '../bridge/coreLogic.js';
import { applyPendingMigrations, pruneStaleHandles, enqueueSplitResults } from './rapierHierarchyApplier.js';
import { spawnLoadVehicle, spawnProjectile, updateProjectiles } from '../bridge/dynamics.js';
import type { BridgeCore } from './types.js';

describe('Organic split flow (no manual handleSplitEvents)', () => {
  beforeAll(async () => {
    // RAPIER is in bridge-sim builder
  });

  it('produces deterministic split events that match expected patterns', async () => {
    const bridge: BridgeCore = await buildBridgeShared({ gravity: -9.81, strengthScale: 0.02 });
    const projectiles: any[] = [];
    const car = spawnLoadVehicle(bridge.world, {});

    // Fixed timestep and seeded randomness for determinism
    const dt = 1/60;
    const seed = 1337;
    let rand = seed;
    const rand01 = () => { rand = (rand * 1664525 + 1013904223) >>> 0; return (rand & 0xffff) / 0x10000; };

    let handleCount = 0;
    for (let i = 0; i < 1800; i++) {
      const shouldSpawn = (bridge._handleSplitEventsCount >= 1 && i % 30 === 0);

      // Safe frame: no events, apply migrations
      if (bridge._justSplitFrames > 0) {
        try {
          bridge.world.step();
        } catch (error) {
          console.error('Safe step error:', error);
          bridge._justSplitFrames = Math.max(bridge._justSplitFrames, 1);
          continue;
        }

        bridge._justSplitFrames -= 1;

        // Process pending splits
        if (bridge.pendingSplitResults.length > 0) {
          const splits = bridge.pendingSplitResults;
          bridge.pendingSplitResults = [];
          enqueueSplitResults(bridge, splits);
        }

        applyPendingMigrations(bridge);

        // Remove disabled handles
        if (bridge.disabledCollidersToRemove.size > 0) {
          for (const h of Array.from(bridge.disabledCollidersToRemove)) {
            const c = bridge.world.getCollider(h);
            if (c) bridge.world.removeCollider(c, false);
            bridge.disabledCollidersToRemove.delete(h);
          }
        }
        if (bridge.bodiesToRemove.size > 0) {
          for (const bh of Array.from(bridge.bodiesToRemove)) {
            const b = bridge.world.getRigidBody(bh);
            if (b) bridge.world.removeRigidBody(b);
            bridge.bodiesToRemove.delete(bh);
          }
        }

        // Rebuild contact tracking
        bridge.activeContactColliders.clear();
        bridge.colliderToNode.clear();
        for (const chunk of bridge.chunks) {
          const h = chunk.colliderHandle;
          if (h !== null) {
            const c = bridge.world.getCollider(h);
            const p = c?.parent();
            const b = p !== undefined ? bridge.world.getRigidBody(p) : undefined;
            const enabled = c?.isEnabled ? c.isEnabled() : true;
            if (c && enabled && b) {
              bridge.activeContactColliders.add(h);
              bridge.colliderToNode.set(h, chunk.nodeIndex);
            }
          }
        }

        continue;
      }

      // Eventful frame: step with events
      sweepBeforeEventfulStep(bridge);

      let stepped = false;
      try {
        bridge.world.step(bridge.eventQueue);
        stepped = true;
      } catch (err) {
        console.error('Eventful step error:', err);
        bridge.eventQueue = new bridge.world.RAPIER.EventQueue(true);
        bridge._justSplitFrames = Math.max(bridge._justSplitFrames, 1);
      }

      if (!stepped) continue;

      pruneStaleHandles(bridge);
      drainContactForces(bridge);

      // Spawn projectiles
      if (shouldSpawn) {
        projectiles.push(spawnProjectile(bridge.world, { kind: 'box', origin: { x: 0, y: 6, z: 0 }, scale: 1.0 }));
      }
      const updatedProjectiles = updateProjectiles(bridge.world, projectiles, dt, bridge);
      projectiles.length = 0;
      projectiles.push(...updatedProjectiles);

      applyForcesAndSolve(bridge);

      if (bridge.overstressed > 0) {
        const result = processSolverFractures(bridge, bridge.world.RAPIER, 0.0);
        if (result?.splitResults) {
          bridge.pendingSplitResults.push(...result.splitResults);
          bridge._justSplitFrames = Math.max(bridge._justSplitFrames, 3);
        }
      }

      handleCount = bridge._handleSplitEventsCount;
      if (handleCount >= 2) break;
    }

    expect(handleCount).toBeGreaterThanOrEqual(1);

    // Assert normalized split log against a stable expectation schema (shape-only)
    expect(Array.isArray(bridge._splitLog)).toBe(true);
    const flat = bridge._splitLog.flat();
    expect(flat.length).toBeGreaterThan(0);
    
    // Validate structure
    for (const evt of flat) {
      expect(typeof evt.parentActorIndex).toBe('number');
      expect(Array.isArray(evt.children)).toBe(true);
      for (const c of evt.children) {
        expect(typeof c.actorIndex).toBe('number');
        expect(Array.isArray(c.nodes)).toBe(true);
        // nodes are sorted
        for (let k = 1; k < c.nodes.length; k++) {
          expect(c.nodes[k]).toBeGreaterThanOrEqual(c.nodes[k-1]);
        }
      }
    }

    // Optional: compare to a predefined snapshot shape by counts
    const summary = flat.map((evt) => ({ parent: evt.parentActorIndex, childCount: evt.children.length }));
    expect(summary.length).toBeGreaterThan(0);

    // Shape-specific assertion for first split: [0..3] then quads of 4 until 95, then 8 singletons [96..103]
    const evt0 = bridge._splitLog[0]?.[0];
    expect(evt0?.parentActorIndex).toBe(0);
    const kids = evt0?.children ?? [];
    // total children = 1 (parent group) + 23 quad groups + 8 singletons = 32
    expect(kids.length).toBe(32);
    // First child: actor 0 with [0,1,2,3]
    expect(kids[0].actorIndex).toBe(0);
    expect(kids[0].nodes).toEqual([0,1,2,3]);
    // Next 23 children: quads [4..7], [8..11], ..., [92..95]
    for (let gi = 0; gi < 23; gi++) {
      const child = kids[1 + gi];
      const start = 4 + gi * 4;
      expect(child.nodes).toEqual([start, start+1, start+2, start+3]);
    }
    // Last 8 children: singletons 96..103
    for (let si = 0; si < 8; si++) {
      const child = kids[1 + 23 + si];
      expect(child.nodes).toEqual([96 + si]);
    }
  });
});


