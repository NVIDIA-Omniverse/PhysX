import { beforeAll, describe, expect, it } from 'vitest';
import { buildBridgeShared } from '../bridge/buildBridge.headless.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces, normalizeSplitResults, sweepBeforeEventfulStep } from '../bridge/coreLogic.js';
import { applyPendingMigrations, pruneStaleHandles } from './rapierHierarchyApplier.js';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { spawnLoadVehicle, updateLoadVehicle, spawnProjectile, updateProjectiles } from '../bridge/dynamics.js';

describe('Organic split flow (no manual handleSplitEvents)', () => {
  beforeAll(async () => {
    // RAPIER is in bridge-sim builder
  });

  it('produces deterministic split events that match expected patterns', async () => {
    const bridge = await buildBridgeShared({ gravity: -9.81, strengthScale: 0.02 });
    bridge.world.RAPIER = (await import('@dimforge/rapier3d-compat')).default;
    bridge.projectiles = [];
    bridge.car = spawnLoadVehicle(bridge.world, {});
    // Enable full Rapier rebuild after split to avoid BVH edge cases
    (bridge as any)._rebuildOnSplit = false;

    // Fixed timestep and seeded randomness for determinism
    const dt = 1/60;
    const seed = 1337;
    let rand = seed;
    const rand01 = () => { rand = (rand * 1664525 + 1013904223) >>> 0; return (rand & 0xffff) / 0x10000; };

    let handleCount = 0;
    for (let i = 0; i < 1800; i++) {
      // Decide spawns; perform after step to avoid aliasing
      const shouldSpawn = ((bridge._handleSplitEventsCount ?? 0) >= 1 && (i % 30 === 0));
      // Defer projectile/world reads that call body.translation() until after stepping
      // updateLoadVehicle is also deferred to post-step

      // Safety sweep before stepping (skip during safe-step frames to avoid aliasing)
      if (!((bridge as any)._justSplitFrames && (bridge as any)._justSplitFrames > 0)) {
        if (bridge.activeContactColliders && bridge.colliderToNode) {
          const stale = [] as number[];
          bridge.activeContactColliders.forEach((h: number) => {
            const col = bridge.world.getCollider(h);
            if (!col || !col.parent()) stale.push(h);
          });
          stale.forEach((h: number) => {
            const col = bridge.world.getCollider(h);
            if (col) col.setEnabled(false);
            bridge.activeContactColliders.delete(h);
            bridge.colliderToNode.delete(h);
          });
        }

        // Extra safety: ensure all colliders referenced by activeContactColliders exist and have parents
        if (bridge.activeContactColliders && bridge.colliderToNode) {
          const stale = [] as number[];
          bridge.activeContactColliders.forEach((h: number) => {
            const col = bridge.world.getCollider(h);
            if (!col || !col.parent()) stale.push(h);
          });
          stale.forEach((h: number) => { bridge.activeContactColliders.delete(h); bridge.colliderToNode.delete(h); });
        }
      }

      // Use the potentially swapped world after rebuild
      // Match browser safe-step semantics: if just split, safe step once; else step with events
      if ((bridge as any)._justSplitFrames && (bridge as any)._justSplitFrames > 0) {
        // Safe-step without events to avoid any iterator/borrow aliasing
        try {
          (bridge.world as any).step();
        } catch (error) {
          console.error('error 1:', error);
          // If even a safe-step panics, skip this frame and try again
          (bridge as any)._justSplitFrames = Math.max((bridge as any)._justSplitFrames ?? 0, 1);
          continue;
        }
        (bridge as any)._justSplitFrames -= 1;
        applyPendingMigrations(bridge as any);
        // Remove deferred bodies/colliders in safe frames
        try {
          if ((bridge as any).disabledCollidersToRemove && (bridge as any).disabledCollidersToRemove.size > 0) {
            for (const h of Array.from((bridge as any).disabledCollidersToRemove)) {
              const c = (bridge.world as any).getCollider(h);
              if (c) (bridge.world as any).removeCollider(c, false);
              (bridge as any).disabledCollidersToRemove.delete(h);
            }
          }
          if ((bridge as any).bodiesToRemove && (bridge as any).bodiesToRemove.size > 0) {
            for (const bh of Array.from((bridge as any).bodiesToRemove)) {
              const b = (bridge.world as any).getRigidBody(bh);
              if (b) (bridge.world as any).removeRigidBody(b);
              (bridge as any).bodiesToRemove.delete(bh);
            }
          }
        } catch (error) {
          console.error('error 2:', error);
        }
        // Early continue: no drains, no projectiles/car updates in safe frames
        continue;
      } else {
        // Eventful step with strict pre-sweep and diagnostic skip if it fails
        sweepBeforeEventfulStep(bridge as any);
        let stepped = false;
        try {
          (bridge.world as any).step(bridge.eventQueue);
          stepped = true;
        } catch (err) {
          console.error('error 3:', err);
          // If eventful step panics, reset queue and schedule a safe-step next frame
          try { const R = (bridge.world as any).RAPIER; if (R) bridge.eventQueue = new R.EventQueue(true); } catch {}
          (bridge as any)._justSplitFrames = Math.max((bridge as any)._justSplitFrames ?? 0, 1);
        }
        if (!stepped) {
          continue;
        }
      }
      pruneStaleHandles(bridge as any);
      // Drain only on eventful frames
      drainContactForces(bridge);

      // Post-step: now safe to update projectiles and car (reads body.translation())
      if (shouldSpawn) {
        bridge.projectiles.push(spawnProjectile(bridge.world, { kind: 'box', origin: { x: 0, y: 6, z: 0 }, scale: 1.0 }));
      }
      bridge.projectiles = updateProjectiles(bridge.world, bridge.projectiles, dt, bridge);
      // Keep car stationary during deterministic test to focus forces on gravity + projectiles
      // updateLoadVehicle(bridge.world, bridge.car, dt);

      applyForcesAndSolve(bridge);
      if (bridge.overstressed > 0) {
        processSolverFractures(bridge, bridge.world.RAPIER, 0.0);
        // rebuild activeContactColliders/colliderToNode from current colliders after splits
        try {
          bridge.activeContactColliders?.clear?.();
          bridge.colliderToNode?.clear?.();
          const world: any = bridge.world;
          for (const chunk of bridge.chunks) {
            const h = chunk?.colliderHandle;
            if (typeof h === 'number') {
              const c = world.getCollider(h);
              const p = c ? c.parent() : undefined;
              const b = p != null ? world.getRigidBody(p) : undefined;
              if (c && (c.isEnabled ? c.isEnabled() : true) && b) {
                bridge.activeContactColliders?.add?.(h);
                bridge.colliderToNode?.set?.(h, chunk.nodeIndex);
              }
            }
          }
        } catch (error) {
          console.error('error 4:', error);
        }
      }

      handleCount = bridge._handleSplitEventsCount ?? 0;
      if (handleCount >= 2) { break; }
    }

    expect(handleCount).toBeGreaterThanOrEqual(1);

    // Assert normalized split log against a stable expectation schema (shape-only)
    expect(Array.isArray(bridge._splitLog)).toBe(true);
    const flat = (bridge._splitLog ?? []).flat();
    expect(flat.length).toBeGreaterThan(0);
    flat.forEach((evt: any) => {
      expect(typeof evt.parentActorIndex).toBe('number');
      expect(Array.isArray(evt.children)).toBe(true);
      evt.children.forEach((c: any) => {
        expect(typeof c.actorIndex).toBe('number');
        expect(Array.isArray(c.nodes)).toBe(true);
        // nodes are sorted
        for (let k = 1; k < c.nodes.length; k++) {
          expect(c.nodes[k]).toBeGreaterThanOrEqual(c.nodes[k-1]);
        }
      });
    });

    // Optional: compare to a predefined snapshot shape by counts
    const summary = flat.map((evt: any) => ({ parent: evt.parentActorIndex, childCount: evt.children.length }));
    expect(summary.length).toBeGreaterThan(0);

    // Shape-specific assertion for first split: [0..3] then quads of 4 until 95, then 8 singletons [96..103]
    const actualFirst = (bridge._splitLog ?? [])[0];
    const evt0 = actualFirst?.[0];
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


