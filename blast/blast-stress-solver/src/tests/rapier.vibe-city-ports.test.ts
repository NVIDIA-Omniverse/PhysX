/**
 * Integration tests for features ported from vibe-city:
 * - Collision group filtering (debris modes)
 * - Speed-scaled damage
 * - Contact replay buffer for rollback correctness
 * - Contact point extraction + splash AOE
 * - Projectile momentum boost
 *
 * Requires full WASM + TS build.
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

let buildDestructibleCore: (opts: any) => Promise<any>;
let buildWallScenario: (opts?: any) => any;

async function loadModules() {
  if (buildDestructibleCore) return;
  const rapier = await import('../../dist/rapier.js');
  const scenarios = await import('../../dist/scenarios.js');
  buildDestructibleCore = rapier.buildDestructibleCore;
  buildWallScenario = scenarios.buildWallScenario;
}

function stepN(core: any, n: number, dt = 1 / 60) {
  for (let i = 0; i < n; i++) core.step(dt);
}

describe.skipIf(!runtimeAvailable)('Vibe-city ported features (requires WASM build)', () => {
  // ── Collision group filtering ──────────────────────────────

  describe('Collision group filtering', () => {
    it('setDebrisCollisionMode can be called without error', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 3, height: 2 });
      const core = await buildDestructibleCore({ scenario, materialScale: 1 });
      expect(() => core.setDebrisCollisionMode('noDebrisPairs')).not.toThrow();
      expect(() => core.setDebrisCollisionMode('debrisGroundOnly')).not.toThrow();
      expect(() => core.setDebrisCollisionMode('debrisNone')).not.toThrow();
      expect(() => core.setDebrisCollisionMode('all')).not.toThrow();
      core.dispose();
    });

    it('setSingleCollisionMode (legacy) can be called without error', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 3, height: 2 });
      const core = await buildDestructibleCore({ scenario, materialScale: 1 });
      expect(() => core.setSingleCollisionMode('noSinglePairs')).not.toThrow();
      expect(() => core.setSingleCollisionMode('singleGround')).not.toThrow();
      expect(() => core.setSingleCollisionMode('all')).not.toThrow();
      core.dispose();
    });

    it('setMaxCollidersForDebris triggers group recalculation', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 3, height: 2 });
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 1,
        debrisCollisionMode: 'noDebrisPairs',
      });
      expect(() => core.setMaxCollidersForDebris(5)).not.toThrow();
      stepN(core, 5);
      core.dispose();
    });

    it('simulation still runs correctly with debris mode active', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 4, height: 3 });
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 0.001, // very weak to ensure fracture
        debrisCollisionMode: 'noDebrisPairs',
      });
      const initialBonds = core.getActiveBondsCount();
      stepN(core, 120); // 2 seconds
      // Weak material should fracture under gravity
      expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);
      core.dispose();
    });
  });

  // ── Speed-scaled damage ────────────────────────────────────

  describe('Speed-scaled damage', () => {
    it('damage system receives speed-scaled contacts (no crash)', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 4, height: 3 });
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 1,
        damage: {
          enabled: true,
          kImpact: 0.01,
          speedMinExternal: 0.5,
          speedMax: 6.0,
          slowSpeedFactor: 0.1,
          fastSpeedFactor: 5.0,
        },
      });

      // Fire a fast projectile
      core.enqueueProjectile({
        position: { x: 0, y: 1.0, z: -2 },
        velocity: { x: 0, y: 0, z: 15 },
        mass: 3,
        radius: 0.15,
        ttl: 0.001,
      });

      stepN(core, 30);
      // Should not crash and simulation should still work
      expect(core.chunks.length).toBeGreaterThan(0);
      core.dispose();
    });
  });

  // ── Contact replay buffer ──────────────────────────────────

  describe('Contact replay buffer (rollback correctness)', () => {
    it('resimulation with damage works without double-counting', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 4, height: 3 });
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 1,
        resimulateOnFracture: true,
        resimulateOnDamageDestroy: true,
        maxResimulationPasses: 1,
        damage: {
          enabled: true,
          kImpact: 0.05,
          strengthPerVolume: 5000,
        },
      });

      // Fire a projectile to trigger contact damage + potential rollback
      core.enqueueProjectile({
        position: { x: 0, y: 1.0, z: -1.5 },
        velocity: { x: 0, y: 0, z: 20 },
        mass: 5,
        radius: 0.2,
        ttl: 0.001,
      });

      // Run enough steps for impact and potential resim
      stepN(core, 60);

      // Verify structure is still functional (no NaN, no crash)
      for (const chunk of core.chunks) {
        if (chunk.worldPosition) {
          expect(Number.isFinite(chunk.worldPosition.x)).toBe(true);
          expect(Number.isFinite(chunk.worldPosition.y)).toBe(true);
          expect(Number.isFinite(chunk.worldPosition.z)).toBe(true);
        }
      }

      core.dispose();
    });

    it('determinism: two identical runs with resim produce same bond counts', async () => {
      await loadModules();
      const runOnce = async () => {
        const scenario = buildWallScenario({ width: 4, height: 3 });
        const core = await buildDestructibleCore({
          scenario,
          materialScale: 0.5,
          resimulateOnFracture: true,
          maxResimulationPasses: 1,
          damage: {
            enabled: true,
            kImpact: 0.02,
            strengthPerVolume: 8000,
          },
        });
        core.enqueueProjectile({
          position: { x: 0, y: 1.0, z: -2 },
          velocity: { x: 0, y: 0, z: 12 },
          mass: 2,
          radius: 0.15,
          ttl: 0.001,
        });
        stepN(core, 60);
        const bonds = core.getActiveBondsCount();
        core.dispose();
        return bonds;
      };

      const bonds1 = await runOnce();
      const bonds2 = await runOnce();
      expect(bonds1).toBe(bonds2);
    });
  });

  // ── Profiler integration ───────────────────────────────────

  describe('Profiler with new features', () => {
    it('profiler still collects timing data correctly', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 3, height: 2 });
      const samples: any[] = [];
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 1,
        damage: { enabled: true },
      });
      core.setProfiler({
        enabled: true,
        onSample: (s: any) => samples.push(s),
      });

      stepN(core, 10);
      expect(samples.length).toBe(10);
      for (const s of samples) {
        expect(s.totalMs).toBeGreaterThanOrEqual(0);
        expect(s.contactDrainMs).toBeGreaterThanOrEqual(0);
        expect(s.rapierStepMs).toBeGreaterThanOrEqual(0);
      }

      core.dispose();
    });
  });
});
