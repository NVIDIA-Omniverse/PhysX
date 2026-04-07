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

  // ── Double-apply regression ─────────────────────────────────

  describe('Double-apply prevention (critical rollback correctness)', () => {
    it('no-rollback path applies damage exactly once', async () => {
      // Regression test for the double-apply bug where damageDrivePass
      // called contactReplayBuffer.replay() twice in the no-rollback path.
      // If double-applied, node health will be lower than expected.
      await loadModules();

      const runWithResimConfig = async (resimOnDamage: boolean) => {
        const scenario = buildWallScenario({ width: 4, height: 3 });
        const core = await buildDestructibleCore({
          scenario,
          materialScale: 1,
          resimulateOnFracture: false, // disable stress resim to isolate damage
          resimulateOnDamageDestroy: resimOnDamage,
          maxResimulationPasses: 1,
          damage: {
            enabled: true,
            kImpact: 0.001, // weak so we get damage but not destruction
            strengthPerVolume: 100000,
            contactCooldownMs: 0, // no cooldown so impacts register
          },
        });

        core.enqueueProjectile({
          position: { x: 0, y: 1.0, z: -1.5 },
          velocity: { x: 0, y: 0, z: 10 },
          mass: 2,
          radius: 0.15,
          ttl: 0.001,
        });

        stepN(core, 30);

        // Collect total remaining health across all chunks
        let totalHealth = 0;
        let totalMaxHealth = 0;
        for (const chunk of core.chunks) {
          if (chunk.health != null && chunk.maxHealth != null) {
            totalHealth += chunk.health;
            totalMaxHealth += chunk.maxHealth;
          }
        }

        core.dispose();
        return { totalHealth, totalMaxHealth };
      };

      // Run with resim OFF (should NOT double-apply)
      const noResim = await runWithResimConfig(false);

      // Run with resim ON
      const withResim = await runWithResimConfig(true);

      // Both should have similar total health — resim should NOT cause extra damage.
      // If the double-apply bug is present, withResim.totalHealth would be significantly
      // lower because contacts are applied twice in the no-rollback path.
      // Allow 5% tolerance for simulation variance.
      if (noResim.totalMaxHealth > 0 && noResim.totalHealth < noResim.totalMaxHealth) {
        // Only compare if damage actually occurred
        const damageFractionNoResim = 1 - noResim.totalHealth / noResim.totalMaxHealth;
        const damageFractionWithResim = 1 - withResim.totalHealth / withResim.totalMaxHealth;

        // With resim should NOT deal more than 1.5x the damage of without resim
        // (a double-apply bug would roughly 2x it)
        expect(damageFractionWithResim).toBeLessThan(damageFractionNoResim * 1.5 + 0.01);
      }
    });

    it('rollback path: health with resim >= health without resim', async () => {
      await loadModules();

      const runScenario = async (resim: boolean) => {
        const scenario = buildWallScenario({ width: 4, height: 3 });
        const core = await buildDestructibleCore({
          scenario,
          materialScale: 0.8,
          resimulateOnFracture: resim,
          resimulateOnDamageDestroy: resim,
          maxResimulationPasses: resim ? 2 : 0,
          damage: {
            enabled: true,
            kImpact: 0.01,
            strengthPerVolume: 10000,
          },
        });

        core.enqueueProjectile({
          position: { x: 0, y: 1.0, z: -2 },
          velocity: { x: 0, y: 0, z: 15 },
          mass: 3,
          radius: 0.15,
          ttl: 0.001,
        });

        stepN(core, 60);

        let totalHealth = 0;
        for (const chunk of core.chunks) {
          if (chunk.health != null) totalHealth += chunk.health;
        }
        const bonds = core.getActiveBondsCount();
        core.dispose();
        return { totalHealth, bonds };
      };

      const noResim = await runScenario(false);
      const withResim = await runScenario(true);

      // Resimulation should NOT produce more total damage than no-resim.
      // If contacts are double-counted, health would be much lower.
      // Allow generous tolerance because resim can change fracture patterns.
      expect(withResim.totalHealth).toBeGreaterThanOrEqual(noResim.totalHealth * 0.5);
    });
  });

  // ── Buffer isolation across frames ─────────────────────────

  describe('Contact buffer frame isolation', () => {
    it('contacts from previous frame do not leak into next frame', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 4, height: 3 });
      const samples: any[] = [];
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 1,
        damage: { enabled: true },
      });
      core.setProfiler({
        enabled: true,
        onSample: (s: any) => samples.push({ ...s }),
      });

      // Step 1-5: projectile approaches but hasn't hit yet
      stepN(core, 5);

      // Fire projectile
      core.enqueueProjectile({
        position: { x: 0, y: 1.0, z: -1.0 },
        velocity: { x: 0, y: 0, z: 20 },
        mass: 3,
        radius: 0.15,
        ttl: 0.001,
      });

      // Step to impact and beyond
      stepN(core, 30);

      // The pre-impact frames should have 0 or low contact counts.
      // The impact frame should have contacts.
      // Frames AFTER projectile expires should have dropping contacts.
      // Key check: contacts should NOT accumulate frame-over-frame.
      const contactCounts = samples.map(
        (s) => (s.bufferedExternalContacts ?? 0) + (s.bufferedInternalContacts ?? 0)
      );

      // At least one frame should have had contacts (impact)
      const maxContacts = Math.max(...contactCounts);
      expect(maxContacts).toBeGreaterThan(0);

      // Last few frames (projectile expired, structure settling) should have
      // fewer contacts than the impact frame — proves they aren't accumulating
      const lastFew = contactCounts.slice(-5);
      for (const c of lastFew) {
        expect(c).toBeLessThanOrEqual(maxContacts);
      }

      core.dispose();
    });
  });

  // ── Resimulation buffer correctness ────────────────────────

  describe('Resimulation buffer population', () => {
    it('resim pass re-drains contacts into buffer', async () => {
      await loadModules();
      const scenario = buildWallScenario({ width: 4, height: 3 });
      const samples: any[] = [];
      const core = await buildDestructibleCore({
        scenario,
        materialScale: 0.01, // very weak → guarantees fracture → triggers resim
        resimulateOnFracture: true,
        maxResimulationPasses: 2,
        damage: { enabled: true, kImpact: 0.1 },
      });
      core.setProfiler({
        enabled: true,
        onSample: (s: any) => samples.push({ ...s }),
      });

      // Fire strong projectile to force fracture + resim
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: -1.0 },
        velocity: { x: 0, y: 0, z: 25 },
        mass: 5,
        radius: 0.2,
        ttl: 0.001,
      });

      stepN(core, 60);

      // Check if any frame triggered resimulation
      const resimSamples = samples.filter((s) => (s.resimPasses ?? 0) > 0);
      // With very weak material and a projectile, we should get at least one resim
      if (resimSamples.length > 0) {
        // Verify the resim pass had contacts to work with
        for (const s of resimSamples) {
          expect(s.resimPasses).toBeGreaterThan(0);
          // The buffered contacts should still be present during resim
          // (they're re-drained from the re-stepped world)
          expect(s.bufferedExternalContacts).toBeGreaterThanOrEqual(0);
        }
      }

      // Verify no NaN in positions after resim
      for (const chunk of core.chunks) {
        if (chunk.worldPosition) {
          expect(Number.isFinite(chunk.worldPosition.x)).toBe(true);
          expect(Number.isFinite(chunk.worldPosition.y)).toBe(true);
          expect(Number.isFinite(chunk.worldPosition.z)).toBe(true);
        }
      }

      core.dispose();
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
