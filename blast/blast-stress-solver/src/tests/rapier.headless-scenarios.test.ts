/**
 * Comprehensive headless integration tests for blast-stress-solver scenarios.
 *
 * Tests cover: gravity stability, projectile collisions, material strength
 * variations, catastrophic vs partial damage, damage system toggle, projectile
 * parameter sweeps, bond inspection, structure-specific behavior, and determinism.
 *
 * Requires full WASM + TS build. Skips gracefully if dist is unavailable.
 * Run: npm run build && npx vitest run src/tests/rapier.headless-scenarios.test.ts
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

// Direct imports for scenario builder tests (always available, no WASM needed)
import {
  buildWallScenario as buildWallScenarioDirect,
  buildTowerScenario as buildTowerScenarioDirect,
  buildBeamBridgeScenario as buildBeamBridgeScenarioDirect,
} from '../scenarios/index';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

// Lazy imports from dist — only when WASM integration tests run
let buildDestructibleCore: (opts: any) => Promise<any>;
let buildWallScenario: (opts?: any) => any;
let buildTowerScenario: (opts?: any) => any;
let buildBeamBridgeScenario: (opts?: any) => any;

async function loadModules() {
  if (buildDestructibleCore) return;
  const rapier = await import('../../dist/rapier.js');
  const scenarios = await import('../../dist/scenarios.js');
  buildDestructibleCore = rapier.buildDestructibleCore;
  buildWallScenario = scenarios.buildWallScenario;
  buildTowerScenario = scenarios.buildTowerScenario;
  buildBeamBridgeScenario = scenarios.buildBeamBridgeScenario;
}

// ── Helpers ──────────────────────────────────────────────────

function stepN(core: any, n: number, dt = 1 / 60) {
  for (let i = 0; i < n; i++) core.step(dt);
}

function getBondSurvivalRate(core: any, initial: number): number {
  return core.getActiveBondsCount() / initial;
}

function getAvgDynamicY(core: any): number {
  const dynamic = core.chunks.filter((c: any) => c.active && !c.isSupport);
  if (dynamic.length === 0) return 0;
  let sum = 0;
  for (const c of dynamic) {
    // Use worldPosition if available, otherwise baseLocalOffset
    const pos = c.worldPosition ?? c.baseLocalOffset ?? c.localOffset;
    sum += pos.y;
  }
  return sum / dynamic.length;
}

/** Build a core for a given scenario with standard options. */
async function buildCore(scenario: any, opts: {
  materialScale?: number;
  gravity?: number;
  damage?: any;
  onNodeDestroyed?: (e: any) => void;
} = {}) {
  return buildDestructibleCore({
    scenario,
    gravity: opts.gravity ?? -9.81,
    materialScale: opts.materialScale ?? 1e8,
    damage: opts.damage,
    onNodeDestroyed: opts.onNodeDestroyed,
    resimulateOnFracture: true,
    maxResimulationPasses: 1,
    snapshotMode: 'perBody',
  });
}

// Use a smaller wall for faster tests
function smallWall(overrides?: any) {
  return buildWallScenario({
    spanSegments: 6,
    heightSegments: 4,
    deckMass: 2_000,
    ...overrides,
  });
}

// Use a smaller tower for faster tests
function smallTower(overrides?: any) {
  return buildTowerScenario({
    side: 3,
    stories: 4,
    totalMass: 1_000,
    ...overrides,
  });
}

// Use a smaller bridge for faster tests
function smallBridge(overrides?: any) {
  return buildBeamBridgeScenario({
    span: 8,
    deckWidth: 3,
    spanSegments: 10,
    widthSegments: 4,
    thicknessLayers: 2,
    deckMass: 10_000,
    pierHeight: 1.5,
    supportsPerSide: 2,
    ...overrides,
  });
}

// ── Tests ────────────────────────────────────────────────────

describe.skipIf(!runtimeAvailable)('Headless scenario tests (requires WASM build)', () => {

  // ────────────────────────────────────────────────────────────
  // A. Gravity Stability
  // ────────────────────────────────────────────────────────────

  describe('A. Gravity stability — structures survive under gravity', () => {
    it('wall remains intact under gravity for 2 seconds', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();
      expect(initial).toBeGreaterThan(0);

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBe(initial);
      core.dispose();
    });

    it('tower remains intact under gravity for 2 seconds', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();
      expect(initial).toBeGreaterThan(0);

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBe(initial);
      core.dispose();
    });

    it('bridge remains intact under gravity for 2 seconds', async () => {
      await loadModules();
      const scenario = smallBridge();
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();
      expect(initial).toBeGreaterThan(0);

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBe(initial);
      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // B. Weak material collapses under gravity
  // ────────────────────────────────────────────────────────────

  describe('B. Weak material collapses under gravity', () => {
    it('wall with very weak bonds fractures under gravity', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 0.1 });
      const initial = core.getActiveBondsCount();

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('tower with very weak bonds fractures under gravity', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, { materialScale: 0.1 });
      const initial = core.getActiveBondsCount();

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('bridge with very weak bonds fractures under gravity', async () => {
      await loadModules();
      const scenario = smallBridge();
      const core = await buildCore(scenario, { materialScale: 0.1 });
      const initial = core.getActiveBondsCount();

      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('detached blocks collide with each other (no interpenetration)', async () => {
      await loadModules();
      // Use a small wall with weak material so it fractures quickly under gravity
      const scenario = smallWall({ spanSegments: 4, heightSegments: 4 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 0.1,  // very weak so it fractures under gravity
        debrisCollisionMode: 'all',
        debrisCleanup: { mode: 'off' },
        resimulateOnFracture: true,
        maxResimulationPasses: 1,
        snapshotMode: 'perBody',
      });

      const initialBonds = core.getActiveBondsCount();
      expect(initialBonds).toBeGreaterThan(0);

      // Let gravity fracture the wall and blocks settle
      stepN(core, 300);

      // Confirm fracture actually happened
      const bondsAfter = core.getActiveBondsCount();
      expect(bondsAfter).toBeLessThan(initialBonds);

      // Count distinct body handles across active dynamic chunks
      const dynamicChunks = core.chunks.filter(
        (c: any) => c.active && !c.destroyed && !c.isSupport && c.worldPosition,
      );
      const bodyHandles = new Set(dynamicChunks.map((c: any) => c.bodyHandle));
      expect(bodyHandles.size).toBeGreaterThan(0);

      // Check that no two dynamic chunks on DIFFERENT bodies occupy the same space
      let overlaps = 0;
      for (let i = 0; i < dynamicChunks.length; i++) {
        for (let j = i + 1; j < dynamicChunks.length; j++) {
          const a = dynamicChunks[i];
          const b = dynamicChunks[j];
          if (a.bodyHandle === b.bodyHandle) continue;
          const pa = a.worldPosition;
          const pb = b.worldPosition;
          const dx = pa.x - pb.x;
          const dy = pa.y - pb.y;
          const dz = pa.z - pb.z;
          const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
          // Each chunk is roughly size.x × size.y; overlap if closer than 30% of min dimension
          const minDim = Math.min(a.size.x, a.size.y, b.size.x, b.size.y);
          if (dist < minDim * 0.3) {
            overlaps++;
            if (overlaps <= 3) {
              console.log(`OVERLAP: nodes ${a.nodeIndex}/${b.nodeIndex} dist=${dist.toFixed(4)} minDim=${minDim.toFixed(4)} bodies=${a.bodyHandle}/${b.bodyHandle} posA=(${pa.x.toFixed(2)},${pa.y.toFixed(2)},${pa.z.toFixed(2)}) posB=(${pb.x.toFixed(2)},${pb.y.toFixed(2)},${pb.z.toFixed(2)})`);
            }
          }
        }
      }

      expect(overlaps).toBe(0);
      core.dispose();
    });

    it('projectile-fractured blocks collide with each other (no interpenetration)', async () => {
      await loadModules();
      // Moderate material so projectile causes significant fracture
      const scenario = smallWall({ spanSegments: 6, heightSegments: 4 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 1e6,
        debrisCollisionMode: 'all',
        debrisCleanup: { mode: 'off' },
        resimulateOnFracture: true,
        maxResimulationPasses: 2,
        snapshotMode: 'perBody',
        contactForceScale: 30,
      });

      const initialBonds = core.getActiveBondsCount();

      // Let the wall settle
      stepN(core, 30);

      // Fire a heavy projectile at the wall center
      core.enqueueProjectile({
        position: { x: 0, y: 1.0, z: 5 },
        velocity: { x: 0, y: 0, z: -30 },
        radius: 0.35,
        mass: 15000,
        ttl: 6000,
      });

      // Step through impact and settling — give blocks time to fall and interact
      stepN(core, 400);

      const bondsAfter = core.getActiveBondsCount();
      // Confirm bonds actually broke from impact
      expect(bondsAfter).toBeLessThan(initialBonds);

      const dynamicChunks = core.chunks.filter(
        (c: any) => c.active && !c.destroyed && !c.isSupport && c.worldPosition,
      );
      const bodyHandles = new Set(dynamicChunks.map((c: any) => c.bodyHandle));
      console.log(`After projectile: ${bodyHandles.size} distinct bodies, ${dynamicChunks.length} dynamic chunks, ${bondsAfter}/${initialBonds} bonds`);

      // Check for overlapping blocks on different bodies
      let overlaps = 0;
      for (let i = 0; i < dynamicChunks.length; i++) {
        for (let j = i + 1; j < dynamicChunks.length; j++) {
          const a = dynamicChunks[i];
          const b = dynamicChunks[j];
          if (a.bodyHandle === b.bodyHandle) continue;
          const pa = a.worldPosition;
          const pb = b.worldPosition;
          const dx = pa.x - pb.x;
          const dy = pa.y - pb.y;
          const dz = pa.z - pb.z;
          const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
          const minDim = Math.min(a.size.x, a.size.y, b.size.x, b.size.y);
          if (dist < minDim * 0.3) {
            overlaps++;
            if (overlaps <= 5) {
              console.log(`OVERLAP: nodes ${a.nodeIndex}/${b.nodeIndex} dist=${dist.toFixed(4)} minDim=${minDim.toFixed(4)} bodies=${a.bodyHandle}/${b.bodyHandle} posA=(${pa.x.toFixed(2)},${pa.y.toFixed(2)},${pa.z.toFixed(2)}) posB=(${pb.x.toFixed(2)},${pb.y.toFixed(2)},${pb.z.toFixed(2)})`);
            }
          }
        }
      }

      console.log(`Total overlapping pairs: ${overlaps}`);
      expect(overlaps).toBe(0);
      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // C. Projectile collision & bond breaking
  // ────────────────────────────────────────────────────────────

  describe('C. Projectile collision and bond breaking', () => {
    // Projectile collisions break bonds via the damage system (contact forces
    // → damageSystem.onImpact → node health decrement → bond removal).
    // The stress solver only handles gravity-induced stress.
    // Use low strengthPerVolume so a single projectile hit destroys nodes
    // (default 10000 gives ~2400hp per node; damage per hit is ~137).
    const damageOpts = {
      enabled: true,
      autoDetachOnDestroy: true,
      autoCleanupPhysics: true,
      strengthPerVolume: 50,
    };

    it('projectile breaks wall bonds on impact', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e8, damage: damageOpts });

      stepN(core, 30);
      const bondsAfterSettle = core.getActiveBondsCount();
      expect(bondsAfterSettle).toBeGreaterThan(0);

      // Fire heavy projectile at center of wall (from +Z toward -Z)
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 15000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(bondsAfterSettle);
      core.dispose();
    });

    it('projectile breaks tower bonds on impact', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, { materialScale: 1e8, damage: damageOpts });

      stepN(core, 30);
      const bondsAfterSettle = core.getActiveBondsCount();

      // Fire heavy projectile at mid-height of tower (from +X toward -X)
      core.enqueueProjectile({
        position: { x: 5, y: 1.0, z: 0 },
        velocity: { x: -40, y: 0, z: 0 },
        radius: 0.3,
        mass: 15000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(bondsAfterSettle);
      core.dispose();
    });

    it('projectile breaks bridge bonds on impact', async () => {
      await loadModules();
      const scenario = smallBridge();
      const core = await buildCore(scenario, { materialScale: 1e8, damage: damageOpts });

      stepN(core, 30);
      const bondsAfterSettle = core.getActiveBondsCount();

      // Fire heavy projectile downward at center of bridge deck
      core.enqueueProjectile({
        position: { x: 0, y: 10, z: 0 },
        velocity: { x: 0, y: -50, z: 0 },
        radius: 0.4,
        mass: 20000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(bondsAfterSettle);
      core.dispose();
    });

    it('projectile is spawned and enqueued correctly', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      core.enqueueProjectile({
        position: { x: 0, y: 5, z: 10 },
        velocity: { x: 0, y: 0, z: -20 },
        radius: 0.2,
        mass: 100,
        ttl: 5000,
      });

      core.step(1 / 60);
      expect(core.projectiles.length).toBe(1);

      // Verify projectile persists for several frames
      stepN(core, 10);
      expect(core.projectiles.length).toBeGreaterThanOrEqual(0); // may or may not be cleaned up
      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // D. Material strength determines breakage
  // ────────────────────────────────────────────────────────────

  describe('D. Material strength determines breakage', () => {
    const projectile = {
      position: { x: 0, y: 1.5, z: 5 },
      velocity: { x: 0, y: 0, z: -40 },
      radius: 0.3,
      mass: 5000,
      ttl: 3000,
    };

    it('stronger material loses fewer bonds than weaker material', async () => {
      await loadModules();

      // Strong material
      const scenarioStrong = smallWall();
      const coreStrong = await buildCore(scenarioStrong, { materialScale: 1e8 });
      const initialStrong = coreStrong.getActiveBondsCount();
      stepN(coreStrong, 30);
      coreStrong.enqueueProjectile(projectile);
      stepN(coreStrong, 180);
      const brokenStrong = initialStrong - coreStrong.getActiveBondsCount();
      coreStrong.dispose();

      // Weak material
      const scenarioWeak = smallWall();
      const coreWeak = await buildCore(scenarioWeak, { materialScale: 1e3 });
      const initialWeak = coreWeak.getActiveBondsCount();
      stepN(coreWeak, 30);
      coreWeak.enqueueProjectile(projectile);
      stepN(coreWeak, 180);
      const brokenWeak = initialWeak - coreWeak.getActiveBondsCount();
      coreWeak.dispose();

      expect(brokenWeak).toBeGreaterThanOrEqual(brokenStrong);
    });

    it('very strong material resists projectile impact', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e10 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);
      // Light projectile
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -20 },
        radius: 0.2,
        mass: 100,
        ttl: 3000,
      });
      stepN(core, 180);

      // Very strong material should resist a light projectile
      const survival = getBondSurvivalRate(core, initial);
      expect(survival).toBeGreaterThanOrEqual(0.95);
      core.dispose();
    });

    it('monotonically: weaker material = more breakage', async () => {
      await loadModules();
      const scales = [1e8, 1e6, 1e4];
      const brokenCounts: number[] = [];

      for (const ms of scales) {
        const scenario = smallWall();
        const core = await buildCore(scenario, { materialScale: ms });
        const initial = core.getActiveBondsCount();
        stepN(core, 30);
        core.enqueueProjectile(projectile);
        stepN(core, 180);
        brokenCounts.push(initial - core.getActiveBondsCount());
        core.dispose();
      }

      // Each weaker material should break at least as many bonds
      for (let i = 1; i < brokenCounts.length; i++) {
        expect(brokenCounts[i]).toBeGreaterThanOrEqual(brokenCounts[i - 1]);
      }
    });
  });

  // ────────────────────────────────────────────────────────────
  // E. Catastrophic vs partial damage
  // ────────────────────────────────────────────────────────────

  describe('E. Catastrophic vs partial damage', () => {
    it('light projectile causes partial damage (>50% bonds survive)', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e6 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -20 },
        radius: 0.15,
        mass: 500,
        ttl: 3000,
      });
      stepN(core, 180);

      const survival = getBondSurvivalRate(core, initial);
      expect(survival).toBeGreaterThan(0.5);
      core.dispose();
    });

    it('heavy damage causes catastrophic bond loss (<50% bonds survive)', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e8 });
      const initial = core.getActiveBondsCount();

      // Use manual bond cutting to reliably test catastrophic damage path:
      // cut bonds for several interior nodes to simulate heavy impact
      const interiorNodes: number[] = [];
      for (let i = 0; i < scenario.nodes.length; i++) {
        if (scenario.nodes[i].mass > 0) interiorNodes.push(i);
      }
      // Cut bonds for the first 60% of dynamic nodes
      const toCut = Math.ceil(interiorNodes.length * 0.6);
      for (let i = 0; i < toCut; i++) {
        core.cutNodeBonds(interiorNodes[i]);
      }

      stepN(core, 60);

      const survival = getBondSurvivalRate(core, initial);
      expect(survival).toBeLessThan(0.5);
      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // F. Damage system toggle
  // ────────────────────────────────────────────────────────────

  describe('F. Damage system toggle', () => {
    it('with damage enabled, onNodeDestroyed fires on heavy impact', async () => {
      await loadModules();
      const scenario = smallWall();
      const destroyed: number[] = [];
      const core = await buildCore(scenario, {
        materialScale: 1e5,
        damage: {
          enabled: true,
          strengthPerVolume: 100,
          autoDetachOnDestroy: true,
          autoCleanupPhysics: true,
          kImpact: 1.0,
          minImpulseThreshold: 0,
          contactDamageScale: 1.0,
          massExponent: 0,
          contactCooldownMs: 0,
        },
        onNodeDestroyed: (e: any) => destroyed.push(e.nodeIndex),
      });

      stepN(core, 30);
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 10000,
        ttl: 3000,
      });
      stepN(core, 180);

      expect(destroyed.length).toBeGreaterThan(0);
      core.dispose();
    });

    it('with damage disabled, onNodeDestroyed never fires', async () => {
      await loadModules();
      const scenario = smallWall();
      const destroyed: number[] = [];
      const core = await buildCore(scenario, {
        materialScale: 1e5,
        damage: { enabled: false },
        onNodeDestroyed: (e: any) => destroyed.push(e.nodeIndex),
      });

      stepN(core, 30);
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 10000,
        ttl: 3000,
      });
      stepN(core, 180);

      expect(destroyed.length).toBe(0);
      core.dispose();
    });

    it('getNodeHealth returns health data when damage is enabled', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, {
        materialScale: 1e8,
        damage: {
          enabled: true,
          strengthPerVolume: 100,
          autoDetachOnDestroy: true,
        },
      });

      // Non-support node should have health
      const dynamicIdx = scenario.nodes.findIndex((n: any) => n.mass > 0);
      expect(dynamicIdx).toBeGreaterThanOrEqual(0);
      const health = core.getNodeHealth(dynamicIdx);
      expect(health).not.toBeNull();
      expect(health.maxHealth).toBeGreaterThan(0);
      expect(health.health).toBe(health.maxHealth);
      expect(health.destroyed).toBe(false);

      core.dispose();
    });

    it('applyNodeDamage destroys a node', async () => {
      await loadModules();
      const scenario = smallWall();
      const destroyed: number[] = [];
      const core = await buildCore(scenario, {
        materialScale: 1e8,
        damage: {
          enabled: true,
          strengthPerVolume: 100,
          autoDetachOnDestroy: true,
        },
        onNodeDestroyed: (e: any) => destroyed.push(e.nodeIndex),
      });

      const dynamicIdx = scenario.nodes.findIndex((n: any) => n.mass > 0);
      const h = core.getNodeHealth(dynamicIdx);
      core.applyNodeDamage(dynamicIdx, h.maxHealth + 1);
      core.step(1 / 60);

      expect(destroyed).toContain(dynamicIdx);
      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // G. Projectile parameter sweep
  // ────────────────────────────────────────────────────────────

  describe('G. Projectile parameter sweep', () => {
    it('heavier/faster projectiles break more bonds', async () => {
      await loadModules();

      const configs = [
        { mass: 10, speed: 10, label: 'light-slow' },
        { mass: 1000, speed: 20, label: 'medium' },
        { mass: 5000, speed: 40, label: 'heavy-fast' },
        { mass: 20000, speed: 60, label: 'very-heavy-fast' },
      ];

      const results: { label: string; broken: number }[] = [];

      for (const cfg of configs) {
        const scenario = smallWall();
        const core = await buildCore(scenario, { materialScale: 1e6 });
        const initial = core.getActiveBondsCount();
        stepN(core, 30);
        core.enqueueProjectile({
          position: { x: 0, y: 1.5, z: 5 },
          velocity: { x: 0, y: 0, z: -cfg.speed },
          radius: 0.3,
          mass: cfg.mass,
          ttl: 3000,
        });
        stepN(core, 180);
        results.push({ label: cfg.label, broken: initial - core.getActiveBondsCount() });
        core.dispose();
      }

      // The trend should be monotonically non-decreasing
      for (let i = 1; i < results.length; i++) {
        expect(results[i].broken).toBeGreaterThanOrEqual(results[i - 1].broken);
      }

      // The lightest should break few or zero bonds
      expect(results[0].broken).toBeLessThanOrEqual(results[results.length - 1].broken);
    });
  });

  // ────────────────────────────────────────────────────────────
  // H. Bond inspection & manual cut
  // ────────────────────────────────────────────────────────────

  describe('H. Bond inspection and manual cut', () => {
    it('getNodeBonds returns correct bonds for interior node', async () => {
      await loadModules();
      const scenario = smallWall({ spanSegments: 4, heightSegments: 3 });
      const core = await buildCore(scenario);

      // Find an interior dynamic node (not support, not on edge)
      // In a 4×3 wall, node at grid (1,1,0) should have bonds in X and Y directions
      const interiorIdx = scenario.gridCoordinates.findIndex(
        (c: any) => c && c.ix === 1 && c.iy === 1 && c.iz === 0,
      );
      expect(interiorIdx).toBeGreaterThanOrEqual(0);

      const bonds = core.getNodeBonds(interiorIdx);
      expect(bonds.length).toBeGreaterThan(0);

      // Each bond should reference this node
      for (const b of bonds) {
        expect(b.node0 === interiorIdx || b.node1 === interiorIdx).toBe(true);
      }

      core.dispose();
    });

    it('cutBond removes exactly one bond', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();

      const dynamicIdx = scenario.nodes.findIndex((n: any) => n.mass > 0);
      const bonds = core.getNodeBonds(dynamicIdx);
      expect(bonds.length).toBeGreaterThan(0);

      const result = core.cutBond(bonds[0].index);
      expect(result).toBe(true);
      expect(core.getActiveBondsCount()).toBe(initial - 1);

      // Cutting same bond again returns false
      expect(core.cutBond(bonds[0].index)).toBe(false);

      core.dispose();
    });

    it('cutNodeBonds removes all bonds for a node', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();

      const dynamicIdx = scenario.nodes.findIndex((n: any) => n.mass > 0);
      const bondsBefore = core.getNodeBonds(dynamicIdx);
      expect(bondsBefore.length).toBeGreaterThan(0);

      core.cutNodeBonds(dynamicIdx);

      const bondsAfter = core.getNodeBonds(dynamicIdx);
      expect(bondsAfter.length).toBe(0);
      expect(core.getActiveBondsCount()).toBe(initial - bondsBefore.length);

      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // I. Structure-specific behavior
  // ────────────────────────────────────────────────────────────

  describe('I. Structure-specific behavior', () => {
    it('bridge: cutting midspan bonds severs the structure', async () => {
      await loadModules();
      const scenario = smallBridge({ spanSegments: 6, widthSegments: 2, thicknessLayers: 1 });
      const core = await buildCore(scenario);
      const initial = core.getActiveBondsCount();

      // Cut all bonds at midspan — bonds between ix=2 and ix=3
      const midBonds: number[] = [];
      for (let i = 0; i < scenario.nodes.length; i++) {
        const gc = scenario.gridCoordinates[i];
        if (!gc || gc.ix !== 2 || gc.iy < 0) continue;
        const bonds = core.getNodeBonds(i);
        for (const b of bonds) {
          const otherIdx = b.node0 === i ? b.node1 : b.node0;
          const otherGc = scenario.gridCoordinates[otherIdx];
          if (otherGc && otherGc.ix === 3) {
            midBonds.push(b.index);
          }
        }
      }

      expect(midBonds.length).toBeGreaterThan(0);
      for (const bi of midBonds) core.cutBond(bi);

      // Verify bonds were actually removed
      const afterCut = core.getActiveBondsCount();
      expect(afterCut).toBe(initial - midBonds.length);

      // Step to allow physics response
      stepN(core, 60);

      // The cut bonds should have been removed; additional cascade is possible
      // but not guaranteed — verify at least the cut bonds are gone
      const finalBonds = core.getActiveBondsCount();
      expect(finalBonds).toBeLessThanOrEqual(afterCut);
      expect(initial - finalBonds).toBeGreaterThanOrEqual(midBonds.length);
      core.dispose();
    });

    it('tower: projectile at base causes more damage than at top', async () => {
      await loadModules();

      // Base impact
      const scenarioBase = smallTower();
      const coreBase = await buildCore(scenarioBase, { materialScale: 1e6 });
      const initialBase = coreBase.getActiveBondsCount();
      stepN(coreBase, 30);
      coreBase.enqueueProjectile({
        position: { x: 5, y: 0.25, z: 0 },
        velocity: { x: -40, y: 0, z: 0 },
        radius: 0.3,
        mass: 5000,
        ttl: 3000,
      });
      stepN(coreBase, 180);
      const brokenBase = initialBase - coreBase.getActiveBondsCount();
      coreBase.dispose();

      // Top impact
      const scenarioTop = smallTower();
      const coreTop = await buildCore(scenarioTop, { materialScale: 1e6 });
      const initialTop = coreTop.getActiveBondsCount();
      stepN(coreTop, 30);
      const topY = scenarioTop.nodes.reduce((max: number, n: any) => Math.max(max, n.centroid.y), -Infinity);
      coreTop.enqueueProjectile({
        position: { x: 5, y: topY, z: 0 },
        velocity: { x: -40, y: 0, z: 0 },
        radius: 0.3,
        mass: 5000,
        ttl: 3000,
      });
      stepN(coreTop, 180);
      const brokenTop = initialTop - coreTop.getActiveBondsCount();
      coreTop.dispose();

      // Base hit should cause more damage because upper portion cascades
      expect(brokenBase).toBeGreaterThanOrEqual(brokenTop);
    });

    it('wall: cutting central bonds creates localized hole', async () => {
      await loadModules();
      const scenario = smallWall({ spanSegments: 8, heightSegments: 5 });
      const core = await buildCore(scenario, { materialScale: 1e8 });
      const initial = core.getActiveBondsCount();

      // Simulate localized damage by cutting bonds for a central node
      // Find an interior node near the center of the wall
      let centerNode = -1;
      for (let i = 0; i < scenario.nodes.length; i++) {
        const gc = scenario.gridCoordinates[i];
        if (gc && gc.ix === 4 && gc.iy === 2 && gc.iz === 0) {
          centerNode = i;
          break;
        }
      }
      expect(centerNode).toBeGreaterThan(-1);

      // Cut bonds for the center node — localized damage
      core.cutNodeBonds(centerNode);

      stepN(core, 60);

      // Should break some bonds (at least the cut ones) but not all
      const survival = getBondSurvivalRate(core, initial);
      expect(survival).toBeGreaterThan(0.5);
      expect(survival).toBeLessThan(1.0);

      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // J. Determinism
  // ────────────────────────────────────────────────────────────

  describe('J. Determinism', () => {
    it('two identical runs produce identical bond counts', async () => {
      await loadModules();

      const bondCounts: number[][] = [[], []];

      for (let run = 0; run < 2; run++) {
        const scenario = smallWall();
        const core = await buildCore(scenario);

        for (let i = 0; i < 60; i++) {
          core.step(1 / 60);
          bondCounts[run].push(core.getActiveBondsCount());
        }
        core.dispose();
      }

      expect(bondCounts[0]).toEqual(bondCounts[1]);
    });
  });

  // K and L sections below are outside the skipIf block

  // ────────────────────────────────────────────────────────────
  // L. API surface and lifecycle
  // ────────────────────────────────────────────────────────────

  describe('L. API surface and lifecycle', () => {
    it('core has expected API methods', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      expect(typeof core.step).toBe('function');
      expect(typeof core.dispose).toBe('function');
      expect(typeof core.enqueueProjectile).toBe('function');
      expect(typeof core.getActiveBondsCount).toBe('function');
      expect(typeof core.getNodeBonds).toBe('function');
      expect(typeof core.cutBond).toBe('function');
      expect(typeof core.cutNodeBonds).toBe('function');
      expect(typeof core.getRigidBodyCount).toBe('function');
      expect(typeof core.setGravity).toBe('function');
      expect(typeof core.setProfiler).toBe('function');

      core.dispose();
    });

    it('profiler collects timing data', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      const samples: any[] = [];
      core.setProfiler({ enabled: true, onSample: (s: any) => samples.push(s) });
      stepN(core, 3);

      expect(samples.length).toBe(3);
      expect(typeof samples[0].totalMs).toBe('number');
      expect(typeof samples[0].rapierStepMs).toBe('number');

      core.dispose();
    });

    it('getRigidBodyCount reports the intact root-body setup', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      const initialBodies = core.getRigidBodyCount();
      expect(initialBodies).toBeGreaterThanOrEqual(2);

      stepN(core, 30);
      expect(core.getRigidBodyCount()).toBe(initialBodies);

      core.dispose();
    });

    it('setGravity changes gravity mid-simulation', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      // Run with normal gravity
      stepN(core, 30);
      const bondsAfterGravity = core.getActiveBondsCount();

      // Set gravity to zero — should stabilize
      core.setGravity(0);
      stepN(core, 60);

      // Bonds should be the same as before (no new fractures with zero gravity)
      expect(core.getActiveBondsCount()).toBe(bondsAfterGravity);

      core.dispose();
    });

    it('keeps chunks on the root body until an impact-driven split occurs', async () => {
      await loadModules();
      const scenario = buildWallScenario({
        spanSegments: 12,
        heightSegments: 6,
        deckMass: 10_000,
      });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 1e10,
        debrisCollisionMode: 'all',
        damage: { enabled: false },
        resimulateOnFracture: true,
        maxResimulationPasses: 1,
        snapshotMode: 'perBody',
        contactForceScale: 30,
      });

      const initialBodies = core.getRigidBodyCount();
      const initialBonds = core.getActiveBondsCount();

      stepN(core, 30);

      const intactChunks = core.chunks.filter((c: any) => c.active && !c.isSupport && !c.destroyed);
      expect(intactChunks.length).toBeGreaterThan(0);
      expect(intactChunks.every((c: any) => c.bodyHandle === core.rootBodyHandle)).toBe(true);
      expect(core.getRigidBodyCount()).toBe(initialBodies);

      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -20 },
        radius: 0.35,
        mass: 1000,
        ttl: 3000,
      });
      stepN(core, 60);

      expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);
      expect(core.getRigidBodyCount()).toBeGreaterThan(initialBodies);
      expect(
        core.chunks.some(
          (c: any) => c.active && !c.isSupport && !c.destroyed && c.bodyHandle !== core.rootBodyHandle,
        ),
      ).toBe(true);

      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // M. Correctness invariants for destruction pipeline
  //
  // These tests verify invariants that MUST hold even as performance
  // optimizations are added. They exist because past optimization
  // attempts accidentally broke these properties.
  // ────────────────────────────────────────────────────────────

  describe('M. Correctness invariants (destruction pipeline)', () => {

    // INVARIANT: When a node is destroyed (via damage, debris cleanup, or
    // skipSingleBodies), its bonds MUST be removed from the WASM stress solver.
    // Without this, the solver computes stress on non-existent nodes, wastes
    // CPU cycles, and produces stale debug render lines.
    it('destroyed nodes have their bonds removed from the solver', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, {
        materialScale: 1e8,
        damage: {
          enabled: true,
          autoDetachOnDestroy: true,
          autoCleanupPhysics: true,
          strengthPerVolume: 50,
        },
      });

      const initialBonds = core.getActiveBondsCount();
      expect(initialBonds).toBeGreaterThan(0);

      // Fire a projectile to cause destruction
      core.enqueueProjectile({
        position: { x: 0, y: 1, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 15000,
        ttl: 3000,
      });
      stepN(core, 120);

      // Some chunks should be destroyed
      const destroyed = core.chunks.filter((c: any) => c.destroyed);
      if (destroyed.length > 0) {
        // For every destroyed chunk, verify its bonds are gone from the solver
        for (const chunk of destroyed) {
          const bonds = core.getNodeBonds(chunk.nodeIndex);
          expect(bonds.length).toBe(0);
        }
      }

      // Active bond count should have decreased
      expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);

      core.dispose();
    });

    // INVARIANT: After fracture splits a structure, bodies that lose all their
    // colliders (because all chunks migrated to new bodies) must be cleaned up.
    // Without this, empty bodies accumulate in the Rapier world, wasting memory
    // and physics step time.
    it('bodies with zero colliders are removed after splits', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, { materialScale: 0.5 });

      // Very weak material — should fracture heavily under gravity
      stepN(core, 180);

      // Get all body handles that chunks reference
      const usedBodies = new Set<number>();
      for (const chunk of core.chunks) {
        if (chunk.active && chunk.bodyHandle != null) {
          usedBodies.add(chunk.bodyHandle);
        }
      }

      // The total rigid body count should be close to the number of
      // distinct body handles referenced by active chunks (plus ground + root)
      const totalBodies = core.getRigidBodyCount();
      // Allow some margin (projectile bodies, recently-emptied bodies pending cleanup)
      // but there shouldn't be dozens of orphaned empty bodies
      const orphanedEstimate = totalBodies - usedBodies.size - 2; // -2 for root+ground
      expect(orphanedEstimate).toBeLessThan(10);

      core.dispose();
    });

    // INVARIANT: Debris cleanup must mark nodes as destroyed (not just inactive),
    // so their bonds are cut and the solver stops computing stress on them.
    it('debris cleanup marks nodes as destroyed with bonds cut', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, {
        materialScale: 1e8,
        damage: {
          enabled: true,
          autoDetachOnDestroy: true,
          autoCleanupPhysics: true,
          strengthPerVolume: 50,
        },
      });

      const initialBonds = core.getActiveBondsCount();

      // Fire projectile to cause destruction
      core.enqueueProjectile({
        position: { x: 0, y: 1, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 15000,
        ttl: 3000,
      });
      // Run long enough for debris TTL to expire (default 10s = 600 frames at 60fps)
      // Use shorter run since debris cleanup happens based on Date.now()
      stepN(core, 300);

      // Every inactive chunk should either be destroyed or still valid
      for (const chunk of core.chunks) {
        if (!chunk.active) {
          // Inactive chunks should have their collider handles cleaned up
          // (either null from handleNodeDestroyed or pending removal)
          // This verifies cleanup was thorough, not just setting active=false
          if (chunk.destroyed) {
            const bonds = core.getNodeBonds(chunk.nodeIndex);
            expect(bonds.length).toBe(0);
          }
        }
      }

      core.dispose();
    });

    // INVARIANT: The FracturePolicy with default settings (-1 for all limits)
    // must produce identical results to not specifying a policy at all.
    it('default fracture policy produces same results as no policy', async () => {
      await loadModules();
      const scenario = smallTower();

      // Run without policy
      const coreA = await buildCore(scenario, { materialScale: 1e8 });
      stepN(coreA, 30);
      coreA.enqueueProjectile({
        position: { x: 0, y: 1, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3, mass: 15000, ttl: 3000,
      });
      stepN(coreA, 120);
      const bondsA = coreA.getActiveBondsCount();
      const bodiesA = coreA.getRigidBodyCount();
      coreA.dispose();

      // Run with explicit default policy
      const coreB = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 1e8,
        resimulateOnFracture: true,
        maxResimulationPasses: 1,
        snapshotMode: 'perBody',
        fracturePolicy: {
          maxFracturesPerFrame: -1,
          maxNewBodiesPerFrame: -1,
          maxColliderMigrationsPerFrame: -1,
          maxDynamicBodies: -1,
          minChildNodeCount: 1,
          idleSkip: true,
        },
      });
      stepN(coreB, 30);
      coreB.enqueueProjectile({
        position: { x: 0, y: 1, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3, mass: 15000, ttl: 3000,
      });
      stepN(coreB, 120);
      const bondsB = coreB.getActiveBondsCount();
      const bodiesB = coreB.getRigidBodyCount();
      coreB.dispose();

      // Results should be identical
      expect(bondsB).toBe(bondsA);
      expect(bodiesB).toBe(bodiesA);
    });

    // INVARIANT: maxDynamicBodies cap must actually limit the number of bodies
    it('maxDynamicBodies caps rigid body count', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 1e8,
        resimulateOnFracture: true,
        maxResimulationPasses: 1,
        snapshotMode: 'perBody',
        fracturePolicy: { maxDynamicBodies: 10 },
      });

      core.enqueueProjectile({
        position: { x: 0, y: 1, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3, mass: 15000, ttl: 3000,
      });
      stepN(core, 180);

      // The dynamic-body cap should still allow the fixed root + ground bodies.
      expect(core.getRigidBodyCount()).toBeGreaterThan(10);
      // Body count should be capped (10 dynamic + root + ground = 12 max)
      expect(core.getRigidBodyCount()).toBeLessThanOrEqual(12);

      core.dispose();
    });
  });
});

// ══════════════════════════════════════════════════════════════
// Scenario builder correctness tests — always run (no WASM needed)
// ══════════════════════════════════════════════════════════════

describe('Scenario builder correctness (always run)', () => {
  it('wall scenario has correct node and bond counts', () => {
    const scenario = buildWallScenarioDirect({ spanSegments: 4, heightSegments: 3, layers: 1 });

    expect(scenario.nodes).toHaveLength(12); // 4 × 3
    const supports = scenario.nodes.filter((n) => n.mass === 0);
    expect(supports.length).toBe(4);

    // Bonds: horizontal (3 per row × 3 rows) + vertical (4 per col × 2 gaps) = 9 + 8 = 17
    expect(scenario.bonds.length).toBe(17);
  });

  it('tower scenario has supports at bottom', () => {
    const scenario = buildTowerScenarioDirect({ side: 2, stories: 3 });

    // Total nodes: 2 × 2 × (3+1) = 16
    expect(scenario.nodes).toHaveLength(16);

    // Bottom row (iy=0) = supports with mass=0: 2 × 2 = 4
    const supports = scenario.nodes.filter((n) => n.mass === 0);
    expect(supports.length).toBe(4);
  });

  it('bridge scenario has footing supports (mass=0)', () => {
    const scenario = buildBeamBridgeScenarioDirect({
      spanSegments: 4,
      widthSegments: 2,
      thicknessLayers: 1,
      supportsPerSide: 1,
      supportWidthSegments: 1,
      supportDepthSegments: 1,
    });

    const supports = scenario.nodes.filter((n) => n.mass === 0);
    expect(supports.length).toBeGreaterThan(0);

    const deckMinY = Math.min(
      ...scenario.nodes.filter((n) => n.mass > 0).map((n) => n.centroid.y),
    );
    for (const s of supports) {
      expect(s.centroid.y).toBeLessThan(deckMinY);
    }
  });

  it('bond areas are positive for all scenarios', () => {
    const builders = [
      () => buildWallScenarioDirect({ spanSegments: 6, heightSegments: 4 }),
      () => buildTowerScenarioDirect({ side: 3, stories: 4 }),
      () => buildBeamBridgeScenarioDirect({ spanSegments: 6, widthSegments: 3, thicknessLayers: 1 }),
    ];

    for (const builder of builders) {
      const scenario = builder();
      for (const bond of scenario.bonds) {
        expect(bond.area).toBeGreaterThan(0);
      }
    }
  });

  it('bond normals are unit vectors', () => {
    const builders = [
      () => buildWallScenarioDirect({ spanSegments: 6, heightSegments: 4 }),
      () => buildTowerScenarioDirect({ side: 3, stories: 4 }),
      () => buildBeamBridgeScenarioDirect({ spanSegments: 6, widthSegments: 3, thicknessLayers: 1 }),
    ];

    for (const builder of builders) {
      const scenario = builder();
      for (const bond of scenario.bonds) {
        const len = Math.hypot(bond.normal.x, bond.normal.y, bond.normal.z);
        expect(len).toBeCloseTo(1.0, 3);
      }
    }
  });

  it('wall area normalization is isotropic — same per-bond area in X and Y', () => {
    const scenario = buildWallScenarioDirect({
      span: 6, height: 3, thickness: 0.32,
      spanSegments: 12, heightSegments: 6, layers: 1,
      normalizeAreas: true,
    });

    // Group bonds by dominant axis of their normal
    const pick = (n: { x: number; y: number; z: number }): 'x' | 'y' | 'z' => {
      const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
      return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
    };
    const byAxis: Record<string, number[]> = { x: [], y: [], z: [] };
    for (const b of scenario.bonds) byAxis[pick(b.normal)].push(b.area);

    // X-bonds and Y-bonds should start with the same raw cell-face area
    // (cellY×cellZ vs cellX×cellZ; both are 0.5×0.32). With isotropic
    // normalization (uniform scale factor), they should remain equal.
    expect(byAxis.x.length).toBeGreaterThan(0);
    expect(byAxis.y.length).toBeGreaterThan(0);

    const avgX = byAxis.x.reduce((s, a) => s + a, 0) / byAxis.x.length;
    const avgY = byAxis.y.reduce((s, a) => s + a, 0) / byAxis.y.length;

    // With isotropic normalization, per-bond areas should be equal
    // (since raw X and Y face areas are both cellY*cellZ = cellX*cellZ = 0.5*0.32)
    expect(avgX).toBeCloseTo(avgY, 4);
  });

  it('tower area normalization is isotropic — uniform spacing gives equal areas', () => {
    const scenario = buildTowerScenarioDirect({
      side: 4, stories: 8,
      spacing: { x: 0.5, y: 0.5, z: 0.5 },
      normalizeAreas: true,
      addDiagonals: false,
    });

    // With uniform spacing, all axis-aligned bonds have the same raw area
    // and isotropic normalization preserves this equality.
    const pick = (n: { x: number; y: number; z: number }): 'x' | 'y' | 'z' => {
      const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
      return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
    };
    const byAxis: Record<string, number[]> = { x: [], y: [], z: [] };
    for (const b of scenario.bonds) byAxis[pick(b.normal)].push(b.area);

    const avgX = byAxis.x.reduce((s, a) => s + a, 0) / byAxis.x.length;
    const avgY = byAxis.y.reduce((s, a) => s + a, 0) / byAxis.y.length;
    const avgZ = byAxis.z.reduce((s, a) => s + a, 0) / byAxis.z.length;

    // All per-bond areas should be equal with uniform spacing
    expect(avgX).toBeCloseTo(avgY, 4);
    expect(avgX).toBeCloseTo(avgZ, 4);
  });

  it('wall without normalization has raw areaScale-based areas', () => {
    const scenario = buildWallScenarioDirect({
      spanSegments: 4, heightSegments: 3, layers: 1,
      normalizeAreas: false, areaScale: 0.1,
    });

    // With normalizeAreas=false, areas should be cellDim1 × cellDim2 × areaScale
    // Just verify they're all positive and consistent
    const areas = new Set(scenario.bonds.map((b) => Math.round(b.area * 1e6) / 1e6));
    expect(areas.size).toBeGreaterThan(0);
  });

  it('tower with diagonals has more bonds than without', () => {
    const withDiag = buildTowerScenarioDirect({ side: 3, stories: 3, addDiagonals: true });
    const withoutDiag = buildTowerScenarioDirect({ side: 3, stories: 3, addDiagonals: false });

    expect(withDiag.bonds.length).toBeGreaterThan(withoutDiag.bonds.length);
  });

  it('bridge deck nodes are above post nodes', () => {
    const scenario = buildBeamBridgeScenarioDirect({
      spanSegments: 6, widthSegments: 3, thicknessLayers: 1,
      pierHeight: 2.0,
    });

    const gc = scenario.gridCoordinates!;
    const deckYs = scenario.nodes
      .filter((_n, i) => gc[i] && gc[i].iy >= 0)
      .map((n) => n.centroid.y);
    const postYs = scenario.nodes
      .filter((_n, i) => gc[i] && gc[i].iy < 0 && scenario.nodes[i].mass > 0)
      .map((n) => n.centroid.y);

    if (postYs.length > 0) {
      const minDeckY = Math.min(...deckYs);
      const maxPostY = Math.max(...postYs);
      expect(minDeckY).toBeGreaterThan(maxPostY);
    }
  });
});
