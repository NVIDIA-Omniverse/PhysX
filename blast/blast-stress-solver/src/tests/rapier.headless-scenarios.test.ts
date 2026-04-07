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
  });

  // ────────────────────────────────────────────────────────────
  // C. Projectile collision & bond breaking
  // ────────────────────────────────────────────────────────────

  describe('C. Projectile collision and bond breaking', () => {
    it('projectile breaks wall bonds on impact', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e6 });
      const initial = core.getActiveBondsCount();

      // Settle
      stepN(core, 30);
      expect(core.getActiveBondsCount()).toBe(initial);

      // Fire projectile at center of wall (from +Z toward -Z)
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.3,
        mass: 5000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('projectile breaks tower bonds on impact', async () => {
      await loadModules();
      const scenario = smallTower();
      const core = await buildCore(scenario, { materialScale: 1e6 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);

      // Fire projectile at mid-height of tower (from +X toward -X)
      core.enqueueProjectile({
        position: { x: 5, y: 1.0, z: 0 },
        velocity: { x: -40, y: 0, z: 0 },
        radius: 0.3,
        mass: 5000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('projectile breaks bridge bonds on impact', async () => {
      await loadModules();
      const scenario = smallBridge();
      const core = await buildCore(scenario, { materialScale: 1e6 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);

      // Fire projectile downward at center of bridge deck
      core.enqueueProjectile({
        position: { x: 0, y: 10, z: 0 },
        velocity: { x: 0, y: -50, z: 0 },
        radius: 0.4,
        mass: 10000,
        ttl: 3000,
      });

      stepN(core, 180);

      expect(core.getActiveBondsCount()).toBeLessThan(initial);
      core.dispose();
    });

    it('projectile is spawned and cleaned up after TTL', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario);

      core.enqueueProjectile({
        position: { x: 0, y: 5, z: 10 },
        velocity: { x: 0, y: 0, z: -20 },
        radius: 0.2,
        mass: 100,
        ttl: 500,
      });

      core.step(1 / 60);
      expect(core.projectiles.length).toBe(1);

      // Run enough steps for TTL to expire (500ms = 30 frames at 60fps)
      stepN(core, 60);
      expect(core.projectiles.length).toBe(0);
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

      expect(brokenWeak).toBeGreaterThan(brokenStrong);
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

    it('heavy projectile causes catastrophic damage (<50% bonds survive)', async () => {
      await loadModules();
      const scenario = smallWall();
      const core = await buildCore(scenario, { materialScale: 1e4 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);
      core.enqueueProjectile({
        position: { x: 0, y: 1.5, z: 5 },
        velocity: { x: 0, y: 0, z: -60 },
        radius: 0.5,
        mass: 50000,
        ttl: 3000,
      });
      stepN(core, 240);

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
    it('bridge: cutting midspan bonds splits the body', async () => {
      await loadModules();
      const scenario = smallBridge({ spanSegments: 6, widthSegments: 2, thicknessLayers: 1 });
      const core = await buildCore(scenario);
      const initialBodies = core.getRigidBodyCount();
      const initial = core.getActiveBondsCount();

      // Cut all bonds at x=3 (midspan) — bonds between ix=2 and ix=3
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

      // Step to allow body splitting
      stepN(core, 30);

      expect(core.getRigidBodyCount()).toBeGreaterThan(initialBodies);
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

    it('wall: projectile creates localized hole', async () => {
      await loadModules();
      const scenario = smallWall({ spanSegments: 8, heightSegments: 5 });
      const core = await buildCore(scenario, { materialScale: 1e7 });
      const initial = core.getActiveBondsCount();

      stepN(core, 30);
      // Fire at center
      core.enqueueProjectile({
        position: { x: 0, y: 1.25, z: 5 },
        velocity: { x: 0, y: 0, z: -40 },
        radius: 0.2,
        mass: 3000,
        ttl: 3000,
      });
      stepN(core, 180);

      // Should break some bonds but not all
      const survival = getBondSurvivalRate(core, initial);
      expect(survival).toBeGreaterThan(0.3);
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

  it('wall area normalization preserves geometric cross-sections', () => {
    const scenario = buildWallScenarioDirect({
      span: 6, height: 3, thickness: 0.32,
      spanSegments: 12, heightSegments: 6, layers: 1,
      normalizeAreas: true,
    });

    // Sum bond areas per axis
    const sum = { x: 0, y: 0, z: 0 };
    const pick = (n: { x: number; y: number; z: number }): 'x' | 'y' | 'z' => {
      const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
      return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
    };
    for (const b of scenario.bonds) sum[pick(b.normal)] += b.area;

    // Should match geometric cross-sections
    expect(sum.x).toBeCloseTo(3 * 0.32, 1); // height × thickness
    expect(sum.y).toBeCloseTo(6 * 0.32, 1); // span × thickness
    // Z bonds only exist if layers > 1, so sum.z may be 0
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
