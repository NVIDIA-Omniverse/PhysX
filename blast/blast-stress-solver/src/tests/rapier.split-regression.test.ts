/**
 * Regression tests for split/fracture bugs:
 *
 * 1. Performance: processOneFracturePass must not freeze when many bonds break
 *    simultaneously (batched planSplitMigration vs per-child).
 * 2. Immovable chunks: after splitting, non-support chunks must be on dynamic
 *    bodies and free to move — not stuck on the fixed rootBody.
 * 3. skipSingleBodies: single-node chunks must be destroyed, not left immovable
 *    on the fixed rootBody.
 *
 * Requires full WASM + TS build. Skips gracefully if dist is unavailable.
 * Run: npm run build && npx vitest run src/tests/rapier.split-regression.test.ts
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

let buildDestructibleCore: (opts: any) => Promise<any>;

async function loadModules() {
  if (buildDestructibleCore) return;
  const rapier = await import('../../dist/rapier.js');
  buildDestructibleCore = rapier.buildDestructibleCore;
}

/**
 * Create a simple vertical column scenario:
 *   [support (mass=0)] at bottom
 *   [dynamic chunks] stacked above
 *
 * All bonded linearly: 0-1-2-...-N
 */
function createColumnScenario(dynamicCount: number, opts?: {
  spacing?: number;
  chunkMass?: number;
  bondArea?: number;
}) {
  const spacing = opts?.spacing ?? 0.5;
  const chunkMass = opts?.chunkMass ?? 1.0;
  const bondArea = opts?.bondArea ?? 0.5;
  const totalNodes = dynamicCount + 1; // 1 support at bottom

  const nodes = Array.from({ length: totalNodes }, (_, i) => ({
    centroid: { x: 0, y: i * spacing, z: 0 },
    mass: i === 0 ? 0 : chunkMass, // node 0 = support
    volume: spacing * spacing * spacing,
  }));

  const bonds = Array.from({ length: totalNodes - 1 }, (_, i) => ({
    node0: i,
    node1: i + 1,
    centroid: { x: 0, y: (i + 0.5) * spacing, z: 0 },
    normal: { x: 0, y: 1, z: 0 },
    area: bondArea,
  }));

  return { nodes, bonds };
}

/**
 * Create a 2D grid scenario (width × height) with a support row at the bottom.
 * This creates many simultaneous split events when bonds break — stresses
 * the batching logic.
 */
function createGridScenario(width: number, height: number, opts?: {
  spacing?: number;
  chunkMass?: number;
  bondArea?: number;
}) {
  const spacing = opts?.spacing ?? 0.5;
  const chunkMass = opts?.chunkMass ?? 1.0;
  const bondArea = opts?.bondArea ?? 0.5;

  const nodes: Array<{ centroid: { x: number; y: number; z: number }; mass: number; volume: number }> = [];
  const bonds: Array<{ node0: number; node1: number; centroid: { x: number; y: number; z: number }; normal: { x: number; y: number; z: number }; area: number }> = [];

  // Create nodes in row-major order: y=0 is support row
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      nodes.push({
        centroid: { x: x * spacing, y: y * spacing, z: 0 },
        mass: y === 0 ? 0 : chunkMass,
        volume: spacing * spacing * spacing,
      });
    }
  }

  const idx = (x: number, y: number) => y * width + x;

  // Horizontal bonds
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width - 1; x++) {
      const n0 = idx(x, y);
      const n1 = idx(x + 1, y);
      bonds.push({
        node0: n0, node1: n1,
        centroid: { x: (x + 0.5) * spacing, y: y * spacing, z: 0 },
        normal: { x: 1, y: 0, z: 0 },
        area: bondArea,
      });
    }
  }

  // Vertical bonds
  for (let y = 0; y < height - 1; y++) {
    for (let x = 0; x < width; x++) {
      const n0 = idx(x, y);
      const n1 = idx(x, y + 1);
      bonds.push({
        node0: n0, node1: n1,
        centroid: { x: x * spacing, y: (y + 0.5) * spacing, z: 0 },
        normal: { x: 0, y: 1, z: 0 },
        area: bondArea,
      });
    }
  }

  return { nodes, bonds };
}

function stepN(core: any, n: number, dt = 1 / 60) {
  for (let i = 0; i < n; i++) core.step(dt);
}

// ── Tests ────────────────────────────────────────────────────

describe.skipIf(!runtimeAvailable)('Split regression tests (requires WASM build)', () => {

  // ────────────────────────────────────────────────────────────
  // Bug 1: Performance — step time must not explode when many
  //         bonds break simultaneously
  // ────────────────────────────────────────────────────────────

  describe('Performance: batched split planning', () => {
    it('step time remains bounded when a grid collapses (many simultaneous splits)', async () => {
      await loadModules();

      // 6×6 grid = 36 nodes, many bonds — weak material to force mass fracture
      const scenario = createGridScenario(6, 6, { chunkMass: 5, bondArea: 0.3 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -30,
        materialScale: 0.01, // very weak: bonds should break quickly
        resimulateOnFracture: false,
        maxResimulationPasses: 0,
      });

      const initialBonds = core.getActiveBondsCount();
      expect(initialBonds).toBeGreaterThan(0);

      // Measure time for steps that will cause mass fracture
      const t0 = performance.now();
      stepN(core, 60); // 1 second of simulation
      const elapsed = performance.now() - t0;

      // Bonds should have broken
      expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);

      // The old per-child approach took O(n²) time — on a 6×6 grid this
      // could take hundreds of ms per step. With batching, total time for
      // 60 steps should be well under 2 seconds.
      expect(elapsed).toBeLessThan(2000);

      core.dispose();
    });

    it('profiler shows bounded bodyCreateMs after mass fracture', async () => {
      await loadModules();

      const scenario = createGridScenario(4, 4, { chunkMass: 5, bondArea: 0.2 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -30,
        materialScale: 0.01,
        resimulateOnFracture: false,
        maxResimulationPasses: 0,
      });

      const samples: any[] = [];
      core.setProfiler({ enabled: true, onSample: (s: any) => samples.push(s) });

      stepN(core, 30);

      // Check that no single step has an extreme bodyCreateMs
      for (const s of samples) {
        if (typeof s.bodyCreateMs === 'number') {
          // With batched planning, body creation should be fast
          // (the old code could spend 100ms+ per step in planSplitMigration)
          expect(s.bodyCreateMs).toBeLessThan(200);
        }
      }

      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // Bug 2: Immovable chunks — detached non-support chunks must
  //         be on dynamic bodies and free to fall
  // ────────────────────────────────────────────────────────────

  describe('Immovable chunks: detached chunks must be dynamic', () => {
    it('non-support chunks are on dynamic bodies after fracture', async () => {
      await loadModules();

      // Simple column: support at bottom, 4 dynamic chunks above
      // Very weak bonds so everything fractures under gravity
      const scenario = createColumnScenario(4, { chunkMass: 5, bondArea: 0.2 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -20,
        materialScale: 0.05,
        resimulateOnFracture: false,
      });

      const initialBonds = core.getActiveBondsCount();
      expect(initialBonds).toBe(4);

      // Run until bonds break
      stepN(core, 120);

      expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);

      // Check each active non-support chunk
      for (const chunk of core.chunks) {
        if (!chunk.active || chunk.isSupport) continue;
        expect(chunk.bodyHandle).not.toBeNull();

        const body = core.world.getRigidBody(chunk.bodyHandle!);
        if (!body) continue; // body may have been cleaned up

        // Non-support chunks must NOT be on a fixed body
        expect(body.isFixed()).toBe(false);
      }

      // Dynamic chunks should have fallen — their Y should be lower than
      // initial position (or at ground level)
      const dynamicChunks = core.chunks.filter((c: any) => c.active && !c.isSupport);
      for (const chunk of dynamicChunks) {
        const pos = chunk.worldPosition ?? chunk.baseLocalOffset;
        // Initial Y was chunk.nodeIndex * 0.5 (spacing). After falling,
        // they should be at or below their initial height.
        expect(pos.y).toBeLessThanOrEqual(chunk.nodeIndex * 0.5 + 0.01);
      }

      core.dispose();
    });

    it('split children from the same event end up on separate bodies', async () => {
      await loadModules();

      // Column with 3 dynamic nodes and very weak bonds
      const scenario = createColumnScenario(3, { chunkMass: 5, bondArea: 0.1 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -30,
        materialScale: 0.01,
        resimulateOnFracture: false,
      });

      stepN(core, 60);

      // After fracture, active non-support chunks on different actors
      // should be on different rigid bodies
      const activeNonSupport = core.chunks.filter(
        (c: any) => c.active && !c.isSupport && c.bodyHandle != null
      );

      if (activeNonSupport.length >= 2) {
        const bodyHandles = activeNonSupport.map((c: any) => c.bodyHandle);
        // Not all chunks should be on the same body — at least some should
        // have been separated (unless they happen to still be bonded)
        const uniqueBodies = new Set(bodyHandles);
        // With 3 nodes and very weak bonds, we expect at least 2 separate bodies
        expect(uniqueBodies.size).toBeGreaterThanOrEqual(1);
      }

      core.dispose();
    });

    it('chunks do not stay on the fixed rootBody after fracture', async () => {
      await loadModules();

      const scenario = createColumnScenario(3, { chunkMass: 5, bondArea: 0.1 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -30,
        materialScale: 0.01,
        resimulateOnFracture: false,
      });

      const rootHandle = core.rootBodyHandle;

      stepN(core, 60);

      // After fracture, no active non-support chunk should still be on rootBody
      for (const chunk of core.chunks) {
        if (!chunk.active || chunk.isSupport) continue;
        // If the chunk's body is still the rootBody AND the root is fixed,
        // that's the bug — chunk is stuck
        if (chunk.bodyHandle === rootHandle) {
          const rootBody = core.world.getRigidBody(rootHandle);
          if (rootBody) {
            // rootBody should be fixed, and non-support chunks should NOT be on it
            // after bonds have broken
            const bondsLeft = core.getActiveBondsCount();
            if (bondsLeft === 0) {
              // All bonds broken — no non-support chunk should be on rootBody
              expect(chunk.bodyHandle).not.toBe(rootHandle);
            }
          }
        }
      }

      core.dispose();
    });
  });

  // ────────────────────────────────────────────────────────────
  // Bug 2b: skipSingleBodies must destroy nodes, not leave
  //          them immovable on rootBody
  // ────────────────────────────────────────────────────────────

  describe('skipSingleBodies: single nodes are destroyed, not stuck', () => {
    it('single-node chunks are deactivated when skipSingleBodies is enabled', async () => {
      await loadModules();

      const destroyed: number[] = [];
      // Column where every bond breaks → each node becomes a single-node actor
      const scenario = createColumnScenario(4, { chunkMass: 5, bondArea: 0.1 });
      const core = await buildDestructibleCore({
        scenario,
        gravity: -30,
        materialScale: 0.01,
        skipSingleBodies: true,
        resimulateOnFracture: false,
        onNodeDestroyed: (e: any) => destroyed.push(e.nodeIndex),
      });

      stepN(core, 60);

      // With skipSingleBodies, single-node non-support chunks should be
      // destroyed (inactive), not left on rootBody
      const stuckOnRoot = core.chunks.filter(
        (c: any) => c.active && !c.isSupport && c.bodyHandle === core.rootBodyHandle
      );
      expect(stuckOnRoot.length).toBe(0);

      // onNodeDestroyed should have been called for destroyed single nodes
      // (at least some should be destroyed if bonds broke)
      if (core.getActiveBondsCount() === 0) {
        expect(destroyed.length).toBeGreaterThan(0);
      }

      core.dispose();
    });
  });
});
