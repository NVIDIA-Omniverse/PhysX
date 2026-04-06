/**
 * Integration tests for the full Rapier destruction pipeline.
 *
 * These tests exercise: WASM solver → Rapier physics → contact forces →
 * fracture detection → body splitting → damage accumulation → projectiles.
 *
 * They require the full build (WASM + TS) and will skip if dist is unavailable.
 * Run: npm run build && npx vitest run src/tests/rapier.integration.test.ts
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

/**
 * Build a simple horizontal bridge scenario:
 *   [support] -- [chunk1] -- [chunk2] -- ... -- [support]
 */
function createBridgeScenario(segmentCount: number, opts?: {
  spacing?: number;
  chunkMass?: number;
  bondArea?: number;
}) {
  const spacing = opts?.spacing ?? 1.0;
  const chunkMass = opts?.chunkMass ?? 1.0;
  const bondArea = opts?.bondArea ?? 1.0;
  const totalNodes = segmentCount + 2;

  const nodes = Array.from({ length: totalNodes }, (_, i) => {
    const isSupport = i === 0 || i === totalNodes - 1;
    return {
      centroid: { x: i * spacing, y: 2, z: 0 },
      mass: isSupport ? 0 : chunkMass,
      volume: spacing * spacing * spacing,
    };
  });

  const bonds = Array.from({ length: totalNodes - 1 }, (_, i) => ({
    node0: i,
    node1: i + 1,
    centroid: { x: (i + 0.5) * spacing, y: 2, z: 0 },
    normal: { x: 1, y: 0, z: 0 },
    area: bondArea,
  }));

  return { nodes, bonds };
}

describe.skipIf(!runtimeAvailable)('buildDestructibleCore integration (requires WASM build)', () => {
  // Lazily loaded — only when tests actually run
  let buildDestructibleCore: (opts: any) => Promise<any>;

  it('creates a core with correct chunk count and API surface', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(3);
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 1.0 });

    expect(core.chunks).toHaveLength(5);
    expect(typeof core.step).toBe('function');
    expect(typeof core.dispose).toBe('function');
    expect(typeof core.enqueueProjectile).toBe('function');
    expect(typeof core.getActiveBondsCount).toBe('function');
    expect(typeof core.getNodeBonds).toBe('function');
    expect(typeof core.cutBond).toBe('function');
    expect(core.getActiveBondsCount()).toBe(4);
    core.dispose();
  });

  it('runs physics steps without crashing', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(2);
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 5.0 });

    for (let i = 0; i < 60; i++) core.step(1 / 60);
    expect(core.getRigidBodyCount()).toBeGreaterThan(0);
    core.dispose();
  });

  it('fractures bonds under sufficient gravity stress', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(6, { spacing: 1.0, chunkMass: 5.0, bondArea: 0.5 });
    const core = await buildDestructibleCore({ scenario, gravity: -20, materialScale: 0.1 });

    const initialBonds = core.getActiveBondsCount();
    for (let i = 0; i < 120; i++) core.step(1 / 60);
    expect(core.getActiveBondsCount()).toBeLessThan(initialBonds);
    core.dispose();
  });

  it('cutBond removes a specific bond', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(2);
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 10 });

    const bonds = core.getNodeBonds(1);
    expect(bonds.length).toBeGreaterThan(0);
    expect(core.cutBond(bonds[0].index)).toBe(true);
    expect(core.cutBond(bonds[0].index)).toBe(false);
    core.dispose();
  });

  it('damage system destroys chunks', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(3);
    const destroyed: number[] = [];
    const core = await buildDestructibleCore({
      scenario,
      gravity: -9.81,
      materialScale: 10,
      damage: { enabled: true, strengthPerVolume: 100, autoDetachOnDestroy: true },
      onNodeDestroyed: (e: any) => destroyed.push(e.nodeIndex),
    });

    const h = core.getNodeHealth(1);
    core.applyNodeDamage(1, h.maxHealth + 1);
    core.step(1 / 60);
    expect(destroyed).toContain(1);
    core.dispose();
  });

  it('projectiles are spawned and cleaned up', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(2);
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 10 });

    core.enqueueProjectile({ position: { x: 2, y: 5, z: 0 }, velocity: { x: 0, y: -10, z: 0 }, radius: 0.2, mass: 3, ttl: 0.5 });
    core.step(1 / 60);
    expect(core.projectiles.length).toBe(1);

    for (let i = 0; i < 60; i++) core.step(1 / 60);
    expect(core.projectiles.length).toBe(0);
    core.dispose();
  });

  it('profiler collects timing data', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(2);
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 10 });

    const samples: any[] = [];
    core.setProfiler({ enabled: true, onSample: (s: any) => samples.push(s) });
    core.step(1 / 60);
    core.step(1 / 60);
    expect(samples.length).toBe(2);
    expect(typeof samples[0].totalMs).toBe('number');
    core.dispose();
  });

  it('strong bridge survives moderate load', async () => {
    const mod = await import('../../dist/rapier.js');
    buildDestructibleCore = mod.buildDestructibleCore;

    const scenario = createBridgeScenario(3, { chunkMass: 0.5, bondArea: 2.0 });
    const core = await buildDestructibleCore({ scenario, gravity: -9.81, materialScale: 5.0 });

    const initialBonds = core.getActiveBondsCount();
    for (let i = 0; i < 120; i++) core.step(1 / 60);
    expect(core.getActiveBondsCount()).toBe(initialBonds);
    core.dispose();
  });
});

describe('bridge scenario helpers (always run)', () => {
  it('createBridgeScenario produces correct node/bond structure', () => {
    const scenario = createBridgeScenario(4, { spacing: 2, chunkMass: 3, bondArea: 0.5 });

    expect(scenario.nodes).toHaveLength(6); // 4 interior + 2 supports
    expect(scenario.bonds).toHaveLength(5); // N-1 bonds for N nodes

    // Supports at ends
    expect(scenario.nodes[0].mass).toBe(0);
    expect(scenario.nodes[5].mass).toBe(0);

    // Interior chunks
    for (let i = 1; i <= 4; i++) {
      expect(scenario.nodes[i].mass).toBe(3);
    }

    // Bond connectivity
    for (let i = 0; i < 5; i++) {
      expect(scenario.bonds[i].node0).toBe(i);
      expect(scenario.bonds[i].node1).toBe(i + 1);
      expect(scenario.bonds[i].area).toBe(0.5);
    }

    // Spacing
    expect(scenario.nodes[1].centroid.x).toBe(2);
    expect(scenario.nodes[2].centroid.x).toBe(4);
  });
});
