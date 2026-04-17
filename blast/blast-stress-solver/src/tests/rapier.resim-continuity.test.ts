import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = resolve(here, '../../dist/stress_solver.wasm');
const runtimeAvailable = existsSync(wasmPath);

type Vec3 = { x: number; y: number; z: number };

type DebugSplitContinuityRecord = {
  phase: 'migration' | 'restore';
  sourceBodyHandle: number;
  targetBodyHandle: number;
  nodeIndices: number[];
  sourceBodyIsFixed: boolean;
  targetBodyIsFixed: boolean;
  translationError: number;
  rotationError: number;
  linearVelocityError: number;
  angularVelocityError: number;
  maxChunkWorldPositionError: number;
  maxChunkPointVelocityError: number;
};

type DebuggableCore = {
  step: (dt?: number) => void;
  dispose: () => void;
  enqueueProjectile: (spawn: {
    position: Vec3;
    velocity: Vec3;
    radius?: number;
    mass?: number;
    ttl?: number;
  }) => void;
  __debugSplitContinuityLog?: DebugSplitContinuityRecord[];
  __clearDebugSplitContinuityLog?: () => void;
};

let buildDestructibleCore: (opts: any) => Promise<DebuggableCore>;
let buildWallScenario: (opts?: any) => any;
let buildBeamBridgeScenario: (opts?: any) => any;

async function loadModules() {
  if (buildDestructibleCore) return;
  const rapier = await import('../../dist/rapier.js');
  const scenarios = await import('../../dist/scenarios.js');
  buildDestructibleCore = rapier.buildDestructibleCore;
  buildWallScenario = scenarios.buildWallScenario;
  buildBeamBridgeScenario = scenarios.buildBeamBridgeScenario;
}

function stepN(core: DebuggableCore, count: number, dt = 1 / 60) {
  for (let index = 0; index < count; index += 1) {
    core.step(dt);
  }
}

function createCascadingGridScenario() {
  const rows = 4;
  const cols = 3;
  const nodes: Array<{ centroid: Vec3; mass: number; volume: number }> = [];
  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      nodes.push({
        centroid: { x: col, y: row + 0.5, z: 0 },
        mass: row === 0 ? 0 : 5,
        volume: 1,
      });
    }
  }

  const bonds: Array<{ node0: number; node1: number; centroid: Vec3; normal: Vec3; area: number }> = [];
  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols - 1; col += 1) {
      const node0 = row * cols + col;
      const node1 = row * cols + col + 1;
      bonds.push({
        node0,
        node1,
        centroid: { x: col + 0.5, y: row + 0.5, z: 0 },
        normal: { x: 1, y: 0, z: 0 },
        area: 2.0,
      });
    }
  }
  for (let row = 0; row < rows - 1; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      const node0 = row * cols + col;
      const node1 = (row + 1) * cols + col;
      bonds.push({
        node0,
        node1,
        centroid: { x: col, y: row + 1.0, z: 0 },
        normal: { x: 0, y: 1, z: 0 },
        area: 0.01,
      });
    }
  }

  return { nodes, bonds };
}

function getDynamicRestoreRecords(core: DebuggableCore) {
  return (core.__debugSplitContinuityLog ?? []).filter(
    (record) =>
      record.phase === 'restore' &&
      !record.sourceBodyIsFixed &&
      !record.targetBodyIsFixed &&
      record.sourceBodyHandle !== record.targetBodyHandle &&
      record.nodeIndices.length > 0,
  );
}

async function runDeterministicCascade(snapshotMode?: 'perBody' | 'world') {
  const core = await buildDestructibleCore({
    scenario: createCascadingGridScenario(),
    gravity: -20,
    materialScale: 0.001,
    resimulateOnFracture: true,
    maxResimulationPasses: 5,
    snapshotMode,
    skipSingleBodies: false,
  });

  core.__clearDebugSplitContinuityLog?.();
  stepN(core, 220);
  return core;
}

function assertContinuity(records: DebugSplitContinuityRecord[]) {
  expect(records.length).toBeGreaterThan(0);
  for (const record of records) {
    expect(record.translationError).toBeLessThan(1e-6);
    expect(record.rotationError).toBeLessThan(1e-6);
    // Raw COM linear velocity can differ after fracture because the child
    // body's center of mass no longer matches the parent's. The physically
    // correct invariant is point-velocity continuity at the migrated chunks.
    expect(Number.isFinite(record.linearVelocityError)).toBe(true);
    expect(record.angularVelocityError).toBeLessThan(1e-6);
    expect(record.maxChunkWorldPositionError).toBeLessThan(1e-3);
    expect(record.maxChunkPointVelocityError).toBeLessThan(1e-3);
  }
}

describe.skipIf(!runtimeAvailable)('Resim continuity after fracture (requires WASM build)', () => {
  it('defaults to the per-body rollback path', async () => {
    await loadModules();

    const implicitCore = await runDeterministicCascade();
    const implicitRecords = getDynamicRestoreRecords(implicitCore);

    const explicitCore = await runDeterministicCascade('perBody');
    const explicitRecords = getDynamicRestoreRecords(explicitCore);

    expect(implicitRecords.length).toBeGreaterThan(0);
    expect(implicitRecords).toEqual(explicitRecords);

    implicitCore.dispose();
    explicitCore.dispose();
  });

  it('keeps second-level fracture bodies continuous in a deterministic cascade', async () => {
    await loadModules();
    const core = await runDeterministicCascade('perBody');

    assertContinuity(getDynamicRestoreRecords(core));
    core.dispose();
  });

  it('keeps wall fragments continuous through projectile-driven resim', async () => {
    await loadModules();
    const core = await buildDestructibleCore({
      scenario: buildWallScenario({
        spanSegments: 6,
        heightSegments: 5,
        deckMass: 3_000,
        layers: 1,
      }),
      gravity: -9.81,
      materialScale: 1e6,
      resimulateOnFracture: true,
      maxResimulationPasses: 3,
      snapshotMode: 'perBody',
      contactForceScale: 30,
      debrisCleanup: { mode: 'off' },
    });

    core.__clearDebugSplitContinuityLog?.();
    stepN(core, 30);
    core.enqueueProjectile({
      position: { x: 0, y: 1.6, z: 5 },
      velocity: { x: 0, y: 0, z: -35 },
      radius: 0.35,
      mass: 15_000,
      ttl: 5,
    });
    stepN(core, 260);

    assertContinuity(getDynamicRestoreRecords(core));
    core.dispose();
  });

  it('keeps bridge fragments continuous through side-impact resim', async () => {
    await loadModules();
    const core = await buildDestructibleCore({
      scenario: buildBeamBridgeScenario({
        span: 8,
        deckWidth: 3,
        spanSegments: 10,
        widthSegments: 4,
        thicknessLayers: 2,
        deckMass: 10_000,
        pierHeight: 1.5,
        supportsPerSide: 2,
      }),
      gravity: -9.81,
      materialScale: 1e7,
      resimulateOnFracture: true,
      maxResimulationPasses: 3,
      snapshotMode: 'perBody',
      contactForceScale: 30,
      debrisCleanup: { mode: 'off' },
    });

    core.__clearDebugSplitContinuityLog?.();
    stepN(core, 40);
    core.enqueueProjectile({
      position: { x: -5, y: 1.5, z: 0 },
      velocity: { x: 35, y: 0, z: 0 },
      radius: 0.35,
      mass: 18_000,
      ttl: 5,
    });
    stepN(core, 340);

    assertContinuity(getDynamicRestoreRecords(core));
    core.dispose();
  });
});
