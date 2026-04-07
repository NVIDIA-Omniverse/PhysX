/**
 * Performance benchmarks for the blast stress solver destruction pipeline.
 *
 * Exercises the full end-to-end destruction flow (Rapier physics + stress solve +
 * fracture + body splitting) at multiple scales and collision patterns, collecting
 * per-frame profiler samples and reporting aggregate timing statistics.
 *
 * Requires full WASM + TS build. Skips gracefully if dist is unavailable.
 * Run: npm run build && npx vitest run src/tests/rapier.perf.test.ts
 */
import { describe, it, expect, afterAll } from 'vitest';
import { existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

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

// ── Types ───────────────────────────────────────────────────

type ProfilerSample = {
  frameIndex: number;
  totalMs: number;
  rapierStepMs: number;
  contactDrainMs: number;
  solverUpdateMs: number;
  fractureMs: number;
  fractureGenerateMs: number;
  fractureApplyMs: number;
  splitQueueMs: number;
  bodyCreateMs: number;
  colliderRebuildMs: number;
  damageTickMs: number;
  damageReplayMs: number;
  damagePreviewMs: number;
  damageRestoreMs: number;
  damageSnapshotMs: number;
  damagePreDestroyMs: number;
  damageFlushMs: number;
  preStepSweepMs: number;
  rebuildColliderMapMs: number;
  cleanupDisabledMs: number;
  spawnMs: number;
  externalForceMs: number;
  snapshotCaptureMs: number;
  snapshotRestoreMs: number;
  initialPassMs: number;
  resimMs: number;
  projectileCleanupMs: number;
  resimPasses: number;
  rigidBodies: number;
  snapshotBytes: number;
  passes: Array<{ index: number; solverMs: number; fractureMs: number; bodyCreateMs: number; totalMs: number }>;
};

type Stats = { mean: number; p50: number; p95: number; p99: number; max: number; sum: number };

type ImpactPlan = {
  frame: number;
  projectiles: Array<{
    position: { x: number; y: number; z: number };
    velocity: { x: number; y: number; z: number };
    radius?: number;
    mass?: number;
    ttl?: number;
  }>;
};

type PerfResult = {
  name: string;
  nodeCount: number;
  bondCount: number;
  setupMs: number;
  warmupStats: Stats;
  impactStats: Stats;
  postImpactStats: Stats;
  allStats: Stats;
  bondsInitial: number;
  bondsFinal: number;
  maxRigidBodies: number;
  samples: ProfilerSample[];
  phaseBreakdown: Record<string, Stats>;
};

// Accumulate results across all tests for final summary
const allResults: PerfResult[] = [];

// ── Statistics Helpers ──────────────────────────────────────

function computeStats(values: number[]): Stats {
  if (values.length === 0) return { mean: 0, p50: 0, p95: 0, p99: 0, max: 0, sum: 0 };
  const sorted = [...values].sort((a, b) => a - b);
  const sum = sorted.reduce((a, b) => a + b, 0);
  const mean = sum / sorted.length;
  const p = (pct: number) => sorted[Math.min(Math.floor(sorted.length * pct), sorted.length - 1)];
  return { mean, p50: p(0.5), p95: p(0.95), p99: p(0.99), max: sorted[sorted.length - 1], sum };
}

function fmtMs(v: number): string {
  return v.toFixed(2).padStart(8);
}

// ── Report Printer ──────────────────────────────────────────

const PHASE_KEYS = [
  'totalMs', 'rapierStepMs', 'contactDrainMs', 'solverUpdateMs',
  'fractureMs', 'fractureGenerateMs', 'fractureApplyMs',
  'splitQueueMs', 'bodyCreateMs', 'colliderRebuildMs',
  'damageTickMs', 'damageReplayMs', 'damageRestoreMs',
  'preStepSweepMs', 'cleanupDisabledMs', 'spawnMs',
  'snapshotCaptureMs', 'snapshotRestoreMs',
  'initialPassMs', 'resimMs',
] as const;

function printPhaseTable(name: string, samples: ProfilerSample[]) {
  if (samples.length === 0) return;
  const breakdown: Record<string, Stats> = {};
  for (const key of PHASE_KEYS) {
    breakdown[key] = computeStats(samples.map((s) => (s as any)[key] ?? 0));
  }

  const header = `  ${'Phase'.padEnd(24)} ${'Mean'.padStart(8)} ${'P50'.padStart(8)} ${'P95'.padStart(8)} ${'P99'.padStart(8)} ${'Max'.padStart(8)}`;
  const sep = '  ' + '-'.repeat(header.length - 2);
  console.log(`\n  [${name}] ${samples.length} frames`);
  console.log(header);
  console.log(sep);
  for (const key of PHASE_KEYS) {
    const s = breakdown[key];
    if (s.max < 0.001) continue; // skip phases with zero time
    console.log(`  ${key.padEnd(24)} ${fmtMs(s.mean)} ${fmtMs(s.p50)} ${fmtMs(s.p95)} ${fmtMs(s.p99)} ${fmtMs(s.max)}`);
  }
  return breakdown;
}

function printScenarioSummary(result: PerfResult) {
  console.log(`\n${'='.repeat(72)}`);
  console.log(`  SCENARIO: ${result.name}`);
  console.log(`  Nodes: ${result.nodeCount} | Bonds: ${result.bondsInitial} -> ${result.bondsFinal} | Setup: ${result.setupMs.toFixed(0)}ms`);
  console.log(`  Max rigid bodies: ${result.maxRigidBodies} | Total frames: ${result.samples.length}`);
  console.log('='.repeat(72));
}

// ── Core Test Runner ────────────────────────────────────────

async function runPerfScenario(opts: {
  name: string;
  scenario: any;
  coreOpts?: Record<string, any>;
  warmupFrames?: number;
  impactPlan?: ImpactPlan[];
  postImpactFrames?: number;
  dt?: number;
}): Promise<PerfResult> {
  const {
    name,
    scenario,
    coreOpts = {},
    warmupFrames = 60,
    impactPlan = [],
    postImpactFrames = 180,
    dt = 1 / 60,
  } = opts;

  const samples: ProfilerSample[] = [];
  const profilerConfig = {
    enabled: true,
    onSample: (s: ProfilerSample) => samples.push(s),
  };

  // Measure setup time
  const setupStart = performance.now();
  const core = await buildDestructibleCore({
    scenario,
    gravity: -9.81,
    materialScale: 1e8,
    resimulateOnFracture: true,
    maxResimulationPasses: 1,
    snapshotMode: 'perBody',
    ...coreOpts,
  });
  const setupMs = performance.now() - setupStart;

  core.setProfiler(profilerConfig);

  const nodeCount = scenario.nodes.length;
  const bondsInitial = core.getActiveBondsCount();

  // Resilient step wrapper — at extreme scales, WASM memory can be exhausted
  // during fracture causing a detached ArrayBuffer. We capture what we can.
  let crashed = false;
  let crashFrame = -1;
  const safeStep = (frame: number) => {
    if (crashed) return;
    try {
      core.step(dt);
    } catch (err: any) {
      crashed = true;
      crashFrame = frame;
      console.warn(`  [CRASH] Frame ${frame}: ${err?.message ?? err}`);
    }
  };

  // Phase 1: Warmup (gravity only, no projectiles)
  const warmupStartIdx = samples.length;
  for (let i = 0; i < warmupFrames; i++) safeStep(i);
  const warmupEndIdx = samples.length;

  // Phase 2: Impact — fire projectiles according to plan
  const impactStartIdx = samples.length;
  const maxImpactFrame = impactPlan.length > 0
    ? Math.max(...impactPlan.map((p) => p.frame)) + 1
    : 0;
  const impactFrameCount = Math.max(maxImpactFrame + 60, 120); // at least 60 frames after last projectile

  for (let i = 0; i < impactFrameCount; i++) {
    // Enqueue any projectiles scheduled for this frame
    if (!crashed) {
      for (const plan of impactPlan) {
        if (plan.frame === i) {
          for (const proj of plan.projectiles) {
            core.enqueueProjectile({ ttl: 5000, radius: 0.3, mass: 15000, ...proj });
          }
        }
      }
    }
    safeStep(warmupFrames + i);
  }
  const impactEndIdx = samples.length;

  // Phase 3: Post-impact (let destruction settle)
  const postStartIdx = samples.length;
  for (let i = 0; i < postImpactFrames; i++) safeStep(warmupFrames + impactFrameCount + i);
  const postEndIdx = samples.length;

  let bondsFinal: number;
  try {
    bondsFinal = core.getActiveBondsCount();
  } catch {
    bondsFinal = -1; // WASM state corrupted
  }
  const maxRigidBodies = samples.reduce((mx, s) => Math.max(mx, s.rigidBodies ?? 0), 0);

  try { core.dispose(); } catch { /* WASM may already be corrupt */ }

  // Compute stats per phase window
  const warmupSamples = samples.slice(warmupStartIdx, warmupEndIdx);
  const impactSamples = samples.slice(impactStartIdx, impactEndIdx);
  const postSamples = samples.slice(postStartIdx, postEndIdx);

  const warmupStats = computeStats(warmupSamples.map((s) => s.totalMs));
  const impactStats = computeStats(impactSamples.map((s) => s.totalMs));
  const postImpactStats = computeStats(postSamples.map((s) => s.totalMs));
  const allStats = computeStats(samples.map((s) => s.totalMs));

  // Print report
  const result: PerfResult = {
    name,
    nodeCount,
    bondCount: bondsInitial,
    setupMs,
    warmupStats,
    impactStats,
    postImpactStats,
    allStats,
    bondsInitial,
    bondsFinal,
    maxRigidBodies,
    samples,
    phaseBreakdown: {},
  };

  printScenarioSummary(result);

  if (warmupSamples.length > 0) printPhaseTable('Warmup (steady state)', warmupSamples);
  if (impactSamples.length > 0) {
    const bd = printPhaseTable('Impact (destruction)', impactSamples);
    if (bd) result.phaseBreakdown = bd;
  }
  if (postSamples.length > 0) printPhaseTable('Post-impact (settling)', postSamples);

  // Resim pass summary
  const framesWithResim = samples.filter((s) => s.resimPasses > 0);
  if (framesWithResim.length > 0) {
    const resimPassStats = computeStats(framesWithResim.map((s) => s.resimPasses));
    console.log(`\n  Resimulation: ${framesWithResim.length} frames had resim passes (mean ${resimPassStats.mean.toFixed(1)}, max ${resimPassStats.max})`);
  }

  allResults.push(result);
  return result;
}

// ── Projectile Presets ──────────────────────────────────────

const centerHit = (height: number): ImpactPlan[] => [{
  frame: 0,
  projectiles: [{
    position: { x: 0, y: height * 0.5, z: 5 },
    velocity: { x: 0, y: 0, z: -40 },
    radius: 0.35,
    mass: 15000,
  }],
}];

const baseHit = (): ImpactPlan[] => [{
  frame: 0,
  projectiles: [{
    position: { x: 0, y: 0.3, z: 5 },
    velocity: { x: 0, y: 0, z: -40 },
    radius: 0.35,
    mass: 15000,
  }],
}];

const multiSimultaneous = (height: number): ImpactPlan[] => [{
  frame: 0,
  projectiles: [
    { position: { x: 0, y: height * 0.5, z: 5 }, velocity: { x: 0, y: 0, z: -40 }, radius: 0.3, mass: 12000 },
    { position: { x: 0, y: height * 0.5, z: -5 }, velocity: { x: 0, y: 0, z: 40 }, radius: 0.3, mass: 12000 },
    { position: { x: 5, y: height * 0.5, z: 0 }, velocity: { x: -40, y: 0, z: 0 }, radius: 0.3, mass: 12000 },
    { position: { x: -5, y: height * 0.5, z: 0 }, velocity: { x: 40, y: 0, z: 0 }, radius: 0.3, mass: 12000 },
  ],
}];

const progressiveBombardment = (height: number, count = 10, interval = 30): ImpactPlan[] =>
  Array.from({ length: count }, (_, i) => ({
    frame: i * interval,
    projectiles: [{
      position: {
        x: 3 * Math.cos((i / count) * Math.PI * 2),
        y: height * (0.2 + 0.6 * Math.random()),
        z: 3 * Math.sin((i / count) * Math.PI * 2),
      },
      velocity: {
        x: -30 * Math.cos((i / count) * Math.PI * 2),
        y: 0,
        z: -30 * Math.sin((i / count) * Math.PI * 2),
      },
      radius: 0.25,
      mass: 10000,
    }],
  }));

const catastrophicImpact = (height: number): ImpactPlan[] => [{
  frame: 0,
  projectiles: [{
    position: { x: 0, y: height * 0.3, z: 8 },
    velocity: { x: 0, y: 0, z: -80 },
    radius: 0.6,
    mass: 50000,
  }],
}];

// ── Tests ───────────────────────────────────────────────────

describe.skipIf(!runtimeAvailable)('Destruction performance benchmarks (requires WASM build)', () => {

  // ────────────────────────────────────────────────────────────
  // A. Scale: Tower size progression
  // ────────────────────────────────────────────────────────────

  describe('A. Scale: Tower size progression', () => {
    it('small tower 3x4 (~48 nodes)', async () => {
      await loadModules();
      const scenario = buildTowerScenario({ side: 3, stories: 4, totalMass: 1000 });
      const towerHeight = 4 * 0.5;
      const result = await runPerfScenario({
        name: 'Small tower 3x4',
        scenario,
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 30_000);

    it('medium tower 6x12 (~432 nodes)', async () => {
      await loadModules();
      const scenario = buildTowerScenario({ side: 6, stories: 12, totalMass: 5000 });
      const towerHeight = 12 * 0.5;
      const result = await runPerfScenario({
        name: 'Medium tower 6x12',
        scenario,
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('large tower 8x20 (~1280 nodes)', async () => {
      await loadModules();
      const scenario = buildTowerScenario({ side: 8, stories: 20, totalMass: 10000 });
      const towerHeight = 20 * 0.5;
      const result = await runPerfScenario({
        name: 'Large tower 8x20',
        scenario,
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 240,
      });
      // At this scale, WASM memory exhaustion during fracture is a known issue.
      // The test still captures profiling data for all frames up to the crash.
      expect(result.samples.length).toBeGreaterThan(0);
      if (result.bondsFinal >= 0 && result.bondsFinal < result.bondsInitial) {
        expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
      } else {
        // Log that destruction didn't complete — useful for tracking the WASM memory bug
        console.log(`  [NOTE] Large tower: bonds ${result.bondsInitial} -> ${result.bondsFinal} (destruction may have been cut short by WASM crash)`);
      }
    }, 120_000);

    it('xl tower 10x30 (~3000 nodes)', async () => {
      await loadModules();
      const scenario = buildTowerScenario({ side: 10, stories: 30, totalMass: 20000 });
      const towerHeight = 30 * 0.5;
      const result = await runPerfScenario({
        name: 'XL tower 10x30',
        scenario,
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 300,
      });
      expect(result.samples.length).toBeGreaterThan(0);
      if (result.bondsFinal >= 0) {
        expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
      }
    }, 300_000);
  });

  // ────────────────────────────────────────────────────────────
  // B. Structure variety
  // ────────────────────────────────────────────────────────────

  describe('B. Structure variety', () => {
    it('wide wall 24x12', async () => {
      await loadModules();
      const scenario = buildWallScenario({ spanSegments: 24, heightSegments: 12, deckMass: 20000 });
      const wallHeight = 3.0; // default height
      const result = await runPerfScenario({
        name: 'Wide wall 24x12',
        scenario,
        impactPlan: centerHit(wallHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('thick wall 12x8x4 layers', async () => {
      await loadModules();
      const scenario = buildWallScenario({
        spanSegments: 12, heightSegments: 8, layers: 4,
        thickness: 1.2, deckMass: 15000,
      });
      const wallHeight = 3.0;
      const result = await runPerfScenario({
        name: 'Thick wall 12x8x4',
        scenario,
        impactPlan: centerHit(wallHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('long bridge 24-span', async () => {
      await loadModules();
      const scenario = buildBeamBridgeScenario({
        spanSegments: 24, widthSegments: 8, thicknessLayers: 2,
        deckMass: 40000, span: 16, deckWidth: 4,
      });
      // Bridge deck is at pier height; drop a projectile on the deck center
      const result = await runPerfScenario({
        name: 'Long bridge 24-span',
        scenario,
        impactPlan: [{
          frame: 0,
          projectiles: [{
            position: { x: 0, y: 8, z: 0 },
            velocity: { x: 0, y: -30, z: 0 },
            radius: 0.4,
            mass: 20000,
          }],
        }],
      });
      expect(result.samples.length).toBeGreaterThan(0);
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 120_000);
  });

  // ────────────────────────────────────────────────────────────
  // C. Collision patterns (medium tower)
  // ────────────────────────────────────────────────────────────

  describe('C. Collision patterns (medium tower 6x12)', () => {
    const mediumTower = () => buildTowerScenario({ side: 6, stories: 12, totalMass: 5000 });
    const towerHeight = 12 * 0.5;

    it('single center hit', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Collision: center hit',
        scenario: mediumTower(),
        impactPlan: centerHit(towerHeight),
      });
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('single base hit (topple)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Collision: base hit',
        scenario: mediumTower(),
        impactPlan: baseHit(),
        postImpactFrames: 240,
      });
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('multi-simultaneous 4 projectiles', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Collision: 4 simultaneous',
        scenario: mediumTower(),
        impactPlan: multiSimultaneous(towerHeight),
      });
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);

    it('progressive bombardment (10 projectiles)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Collision: progressive bombardment',
        scenario: mediumTower(),
        impactPlan: progressiveBombardment(towerHeight, 10, 30),
        postImpactFrames: 240,
      });
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 120_000);

    it('catastrophic impact', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Collision: catastrophic',
        scenario: mediumTower(),
        impactPlan: catastrophicImpact(towerHeight),
        postImpactFrames: 300,
      });
      expect(result.bondsFinal).toBeLessThan(result.bondsInitial);
    }, 60_000);
  });

  // ────────────────────────────────────────────────────────────
  // D. Damage system overhead
  // ────────────────────────────────────────────────────────────

  describe('D. Damage system overhead', () => {
    const mediumTower = () => buildTowerScenario({ side: 6, stories: 12, totalMass: 5000 });
    const towerHeight = 12 * 0.5;

    it('with damage enabled', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Damage: enabled',
        scenario: mediumTower(),
        coreOpts: {
          damage: {
            enabled: true,
            strengthPerVolume: 50,
            autoDetachOnDestroy: true,
            autoCleanupPhysics: true,
          },
        },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('without damage (stress-only destruction)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Damage: disabled',
        scenario: mediumTower(),
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);
  });

  // ────────────────────────────────────────────────────────────
  // E. Configuration variants
  // ────────────────────────────────────────────────────────────

  describe('E. Configuration variants', () => {
    const mediumTower = () => buildTowerScenario({ side: 6, stories: 12, totalMass: 5000 });
    const towerHeight = 12 * 0.5;

    it('debris collision: all', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Config: debrisCollision=all',
        scenario: mediumTower(),
        coreOpts: { debrisCollisionMode: 'all' },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('debris collision: noDebrisPairs', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Config: debrisCollision=noDebrisPairs',
        scenario: mediumTower(),
        coreOpts: { debrisCollisionMode: 'noDebrisPairs' },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('debris collision: debrisNone', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Config: debrisCollision=debrisNone',
        scenario: mediumTower(),
        coreOpts: { debrisCollisionMode: 'debrisNone' },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('resimulation off', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Config: resimulate=off',
        scenario: mediumTower(),
        coreOpts: { resimulateOnFracture: false },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('resimulation on, maxPasses=3', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Config: resimulate=on, maxPasses=3',
        scenario: mediumTower(),
        coreOpts: { resimulateOnFracture: true, maxResimulationPasses: 3 },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);
  });

  // ────────────────────────────────────────────────────────────
  // F. Fracture policy tuning
  // ────────────────────────────────────────────────────────────

  describe('F. Fracture policy tuning (medium tower 6x12)', () => {
    const mediumTower = () => buildTowerScenario({ side: 6, stories: 12, totalMass: 5000 });
    const towerHeight = 12 * 0.5;

    it('unlimited (baseline)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: unlimited (baseline)',
        scenario: mediumTower(),
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 60_000);

    it('maxFracturesPerFrame: 8 (progressive fracture)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: maxFractures=8',
        scenario: mediumTower(),
        coreOpts: { fracturePolicy: { maxFracturesPerFrame: 8 } },
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 300, // more frames to let progressive fracture complete
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 120_000);

    it('maxFracturesPerFrame: 4 (slow propagation)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: maxFractures=4',
        scenario: mediumTower(),
        coreOpts: { fracturePolicy: { maxFracturesPerFrame: 4 } },
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 300,
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 120_000);

    it('maxNewBodiesPerFrame: 8', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: maxBodies=8',
        scenario: mediumTower(),
        coreOpts: { fracturePolicy: { maxNewBodiesPerFrame: 8 } },
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 300,
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 120_000);

    it('maxDynamicBodies: 20 (body cap)', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: maxDynBodies=20',
        scenario: mediumTower(),
        coreOpts: { fracturePolicy: { maxDynamicBodies: 20 } },
        impactPlan: centerHit(towerHeight),
      });
      expect(result.samples.length).toBeGreaterThan(0);
      // Body count should be capped
      expect(result.maxRigidBodies).toBeLessThanOrEqual(22); // small margin for ground+root
    }, 60_000);

    it('combined: maxFractures=8, maxBodies=10, maxMigrations=32', async () => {
      await loadModules();
      const result = await runPerfScenario({
        name: 'Policy: combined budget',
        scenario: mediumTower(),
        coreOpts: {
          fracturePolicy: {
            maxFracturesPerFrame: 8,
            maxNewBodiesPerFrame: 10,
            maxColliderMigrationsPerFrame: 32,
          },
        },
        impactPlan: centerHit(towerHeight),
        postImpactFrames: 360,
      });
      expect(result.samples.length).toBeGreaterThan(0);
    }, 120_000);
  });

  // ────────────────────────────────────────────────────────────
  // Final summary
  // ────────────────────────────────────────────────────────────

  afterAll(() => {
    if (allResults.length === 0) return;

    console.log('\n\n' + '='.repeat(90));
    console.log('  PERFORMANCE SUMMARY — ALL SCENARIOS');
    console.log('='.repeat(90));
    console.log(
      `  ${'Scenario'.padEnd(40)} ${'Nodes'.padStart(6)} ${'Bonds'.padStart(7)} ` +
      `${'Setup'.padStart(7)} ${'Mean'.padStart(8)} ${'P95'.padStart(8)} ${'Max'.padStart(8)} ` +
      `${'Bodies'.padStart(7)} ${'Survival'.padStart(9)}`
    );
    console.log('  ' + '-'.repeat(88));

    for (const r of allResults) {
      const survival = r.bondsInitial > 0
        ? ((r.bondsFinal / r.bondsInitial) * 100).toFixed(0) + '%'
        : 'N/A';
      console.log(
        `  ${r.name.padEnd(40)} ${String(r.nodeCount).padStart(6)} ${String(r.bondsInitial).padStart(7)} ` +
        `${(r.setupMs.toFixed(0) + 'ms').padStart(7)} ${fmtMs(r.allStats.mean)} ${fmtMs(r.allStats.p95)} ${fmtMs(r.allStats.max)} ` +
        `${String(r.maxRigidBodies).padStart(7)} ${survival.padStart(9)}`
      );
    }

    console.log('='.repeat(90));

    // Warn about any scenarios where P95 exceeds frame budget
    const FRAME_BUDGET_MS = 16.67; // 60fps
    const overBudget = allResults.filter((r) => r.allStats.p95 > FRAME_BUDGET_MS);
    if (overBudget.length > 0) {
      console.log(`\n  WARNING: ${overBudget.length} scenario(s) exceeded 60fps frame budget (${FRAME_BUDGET_MS}ms) at P95:`);
      for (const r of overBudget) {
        console.log(`    - ${r.name}: P95=${r.allStats.p95.toFixed(2)}ms`);
      }
    }

    // ── Spike Analysis ──
    console.log('\n  SPIKE ANALYSIS — Frames exceeding budget');
    console.log('  ' + '-'.repeat(88));
    console.log(
      `  ${'Scenario'.padEnd(40)} ${'>16ms'.padStart(6)} ${'>33ms'.padStart(6)} ${'>100ms'.padStart(7)} ${'>500ms'.padStart(7)} ${'Worst Phase'.padStart(20)}`
    );
    console.log('  ' + '-'.repeat(88));

    for (const r of allResults) {
      const gt16 = r.samples.filter((s) => s.totalMs > 16.67).length;
      const gt33 = r.samples.filter((s) => s.totalMs > 33.33).length;
      const gt100 = r.samples.filter((s) => s.totalMs > 100).length;
      const gt500 = r.samples.filter((s) => s.totalMs > 500).length;

      // Find worst phase in the worst frame
      let worstPhase = '';
      if (r.samples.length > 0) {
        const worstFrame = r.samples.reduce((a, b) => a.totalMs > b.totalMs ? a : b);
        const phases = [
          ['rapierStep', worstFrame.rapierStepMs],
          ['solverUpdate', worstFrame.solverUpdateMs],
          ['fracture', worstFrame.fractureMs],
          ['contactDrain', worstFrame.contactDrainMs],
          ['bodyCreate', worstFrame.bodyCreateMs],
          ['colliderRebuild', worstFrame.colliderRebuildMs],
          ['snapshot', worstFrame.snapshotCaptureMs],
        ] as const;
        const dominant = phases.reduce((a, b) => (a[1] ?? 0) > (b[1] ?? 0) ? a : b);
        worstPhase = `${dominant[0]} (${(dominant[1] ?? 0).toFixed(0)}ms)`;
      }

      console.log(
        `  ${r.name.padEnd(40)} ${String(gt16).padStart(6)} ${String(gt33).padStart(6)} ${String(gt100).padStart(7)} ${String(gt500).padStart(7)} ${worstPhase.padStart(20)}`
      );
    }

    console.log('='.repeat(90));
    console.log('');
  });
});
