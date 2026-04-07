#!/usr/bin/env node
/**
 * Deterministic destruction benchmark.
 *
 * Runs a fixed scenario (medium tower, single projectile impact) headlessly
 * through the full destruction pipeline and prints per-phase timing as JSON.
 *
 * Usage:
 *   node scripts/bench.mjs            # default medium tower
 *   node scripts/bench.mjs --large    # large tower (8x20)
 *   node scripts/bench.mjs --small    # small tower (3x4)
 *
 * Output (JSON to stdout):
 * {
 *   "scenario": "medium-tower-6x12",
 *   "nodes": 468,
 *   "bonds": 2652,
 *   "totalFrames": 360,
 *   "setupMs": 42,
 *   "warmupMeanMs": 0.8,
 *   "impactMeanMs": 16.2,
 *   "impactP95Ms": 20.1,
 *   "impactMaxMs": 440,
 *   "postImpactMeanMs": 11.0,
 *   "overallMeanMs": 12.5,
 *   "overallP95Ms": 19.8,
 *   "overallMaxMs": 440,
 *   "spikeCount_16ms": 120,
 *   "spikeCount_33ms": 3,
 *   "spikeCount_100ms": 1,
 *   "bondsFinal": 2400,
 *   "bondSurvivalPct": 90.5,
 *   "maxRigidBodies": 45,
 *   "phases": {
 *     "rapierStepMs":      { "mean": 5.1, "p95": 8.2, "max": 12.0 },
 *     "solverUpdateMs":    { "mean": 5.5, "p95": 7.0, "max": 65.0 },
 *     "contactDrainMs":    { "mean": 1.2, "p95": 2.1, "max": 5.0 },
 *     "fractureMs":        { "mean": 0.5, "p95": 0.01, "max": 300.0 },
 *     "bodyCreateMs":      { "mean": 0.1, "p95": 0.01, "max": 8.0 },
 *     "colliderRebuildMs": { "mean": 0.1, "p95": 0.01, "max": 5.0 },
 *     "snapshotCaptureMs": { "mean": 1.0, "p95": 2.0, "max": 4.0 },
 *     "preStepSweepMs":    { "mean": 0.02, "p95": 0.05, "max": 0.1 }
 *   }
 * }
 */

import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { existsSync } from 'node:fs';

const here = dirname(fileURLToPath(import.meta.url));
const distDir = resolve(here, '../dist');

// Check WASM exists
if (!existsSync(resolve(distDir, 'stress_solver.wasm'))) {
  console.error('ERROR: dist/stress_solver.wasm not found. Run: npm run build');
  process.exit(1);
}

// Parse args
const args = process.argv.slice(2);
const sizeFlag = args.find(a => ['--small', '--medium', '--large'].includes(a)) || '--medium';

// Load modules (use ESM builds — CJS has WASM URL resolution issues in Node)
const { buildDestructibleCore } = await import(resolve(distDir, 'rapier.js'));
const { buildTowerScenario } = await import(resolve(distDir, 'scenarios.js'));

// Scenario configs (deterministic — no randomness)
const CONFIGS = {
  '--small':  { side: 3, stories: 4,  totalMass: 1000,  name: 'small-tower-3x4' },
  '--medium': { side: 6, stories: 12, totalMass: 5000,  name: 'medium-tower-6x12' },
  '--large':  { side: 8, stories: 20, totalMass: 10000, name: 'large-tower-8x20' },
};

const cfg = CONFIGS[sizeFlag];
const WARMUP_FRAMES = 60;
const IMPACT_FRAMES = 120;
const POST_FRAMES = 180;
const DT = 1 / 60;

// Fixed projectile — always hits center of tower at same angle
const towerHeight = cfg.stories * 0.5;
const PROJECTILE = {
  position: { x: 0, y: towerHeight * 0.5, z: 5 },
  velocity: { x: 0, y: 0, z: -40 },
  radius: 0.35,
  mass: 15000,
  ttl: 5000,
};

// Collect profiler samples
const samples = [];
const profilerConfig = {
  enabled: true,
  onSample: (s) => samples.push(s),
};

// Suppress debug logs from core (they go to stdout and pollute JSON output)
const origDebug = console.debug;
const origWarn = console.warn;
console.debug = () => {};
console.warn = () => {};

// Build scenario + core
const scenario = buildTowerScenario(cfg);
const setupStart = performance.now();
const core = await buildDestructibleCore({
  scenario,
  gravity: -9.81,
  materialScale: 1e8,
  resimulateOnFracture: true,
  maxResimulationPasses: 1,
  snapshotMode: 'perBody',
});
const setupMs = performance.now() - setupStart;

// Restore console
console.debug = origDebug;
console.warn = origWarn;

core.setProfiler(profilerConfig);
const bondsInitial = core.getActiveBondsCount();

// Phase 1: Warmup (gravity only)
for (let i = 0; i < WARMUP_FRAMES; i++) core.step(DT);
const warmupEnd = samples.length;

// Phase 2: Impact
core.enqueueProjectile(PROJECTILE);
for (let i = 0; i < IMPACT_FRAMES; i++) core.step(DT);
const impactEnd = samples.length;

// Phase 3: Post-impact settling
for (let i = 0; i < POST_FRAMES; i++) core.step(DT);

const bondsFinal = core.getActiveBondsCount();
const maxRigidBodies = Math.max(...samples.map(s => s.rigidBodies || 0));
core.dispose();

// Stats helpers
function stats(values) {
  if (!values.length) return { mean: 0, p95: 0, max: 0 };
  const sorted = [...values].sort((a, b) => a - b);
  const mean = sorted.reduce((a, b) => a + b, 0) / sorted.length;
  const p95 = sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95))];
  const max = sorted[sorted.length - 1];
  return { mean: +mean.toFixed(2), p95: +p95.toFixed(2), max: +max.toFixed(2) };
}

const warmupSamples = samples.slice(0, warmupEnd);
const impactSamples = samples.slice(warmupEnd, impactEnd);
const postSamples = samples.slice(impactEnd);
const allTotals = samples.map(s => s.totalMs);

const phaseKeys = [
  'rapierStepMs', 'solverUpdateMs', 'contactDrainMs', 'fractureMs',
  'bodyCreateMs', 'colliderRebuildMs', 'snapshotCaptureMs', 'preStepSweepMs',
  'damageTickMs', 'initialPassMs', 'resimMs',
];

const phases = {};
for (const key of phaseKeys) {
  const vals = samples.map(s => s[key] || 0);
  if (Math.max(...vals) > 0.001) {
    phases[key] = stats(vals);
  }
}

const overallStats = stats(allTotals);

const result = {
  scenario: cfg.name,
  nodes: scenario.nodes.length,
  bonds: bondsInitial,
  totalFrames: samples.length,
  setupMs: +setupMs.toFixed(1),
  warmupMeanMs: +stats(warmupSamples.map(s => s.totalMs)).mean,
  impactMeanMs: +stats(impactSamples.map(s => s.totalMs)).mean,
  impactP95Ms: +stats(impactSamples.map(s => s.totalMs)).p95,
  impactMaxMs: +stats(impactSamples.map(s => s.totalMs)).max,
  postImpactMeanMs: +stats(postSamples.map(s => s.totalMs)).mean,
  overallMeanMs: overallStats.mean,
  overallP95Ms: overallStats.p95,
  overallMaxMs: overallStats.max,
  spikeCount_16ms: allTotals.filter(t => t > 16.67).length,
  spikeCount_33ms: allTotals.filter(t => t > 33.33).length,
  spikeCount_100ms: allTotals.filter(t => t > 100).length,
  bondsFinal,
  bondSurvivalPct: +((bondsFinal / bondsInitial) * 100).toFixed(1),
  maxRigidBodies,
  phases,
};

// Print JSON to stdout — this is what the optimizer reads
console.log(JSON.stringify(result, null, 2));
