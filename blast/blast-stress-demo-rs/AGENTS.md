# AGENTS.md

## Scope

This crate is the Rust Bevy + Rapier performance harness for Blast destruction work.

Prefer documenting and debugging runtime behavior here rather than only in the monorepo root docs.

## Primary Goals

- preserve realistic fracture behavior
- measure exact fracture-time hitches
- minimize Rapier scene-edit cost
- separate split-application cost from post-fracture Rapier stepping cost

## Important Current Assumptions

- The default bridge preset is intentionally coarse and should stay in the roughly `100-200` chunk range unless there is a specific reason to benchmark denser authoring.
- `BLAST_STRESS_DEMO_RAPIER_ONLY=1` is the standalone Rapier benchmark mode. Use it to measure raw Rapier cost without the stress solver or fracture pipeline.
- `[perf]` log lines are rolling medians and are not enough to diagnose single-frame hitching.
- `[summary]` is the machine-readable end-of-run line for automated headless measurement.
- `[shot-fired]` lines are the source of truth for scripted headless projectile reproduction.
- `[fracture-frame]` log lines are the source of truth for exact fracture spikes.
- `[heavy-frame]`, `[topology-frame]`, and `[post-split-frame]` are the fast path for understanding whether a bad frame came from general load, topology churn, or immediate post-split contact shock.

## Commands

Normal bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge cargo run
```

Bridge with meshes:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_SHOW_MESHES=1 cargo run
```

Headless bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=300 cargo run
```

Headless bridge benchmark with scripted shots:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=220 BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT=bridge_benchmark cargo run
```

Rapier-only bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_RAPIER_ONLY=1 cargo run
```

Headless Rapier-only bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_RAPIER_ONLY=1 BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=300 cargo run
```

## Perf Debug Workflow

When investigating hitching:

1. Run the normal bridge and capture a perf log.
2. Grep `^\[fracture-frame\]` first.
3. Compare `split_plan_ms`, `split_apply_ms`, and `split_collider_move_ms` against `rapier_ms`.
4. Then check `^\[post-split-frame\]` and `^\[heavy-frame\]` to see whether sibling-grace filtered pairs reduced the first post-split Rapier spike.
5. If needed, run the same scenario with `BLAST_STRESS_DEMO_RAPIER_ONLY=1` to isolate Rapier stepping cost from fracture logic.
6. For reproducible headless benchmarking, prefer a named shot script and parse `[summary]`.

Useful grep:

```bash
rg "^\[fracture-frame\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log
```

```bash
rg "^\[(post-split-frame|heavy-frame|topology-frame)\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log
```

```bash
rg "^\[(shot-fired|summary)\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log
```

## Current Runtime Design Notes

- Normal splits should preserve collider handles and reparent them instead of recreating colliders.
- Child bodies are recentered and get fitted linear/angular velocities so the post-split handoff to Rapier is closer to the “already separate” baseline.
- `BLAST_STRESS_DEMO_SIBLING_GRACE_STEPS` controls the temporary same-split sibling collision filter used to diagnose and reduce first-step contact shock. Default is `0`; opt in to enable it.
- `BLAST_STRESS_DEMO_PROJECTILE_FRACTURE_GRACE_STEPS` controls the temporary projectile-vs-fresh-fracture filter used during resim replay.
- `BLAST_STRESS_DEMO_PROJECTILE_TRACE=1` emits `[projectile-frame]` lines so you can inspect exact projectile position, velocity, progress, and live contact counts frame-by-frame.
- `BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT` enables reproducible scripted projectile launches in headless mode. Supported scripts are `wall_smoke`, `tower_smoke`, `bridge_smoke`, `wall_benchmark`, `tower_benchmark`, `bridge_benchmark`, plus `auto_smoke` and `auto_benchmark`.
- If logs show `inserted_colliders` or `removed_colliders` rising during normal fracture flow, treat that as a regression.
- The key question is whether a spike comes from:
  - split planning/edit work
  - or the post-edit Rapier step on the resulting contact graph

## Automated Reproduction

The integration test suite for this crate is:

```bash
cargo test --test headless_shot_scripts -- --nocapture
```

It launches the real demo binary headless, fires scripted shots with varying masses/trajectories, and parses `[summary]` from the generated temp log.

It also covers the wall resimulation expectation explicitly:

- heavy wall-face shot without resim must shatter the wall
- heavy wall-face shot with resim must shatter the wall and pass through it

## Files To Know

- `src/main.rs`:
  - scenario presets
  - env parsing
  - perf logging
  - Rapier-only mode
  - headless execution

Most runtime/perf investigation starts there.
