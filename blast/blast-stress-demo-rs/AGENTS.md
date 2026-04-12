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
- `[fracture-frame]` log lines are the source of truth for exact fracture spikes.

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
4. If needed, run the same scenario with `BLAST_STRESS_DEMO_RAPIER_ONLY=1` to isolate Rapier stepping cost from fracture logic.

Useful grep:

```bash
rg "^\[fracture-frame\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log
```

## Current Runtime Design Notes

- Normal splits should preserve collider handles and reparent them instead of recreating colliders.
- If logs show `inserted_colliders` or `removed_colliders` rising during normal fracture flow, treat that as a regression.
- The key question is whether a spike comes from:
  - split planning/edit work
  - or the post-edit Rapier step on the resulting contact graph

## Files To Know

- `src/main.rs`:
  - scenario presets
  - env parsing
  - perf logging
  - Rapier-only mode
  - headless execution

Most runtime/perf investigation starts there.
