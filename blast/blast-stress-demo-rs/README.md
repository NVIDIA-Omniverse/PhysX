# Blast Stress Demo RS

Rust Bevy + Rapier demo for profiling Blast stress/fracture runtime behavior.

## What This Demo Is For

This repo is the runtime/perf harness for:

- measuring Rapier-only baseline cost
- measuring full stress-solver + split-application cost
- logging exact fracture-frame spikes instead of only coarse medians

The current default bridge preset intentionally uses a much coarser chunk graph than the earlier dense bridge. The bridge silhouette is preserved, but the destructible graph is kept in the roughly `100-200` body range instead of the previous `~900`.

## Bridge Preset

Default bridge authoring is intentionally reduced:

- `span_segments = 18`
- `width_segments = 6`
- `thickness_layers = 1`
- `supports_per_side = 3`
- `support_width_segments = 1`
- `support_depth_segments = 1`

This keeps the bridge shape while making runtime fracture and Rapier stepping tractable.

## Run Modes

Normal bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge cargo run
```

Normal bridge with meshes:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_SHOW_MESHES=1 cargo run
```

Headless bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=300 cargo run
```

Rapier-only standalone benchmark:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_RAPIER_ONLY=1 cargo run
```

Headless Rapier-only benchmark:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_RAPIER_ONLY=1 BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=300 cargo run
```

`BLAST_STRESS_DEMO_RAPIER_ONLY=1` disables the stress solver, fracture processing, resimulation, and runtime optimization passes. It creates one Rapier rigid body per scenario chunk so you can measure Rapier stepping cost on its own.

## Performance Logs

Perf logs are written to the system temp directory and contain two different line types:

- `[perf]` for rolling 1-second median windows
- `[fracture-frame]` for exact per-frame fracture/split/edit events

Use `[perf]` to understand steady-state behavior. Use `[fracture-frame]` to diagnose hitching, jitter, or freezing during actual break events.

### Find Exact Fracture Frames

```bash
rg "^\[fracture-frame\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log
```

### Find The Worst Fracture Frames

```bash
rg "^\[fracture-frame\]" /var/folders/t_/xzvrtyx933q778_gtnt_qkp80000gn/T/blast-stress-demo-perf-*.log | sort -t'=' -k3,3nr
```

Then inspect the fields:

- `frame_ms`
- `physics_ms`
- `rapier_ms`
- `solver_ms`
- `split_sanitize_ms`
- `split_estimate_ms`
- `split_plan_ms`
- `split_apply_ms`
- `split_body_create_ms`
- `split_collider_move_ms`
- `split_collider_insert_ms`
- `split_body_retire_ms`

Important counters:

- `fractures`
- `splits`
- `reused_bodies`
- `new_bodies`
- `moved_colliders`
- `inserted_colliders`
- `removed_colliders`
- `dynamic_bodies`
- `awake_dynamic_bodies`
- `active_contact_pairs`
- `contact_manifolds`

## Current Runtime Design Notes

- Surviving colliders are preserved across normal splits and moved with Rapier parent reassignment instead of being recreated.
- Split-edit timings are logged separately from the post-edit Rapier step so scene-edit cost and simulation cost can be distinguished.
- Bridge defaults currently favor realistic free fracture rather than artificial fracture caps.
- `DebrisCollisionMode` remains configurable for experiments, but default behavior should be judged primarily on fracture quality plus exact fracture-frame timings.

## Useful Environment Variables

- `BLAST_STRESS_DEMO_SCENARIO`
- `BLAST_STRESS_DEMO_SHOW_MESHES`
- `BLAST_STRESS_DEMO_HEADLESS`
- `BLAST_STRESS_DEMO_HEADLESS_FRAMES`
- `BLAST_STRESS_DEMO_RAPIER_ONLY`
- `BLAST_STRESS_DEMO_BODY_CCD`
- `BLAST_STRESS_DEMO_DEBRIS_COLLISION_MODE`

Fracture policy / tuning envs are also supported and logged in the perf header:

- `BLAST_STRESS_DEMO_MAX_FRACTURES_PER_FRAME`
- `BLAST_STRESS_DEMO_MAX_NEW_BODIES_PER_FRAME`
- `BLAST_STRESS_DEMO_MAX_COLLIDER_MIGRATIONS_PER_FRAME`
- `BLAST_STRESS_DEMO_MAX_DYNAMIC_BODIES`
- `BLAST_STRESS_DEMO_MIN_CHILD_NODE_COUNT`
- `BLAST_STRESS_DEMO_IDLE_SKIP`
- `BLAST_STRESS_DEMO_APPLY_EXCESS_FORCES`

## What To Look At First

If the demo feels bad:

1. Check `[fracture-frame]` lines first.
2. Separate `split_*_ms` from `rapier_ms`.
3. Check whether the spike is scene-edit work or the post-fracture contact graph.
4. Compare normal mode against `BLAST_STRESS_DEMO_RAPIER_ONLY=1` for the same scenario shape.
