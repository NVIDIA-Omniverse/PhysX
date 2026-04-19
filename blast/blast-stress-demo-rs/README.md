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

## Additional Scene Packs

The demo also ships embedded authored scene packs for:

- `fractured-wall`
- `fractured-tower`
- `fractured-bridge`
- `brick-building`

The brick building preset is a coarse 2-story shell with staggered brick
coursing, interlocking corners, window and door openings, a flat roof, and a
2-course parapet. Its bond graph is generated from the authored brick geometry
rather than hand-authored connectivity.

## Run Modes

Normal bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge cargo run
```

Normal bridge with meshes:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_SHOW_MESHES=1 cargo run
```

Brick building with meshes:

```bash
BLAST_STRESS_DEMO_SCENARIO=brick-building BLAST_STRESS_DEMO_SHOW_MESHES=1 cargo run
```

Headless bridge:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=300 cargo run
```

Headless bridge with automated scripted shots:

```bash
BLAST_STRESS_DEMO_SCENARIO=bridge BLAST_STRESS_DEMO_HEADLESS=1 BLAST_STRESS_DEMO_HEADLESS_FRAMES=220 BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT=bridge_benchmark cargo run
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
- `[summary]` for one end-of-run machine-readable totals/peaks line
- `[shot-fired]` for exact scripted projectile launches in headless mode
- `[fracture-frame]` for exact per-frame fracture/split/edit events
- `[heavy-frame]` for any frame that crosses the configured frame/physics/Rapier threshold
- `[topology-frame]` for frames where body topology changed materially
- `[post-split-frame]` for the first Rapier step(s) where sibling-grace filtering is active

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
- `split_child_pose_ms`
- `split_velocity_fit_ms`
- `split_sleep_init_ms`

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
- `split_cohorts`
- `split_cohort_bodies`
- `sibling_grace_pairs`
- `sibling_grace_filtered_pairs`

### Automated Headless Shot Scripts

The demo can auto-fire reproducible headless shot scripts against the real runtime:

- `wall_smoke`
- `tower_smoke`
- `bridge_smoke`
- `building_smoke`
- `wall_benchmark`
- `tower_benchmark`
- `bridge_benchmark`
- `building_benchmark`
- `auto_smoke`
- `auto_benchmark`

`auto_*` selects the scenario-matching script from `BLAST_STRESS_DEMO_SCENARIO`.

Each scripted shot logs a `[shot-fired]` line with:

- `frame`
- `script`
- `label`
- `mass`
- `speed`
- `origin`
- `target`
- `direction`

Headless runs now end with a `[summary]` line containing:

- total frames
- script name
- shots planned / fired
- max frame / physics / Rapier / solver timings and frame indices
- max split-plan / split-apply / split-move timings and frame indices
- peak body/contact counts and frame indices
- total fractures / splits / new bodies / moved colliders
- first fracture frame and worst fracture-frame timings

This is the machine-readable line the automated integration tests consume.

## Current Runtime Design Notes

- Surviving colliders are preserved across normal splits and moved with Rapier parent reassignment instead of being recreated.
- Child bodies are recentered to child COM / centroid and inherit fitted linear/angular velocity from the pre-split source velocity field.
- Same-split sibling bodies can use a one-step sibling grace filter so the first post-split Rapier step can be measured with reduced sibling-only contact shock.
- Split-edit timings are logged separately from the post-edit Rapier step so scene-edit cost and simulation cost can be distinguished.
- Bridge defaults currently favor realistic free fracture rather than artificial fracture caps.
- `DebrisCollisionMode` remains configurable for experiments, but default behavior should be judged primarily on fracture quality plus exact fracture-frame timings.

## Useful Environment Variables

- `BLAST_STRESS_DEMO_SCENARIO`
- `BLAST_STRESS_DEMO_SHOW_MESHES`
- `BLAST_STRESS_DEMO_HEADLESS`
- `BLAST_STRESS_DEMO_HEADLESS_FRAMES`
- `BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT`
- `BLAST_STRESS_DEMO_RAPIER_ONLY`
- `BLAST_STRESS_DEMO_BODY_CCD`
- `BLAST_STRESS_DEMO_DEBRIS_COLLISION_MODE`
- `BLAST_STRESS_DEMO_SIBLING_GRACE_STEPS` (default `0`; opt in to enable the temporary same-split sibling collision grace)
- `BLAST_STRESS_DEMO_PROJECTILE_FRACTURE_GRACE_STEPS` (temporary projectile-vs-fresh-fracture filter during resim replay)
- `BLAST_STRESS_DEMO_PROJECTILE_TRACE` (emits `[projectile-frame]` lines for exact projectile position/velocity/progress debugging)
- `BLAST_STRESS_DEMO_PROJECTILE_MAX_MASS`
- `BLAST_STRESS_DEMO_HEAVY_FRAME_MS`
- `BLAST_STRESS_DEMO_TOPOLOGY_BODY_DELTA`

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

## Automated Tests

Run the automated headless reproduction suite:

```bash
cargo test --test headless_shot_scripts -- --nocapture
```

That test launches the real binary headless, uses scripted shots for wall/tower/bridge, and parses the emitted `[summary]` and `[shot-fired]` lines from the generated perf log.

The suite now includes a direct wall-face regression:

- without resim, a heavy face shot must shatter the wall
- with resim enabled, the same heavy shot must shatter the wall and pass through it
