# Ovstage Integration and Physics Output Read

ovphysx consumes a **caller-owned ovstage Stage** as its orchestration surface:
the application authors scene edits into ovstage, ovphysx drains committed edits
into the running simulation, steps, and the application reads the simulation
output back. This page explains how ordinals couple the two directions and — the
key principle — **how physics avoids consuming the changes it produced.**

For how to author the physics content in the stage (scenes, rigid bodies, joints,
articulations, deformables, particles), refer to the
[Simulation Setup](simulation_setup/physics_scene.md) pages.

## The data surface: ovstage + ordinals

An ovstage Stage is a versioned, columnar data store. Every committed write lands
at an **ordinal** — a monotonically increasing version number that the
**application owns and advances**. A *write floor* seals ordinals at or below a
value: sealed data never changes and is what reads observe.

ovphysx never advances ordinals or writes to the Stage on its own. It exposes
exactly three ordinal-aware operations:

| ovphysx call | Direction | Ordinal role |
| --- | --- | --- |
| `ovphysx_attach_ovstage(handle, stage, read_ordinal)` | attach | `stage` is an `ovstage_instance_t*`; initial scene parse reads at the sealed `read_ordinal` (`ovstage_ordinal_t`) |
| `ovphysx_update_from_ovstage(handle, range)` | **app → physics** | drains committed edits in the `ovstage_ordinal_range_t` `range` into the sim |
| read API (`ovphysx_query` / `ovphysx_read`) | **physics → app** | reads the latest step's output; the app writes it back at an ordinal *it* chooses |

### Seal population changes before attaching or draining

`ovstage.population.apply_usd_changes()` blocks until population work finishes,
but it does **not** advance the Stage write floor. Seal and wait for the same
ordinal before passing it to ovphysx:

```python
import ovstage


def drain_population_change(stage, physx, ordinal):
    ovstage.population.apply_usd_changes(stage, ordinal=ordinal)
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.update_from_ovstage(ordinal, ordinal)
```

The same rule applies to initial population: seal and wait for `read_ordinal`
before `attach_ovstage()`. That initial ordinal was then already parsed by the
attach. For a later edit at ordinal 2, drain `(2, 2)`, not `(1, 2)`: including
ordinal 1 replays the initial scene changes instead of applying only the new
delta.

Scene-graph instancing currently has one snapshot caveat: ovstage's public
prototype/instance topology queries and resolved instance-material data expose
the latest committed state rather than an ordinal range. Finish population
before attach and keep instance topology stable for that attachment; historical
instance topology cannot be reconstructed from `read_ordinal`.

## Population domains

Population is *domain-scoped*. `ovstage.population.open_usd()` (C:
`ovstage_population_open_usd_from_file` / `_from_string`) takes a **`domains`
bitmask** selecting which parts of the USD file land in the Stage. The values are
OR-combinable flags, not a choice of exactly one:

| Python (`ovstage.PopulationDomain`) | C (`ovstage_population_domain_t`) | Populates |
| --- | --- | --- |
| `NONE` (0) | `OVSTAGE_POPULATION_DOMAIN_NONE` | nothing beyond the stage units below |
| `RENDERING` | `OVSTAGE_POPULATION_DOMAIN_RENDERING` | meshes, lights, materials, cameras |
| `PHYSICS` | `OVSTAGE_POPULATION_DOMAIN_PHYSICS` | colliders, rigid bodies, joints, articulations, and the physics schema attributes authored on them |
| `ALL` (`RENDERING \| PHYSICS`) | `OVSTAGE_POPULATION_DOMAIN_ALL` | both |

Stage units (`metersPerUnit`, `kilogramsPerUnit`, `upAxis`) are authored onto the
`/__ovstage_population_stage_info__` prim regardless of the mask.

> **ovstage's Python default is `PopulationDomain.RENDERING` — physics off.**
> ovphysx finds no simulatable content on a Stage populated with the default, so
> always pass a mask that includes `PHYSICS`.

### Compatibility path: `ALL` (or `PHYSICS | RENDERING`)

For arbitrary USD content — including headless apps where ovphysx is the only
consumer — populate `ALL`. The physics populator on the currently pinned ovstage
uses default USD traversal, which visits neither instance proxies nor private
prototypes. A rigid body whose collider sits under a native USD scene-graph
instance can therefore be populated incompletely under `PHYSICS` alone: shape
bindings that appear under `ALL` are silently missing. Until the physics
populator grows instance-aware traversal, `ALL` (equivalently
`PHYSICS | RENDERING`) is the compatibility path for scenes that may use native
instancing.

```python
import ovstage

stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage,
    "scene.usda",
    ordinal=1,
    domains=ovstage.PopulationDomain.ALL,
)
stage.advance_write_floor(ordinal=1).wait()
```

```c
#include <string.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

ovstage_instance_t* stage = /* already created */;
ovx_string_t path = { .ptr = "scene.usda", .length = strlen("scene.usda") };

ovstage_population_enqueue_result_t pop = ovstage_population_open_usd_from_file(
    stage, path, /*ordinal=*/1, /*time=*/0.0,
    OVSTAGE_POPULATION_DOMAIN_ALL);
if (pop.status != OVSTAGE_OK)
    return /* handle enqueue failure */;
if (ovstage_population_wait_op(
        stage, pop.op_index, OVSTAGE_TIMEOUT_INFINITE, /*out=*/NULL) != OVSTAGE_OK)
    return /* handle wait failure */;
/* Then seal the ordinal (advance_write_floor) before attach -- see above. */
```

A Stage shared with a render consumer such as ovrtx needs the same union for a
second reason: physics-only leaves the renderer with no geometry to draw, and
rendering-only leaves ovphysx with nothing to simulate.

### When `PHYSICS` alone is enough

`PHYSICS` alone remains valid when the USD is known not to put physics under
native scene-graph instances — for example the ovphysx samples and skills, which
author plain (non-instanced) rigid bodies. Prefer it only with that knowledge;
do not treat "ovphysx is the only consumer" as sufficient reason to skip
`RENDERING`.

### The mask is chosen once, at population time

`apply_usd_changes()` and the time-update entry points (Python:
`ovstage.population.update_from_usd_time()`; C:
`ovstage_population_apply_usd_time()`) take no `domains` argument — they honor
the mask the initial `open_usd*()` selected and re-evaluate only those domains.
No call adds a domain to an already-populated Stage. If a Stage may later be
handed to a renderer, or may contain native instances, populate `ALL` up front;
otherwise the only way to change the mask is to re-populate from USD.

### Sequencing a shared Stage

Seal the population ordinal first, exactly as
[above](#seal-population-changes-before-attaching-or-draining), then let each
consumer observe it:

1. `open_usd()` / `apply_usd_changes()`, and wait for the operation.
2. `advance_write_floor(ordinal).wait()`.
3. Physics: `attach_ovstage(stage, read_ordinal=ordinal)` on first populate;
   `ovphysx_update_from_ovstage(handle, [ordinal, ordinal])` only for later
   structural edits (do not re-drain the attach ordinal).
4. Renderer (ovrtx): `ovrtx_attach_ovstage(renderer, stage)` once on first
   populate, then `ovrtx_update_from_stage()` before `ovrtx_step_with_stage()`.
   For a one-shot initial load the first step can rebuild implicitly; later
   committed structural edits use update followed by step.

The ordinal-lane rule in the next section is unchanged by a shared Stage:
physics still must never drain the ordinals where physics output was written,
whatever else reads them.

### Cost of `ALL` / shared population

A Stage populated for rendering as well as physics carries the whole scene
graph, and ovphysx's attach-time scene-graph instancing walk currently scales
with the **total** prototype count rather than with the physics-relevant one. On
a heavily instanced stage whose render-only prototypes greatly outnumber its
colliders, `ovphysx_attach_ovstage` can therefore cost orders of magnitude more
than the same fixture populated physics-only. The cost is paid once at attach,
so it appears as a startup stall rather than a per-step regression. That cost is
the reason the samples stay on `PHYSICS` when their content is known-safe; it is
not a reason to choose `PHYSICS` for arbitrary scenes.

## The key principle: physics must not get its own changes

Both the application's *control* edits (poses, joint targets, gravity, …) and the
physics *output* (simulated poses, velocities, mesh points, …) live in the **same
ovstage Stage**. If physics drained the ordinals where its own output was written,
it would re-ingest its last result as if it were a new authored change — corrupting
the simulation.

The rule that prevents this is simple and the application enforces it through the
ordinals it passes to `ovphysx_update_from_ovstage`:

> **App→physics edits flow through `ovphysx_update_from_ovstage`. Physics→app
> output is written at ordinals that `ovphysx_update_from_ovstage` never covers.**

Concretely, partition the ordinal line into two interleaved lanes:

- **Control ordinals** — where the *application* authors edits physics should
  process. These are the only ordinals ever passed to `ovphysx_update_from_ovstage`.
- **Output ordinals** — where the *application* writes the physics output it read
  back. These are **never** included in any `ovphysx_update_from_ovstage` range.

Because the two lanes never overlap and output ordinals are excluded from every
drain range, physics only ever ingests application-authored control edits — never
its own output.

### Ordinal timeline

A typical per-frame schedule, with the scene parsed at ordinal 1:

```
ordinal:   1        2        3        4        5        6      later
           |        |        |        |        |        |
         parse    CTRL     OUT      CTRL     OUT      CTRL
        (attach) (app)   (physics) (app)   (physics) (app)
                   |        ^        |        ^        |
                   |        |        |        |        |
 update_from_ovstage(2,2)   |  update_from_ovstage(4,4)   then repeat
                            |                 |
                  app writes physics    app writes physics
                  output here, NOT      output here, NOT
                  drained by physics    drained by physics
```

- Frame N: app authors control edits at an **even** ("CTRL") ordinal and seals it,
  passes the single-ordinal range `[ctrl, ctrl]` to
  `ovphysx_update_from_ovstage`, then calls `ovphysx_step`.
- App reads the output and writes it back at the next **odd** ("OUT") ordinal.
- Frame N+1: `ovphysx_update_from_ovstage` advances to the *next control ordinal*
  only — the intervening output ordinal is skipped, so physics never sees it.

> You choose the partitioning scheme (even/odd, fixed stride, two counters — any
> scheme works) as long as **output ordinals are never inside a drain range.**

## Reading simulation output

The read API mirrors the ovstage read idiom — open a query over a simulated type,
read named attributes, iterate typed column groups, release — so the columns feed
straight back into ovstage with no repack. It is ovstage-native: only meaningful
while an ovstage Stage is attached.

```
ovphysx_query(handle, type, scope, &query)
  -> ovphysx_fetch_query_result(handle, query, &result)   // attribute / total-prim discovery
  -> ovphysx_query_shared_dictionary(handle, query, &dict) // ovstage's shared dict: resolve interned tokens / prim lists
  -> ovphysx_read(handle, query, attrs, n, &read)         // attrs = ovx_string_or_token_t[] (name or token)
       -> loop ovphysx_fetch_read_next(handle, read, &group_ptr)  // group_ptr is producer-owned (borrowed)
              until it returns OVPHYSX_API_END_OF_ITERATION
              -> ovphysx_release_group(handle, read, group_ptr->read_group_id)
  -> ovphysx_release_read(handle, read)
ovphysx_release_query(handle, query)
```

The read uses ovstage's own types directly (no ovphysx mirror): a group is an
`ovstage_read_group_t`, discovery is an `ovstage_query_result_t`, and attribute
names are `ovx_string_or_token_t` (a string name OR an interned token from
discovery). The ovstage-native shape passes through unflattened:

- **`data.tensors` / `data.tensor_count`** — a borrowed `DLTensor` array. A **fixed**
  group has one tensor stacked over its prims (for example per-body world transforms); an
  **array** group (`is_array`) has one tensor per prim (for example a point-instancer's
  `positions` or a deformable mesh's `points`). Tuple width is carried in
  `tensors[i].dtype.lanes` (a vec3 column is lanes=3, shape=[N] — not a trailing
  shape dim).
- **`prims.list` / `prims.offset` / `prims.count` / `prims.index_map`** — the interned
  prim set (resolve through the query dictionary, or feed `prims.list` straight into
  `ovstage_query_from_path_list` for the write-back). `attribute` is likewise an
  interned token.
- **`data.index_map` / `data.mask`** — sparsity over the **outer** logical element
  axis (ovstage's per-prim/per-tensor axis), never rows inside a single tensor.
  Point-instancer rigid-body output therefore always emits the instancer's FULL
  instance array (by-index, `index_map == NULL`) for both ALL and ACTIVE scope, so
  the group forwards into `ovstage_query_from_path_list` verbatim; ACTIVE scope only
  selects WHICH instancers are emitted, not a sparse subset of instances.
- **`data.cuda_sync`** — producer stream/event synchronization. Output groups
  are CPU-backed and emit `{0, 0}`. With DirectGPU enabled, rigid-body and
  articulation-link pose and velocity columns are gathered synchronously from
  live GPU state, and volume and surface deformable points and velocities are
  copied from live GPU sim-mesh buffers and synchronized before emission; where
  sleeping is enabled, ALL refreshes sleepers while ACTIVE excludes them.
  Preserve the field when forwarding a group.
- **`is_array`** follows the source attribute kind (a ragged / USD-array /
  byte-string column), not whether the per-element dims happen to be uniform — a
  fixed-width array attribute is still `is_array`. When forwarding a group to
  `ovstage_write_attribute`, set `ovstage_write_data_t.is_array` to this value;
  tensor count and shape do not infer the attribute kind.
- **`semantic`**, **`ordinal`** (0 on this path), **`meta.attribute_write_floor_ordinal`**,
  and **`meta.layout_generation`**. The current output producer zero-initializes
  both metadata fields; rebuild cached queries and layouts explicitly after a
  known structural edit rather than using them as invalidation or sealing signals.
  `is_delete` is always false (physics output never emits tombstones). The queried
  object type is NOT on the group — the read is opened over one type, so the caller
  already knows it.

Ownership & lifetime: the group is **producer-owned** — `ovphysx_fetch_read_next`
hands back a borrowed `const ovstage_read_group_t*`, you do not allocate it. The
struct and every field it points at are valid until the group's `read_group_id` is
released through `ovphysx_release_group` (or `ovphysx_release_read`); fetching further
groups does NOT invalidate earlier ones, and an intervening `ovphysx_step` does NOT
either (columns are gathered into session-owned storage at read time).

### Output attributes by type

| `ovphysx_sim_object_type_t` | Attributes | Group shape |
| --- | --- | --- |
| `OVPHYSX_OBJECT_RIGID_BODY` | `position`, `orientation`, `linearVelocity`, `angularVelocity` | fixed (standalone) + one array group per point-instancer prim (instancer-local) |
| `OVPHYSX_OBJECT_ARTICULATION_LINK` | `position`, `orientation`, velocities | fixed |
| `OVPHYSX_OBJECT_ARTICULATION_JOINT` | `jointPosition`, `jointVelocity` | array per joint (one row per unlocked DOF axis) |
| `OVPHYSX_OBJECT_VEHICLE_WHEEL` | `position`, `orientation` | fixed (per wheel-root prim) |
| `OVPHYSX_OBJECT_DEFORMABLE_VOLUME` / `_SURFACE` | `points`, `velocities` | array per sim-mesh prim (mesh-local) |
| `OVPHYSX_OBJECT_PARTICLE_SET` | `points`, `velocities` | array per particle-set prim |

`OVPHYSX_SCOPE_ACTIVE` restricts the query to objects the solver moved last step
(single-frame — re-query each step); `OVPHYSX_SCOPE_ALL` returns every object of
the type.

For `OVPHYSX_OBJECT_ARTICULATION_JOINT`, the array carries one row per unlocked
reduced-coordinate DOF axis of the joint, in `PxArticulationAxis` enum order —
independent of whether a `JointStateAPI` is authored. Angular axes are reported in
degrees and linear (prismatic) axes in the stage's base length unit; an authored
`JointStateAPI` overrides the per-axis degree/radian convention where present.

## Worked example: one closed-loop frame

The example drives one frame of the loop above with explicit ordinals, showing
the control lane (drained by physics) and the output lane (never drained).

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>            // application owns the ovstage Stage
#include <stdint.h>

typedef void (*author_control_fn)(ovstage_instance_t*, ovstage_ordinal_t);
typedef void (*write_output_fn)(
    ovstage_instance_t*, ovstage_ordinal_t, const ovstage_read_group_t*);

void run_closed_loop(
    ovphysx_handle_t h,
    ovstage_instance_t* stage,
    int num_frames,
    author_control_fn author_control_edits,
    write_output_fn write_output_to_ovstage)
{
    // Attach: the scene was authored and sealed at ordinal 1.
    ovphysx_attach_ovstage(h, stage, /*read_ordinal=*/1);

    ovstage_ordinal_t control_ord = 2;  // even lane: app edits physics processes
    ovstage_ordinal_t output_ord  = 3;  // odd lane: physics output (NEVER drained)

    for (int frame = 0; frame < num_frames; ++frame)
    {
        // App authors control edits (e.g. a new kinematic target) at
        // `control_ord` and seals them through the ovstage write API.
        author_control_edits(stage, control_ord);

        // App->physics: drain ONLY the control ordinal (closed range). The
        // previous output ordinal is below it but is never named here.
        ovstage_ordinal_range_t ctrl = {
            control_ord, control_ord, /*has_start_ordinal=*/true
        };
        ovphysx_update_from_ovstage(h, ctrl);

        // Step the simulation.
        ovphysx_enqueue_result_t step_result = ovphysx_step(h, 1.0f / 60.0f);
        ovphysx_op_wait_result_t step_wait = { 0 };
        ovphysx_wait_op(h, step_result.op_index, UINT64_MAX, &step_wait);
        ovphysx_destroy_wait_result(&step_wait);

        // Physics->app: read the step output.
        ovphysx_query_handle_t q;
        ovphysx_query(h, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ACTIVE, &q);

        // Attribute selectors are names or interned tokens.
        const ovx_string_or_token_t names[] = {
            { .token = 0,
              .string = { .ptr = OVPHYSX_ATTR_POSITION,
                          .length = sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
            { .token = 0,
              .string = { .ptr = OVPHYSX_ATTR_ORIENTATION,
                          .length = sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
        };
        ovphysx_read_handle_t r;
        ovphysx_read(h, q, names, 2, &r);

        for (;;)
        {
            const ovstage_read_group_t* g = NULL; // producer-owned borrowed pointer
            const ovphysx_result_t fetch_result = ovphysx_fetch_read_next(h, r, &g);
            if (fetch_result.status == OVPHYSX_API_END_OF_ITERATION)
                break;
            if (fetch_result.status != OVPHYSX_API_SUCCESS || !g)
            {
                ovphysx_release_read(h, r);
                ovphysx_release_query(h, q);
                return;
            }
            // App writes the output at `output_ord` and seals it, reusing
            // g->prims.list, g->data.tensors, and g->is_array with no repack.
            write_output_to_ovstage(stage, output_ord, g);
            ovphysx_release_group(h, r, g->read_group_id);
        }
        ovphysx_release_read(h, r);
        ovphysx_release_query(h, q);

        // Advance the lanes. The output ordinal is skipped forever.
        control_ord += 2;
        output_ord  += 2;
    }
}
```

The invariant to hold onto: **every ordinal passed to
`ovphysx_update_from_ovstage` is a control ordinal the application authored;
output ordinals are never named in a drain range.** That single rule is
what keeps physics from consuming its own output.

## Notes

- The read is ovstage-only: it requires an attached ovstage Stage, else
  `ovphysx_query` fails. A zero `out_query` means FAILURE only — an empty match is
  still a valid, nonzero query whose read reaches end-of-iteration immediately
  (`ovphysx_fetch_query_result` then reports `total_prim_count == 0`).
- A query is a reusable selector (type + scope), not a captured membership snapshot:
  matched prims and values are evaluated lazily (count at `ovphysx_fetch_query_result`,
  columns at `ovphysx_read`), each observing the most recently completed step. A step
  between query and read is fine — the read reflects the newer step.
- Borrowed lifetime: the producer-owned group and its `tensors` / `index_map` /
  `prim_list` are valid until that group's `read_group_id` is released through
  `ovphysx_release_group` — fetching further groups (or stepping) does not invalidate
  earlier ones. Copy out anything you must retain past release (writing it back into
  ovstage within the window is the default path).
- `OVPHYSX_SCOPE_ACTIVE` is single-frame; re-open the query each step.

Refer also to the [Developer Guide](developer_guide.md) for the async/ordinal execution
model, and to the [API Reference](api.md) for the full C surface.
