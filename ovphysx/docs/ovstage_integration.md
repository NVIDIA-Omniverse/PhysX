<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovstage Integration and Physics Output Read

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
| `ovphysx_attach_ovstage(handle, stage, read_ordinal)` | attach | `stage` is an `ovstage_instance_t*`; initial scene parse reads at the sealed `read_ordinal` (`ovstage_ordinal_t`, must be non-zero) |
| `ovphysx_update_from_ovstage(handle, range)` | **app → physics** | drains committed edits in the `ovstage_ordinal_range_t` `range` into the sim |
| read API (`ovphysx_query` / `ovphysx_read`) | **physics → app** | reads the latest step's output; the app writes it back at an ordinal *it* chooses |

### Register the PhysX schemas before populating

ovphysx ships its PhysX USD schemas as codeless plugins; the application
registers them. Before the first population call in the process (`open_usd`,
`apply_usd_changes`, or an export), register them with ovstage: in Python,
`ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])`;
in C, pass the root from `ovphysx_get_codeless_schema_root()` to
`ovstage_population_register_usd_schemas()`. USD assembles its schema registry
once, so a late registration cannot be repaired: population silently drops every
Physx* API it cannot resolve, and the scene would simulate without the asset's
self-collision, joint-limit and solver settings. `attach_ovstage` therefore
verifies the registration and fails with an error naming the missing call when
population ran without it. Refer to
[Physics Schemas](physics_schemas.md#making-schemas-available).

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
ordinal 1 is harmless because consumed ordinals are skipped, but naming only the
new range makes producer ownership explicit. A range containing only consumed
ordinals is a successful no-op; an overlapping range applies only its unread
suffix.

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
| `NONE` (0) | `OVSTAGE_POPULATION_DOMAIN_NONE` | nothing |
| `RENDERING` | `OVSTAGE_POPULATION_DOMAIN_RENDERING` | meshes, lights, materials, cameras |
| `PHYSICS` | `OVSTAGE_POPULATION_DOMAIN_PHYSICS` | colliders, rigid bodies, joints, articulations, the physics schema attributes authored on them, and the stage units below |
| `ALL` (`RENDERING \| PHYSICS`) | `OVSTAGE_POPULATION_DOMAIN_ALL` | both |

Stage units (`metersPerUnit`, `kilogramsPerUnit`, `upAxis`) are populated onto the
root prim `/` under the reserved `usd-metadata:` column prefix, for example
`usd-metadata:metersPerUnit`. They are brought in by the `PHYSICS` domain (and so
by `ALL`); a `NONE`-only populate authors no units. A bare `PHYSICS`/`ALL`
`open_usd` therefore already carries the units — no extra request is needed.

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
import ovphysx
import ovstage

# Register the codeless PhysX schemas before the first population call.
ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
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
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <string.h>

static int populate_scene(ovstage_instance_t* stage)
{
    /* Register the codeless PhysX schemas before the first population call. */
    ovphysx_string_t schema_root;
    if (ovphysx_get_codeless_schema_root(&schema_root).status != OVPHYSX_API_SUCCESS)
        return 3;
    const ovx_string_t schema_path = { .ptr = schema_root.ptr, .length = schema_root.length };
    if (ovstage_population_register_usd_schemas(&schema_path, 1) != OVSTAGE_OK)
        return 4;

    const ovx_string_t path = { .ptr = "scene.usda", .length = strlen("scene.usda") };
    const ovstage_population_enqueue_result_t pop = ovstage_population_open_usd_from_file(
        stage, path, /*ordinal=*/1, /*time=*/0.0,
        OVSTAGE_POPULATION_DOMAIN_ALL);
    if (pop.status != OVSTAGE_OK)
        return 1;
    if (pop.op_index != OVSTAGE_POPULATION_INVALID_OP_ID &&
        ovstage_population_wait_op(
            stage, pop.op_index, OVSTAGE_TIMEOUT_INFINITE, /*out=*/NULL) != OVSTAGE_OK)
        return 2;

    /* Do not attach yet. The caller must seal ordinal 1 as described in
       "Seal population changes before attaching or draining". */
    return 0;
}
```

After `populate_scene()` succeeds, seal ordinal 1 with
`ovstage_advance_write_floor()` before attaching it to ovphysx, exactly as in
[Seal population changes before attaching or draining](#seal-population-changes-before-attaching-or-draining).

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

Register the codeless PhysX schemas before the first population call and seal
the population ordinal first, exactly as
[Seal population changes before attaching or draining](#seal-population-changes-before-attaching-or-draining)
describes, then let each
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

For a scene known to contain rigid bodies, the sequence is complete when an
`OVPHYSX_OBJECT_RIGID_BODY` query succeeds and
`ovphysx_fetch_query_result()` reports a nonzero `total_prim_count`. Diagnose
population and attach failures from their return codes and
`ovphysx_get_last_error()`. A successful query can legitimately return zero
objects when the requested type is absent from the scene.

The ordinal-lane rule in
[The key principle: physics must not get its own changes](#the-key-principle-physics-must-not-get-its-own-changes)
is unchanged by a shared Stage: physics still must never drain the ordinals
where physics output was written, whatever else reads them.

### Cost of `ALL` / shared population

A Stage populated with `ALL` carries render geometry as well as physics. ovphysx
now scopes attach-time instance-root queries to prototypes that back collision
shapes in common leaf-collider scenes, so render-only prototypes no longer
dominate attach the way they once did. Preferring `PHYSICS` for known-safe
non-instanced sample content is still reasonable when no renderer shares the
Stage; it is not a reason to choose `PHYSICS` for arbitrary scenes, where
native-instance colliders can be omitted.

## The key principle: physics must not get its own changes

Both the application's *control* edits (poses, joint targets, gravity, …) and the
physics *output* (simulated poses, velocities, mesh points, …) live in the **same
ovstage Stage**. If physics drained the ordinals where its own output was written,
it would re-ingest its last result as if it were a new authored change — corrupting
the simulation.

One rule prevents this, and the application enforces it through the ordinals it
passes to `ovphysx_update_from_ovstage`:

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
read named attributes, iterate typed column groups, release. Native borrowed
columns can feed straight back into ovstage with no repack. It is ovstage-native:
only meaningful while an ovstage Stage is attached.

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
- **`data.cuda_sync`** — producer stream/event synchronization. Host-backed groups
  emit `{0, 0}`. With DirectGPU enabled, rigid-body, articulation-link,
  whole-articulation root/COM, and joint-DOF
  columns are **device-resident** (`kDLCUDA`) and emit **`{0, event}`** — event only,
  no stream, because a non-zero `stream` means "drain everything queued on it" to
  ovstage, which is a heavier barrier than this handoff needs. **Such a column is
  handed over before its producer work has necessarily completed:** wait on the event
  before reading it on your own stream (consuming on the default/null stream is safe,
  since the producer work is ordered there). `ovphysx_cuda_stream_wait_event(stream,
  event)` is that wait, so honouring the native contract costs no CUDA dependency
  of your own. The Python frontend instead orders the event onto Warp's current
  stream before returning its `warp.array`. Volume and surface deformable `points`
  and `velocities` are **device columns on a DirectGPU scene** — sourced from the live
  GPU sim-mesh buffers and delivered without a copy to the host, carrying the same
  `{0, event}` handoff as every other device column. Their `restPoints` and element
  indices are authored topology and stay host-resident, so one deformable group
  routinely mixes residencies; branch on each tensor's device rather than on the read.
  See [Device residency and interop](read_write/device.md) for the per-column split. Where sleeping
  is enabled, ALL refreshes sleepers while ACTIVE excludes them. Preserve the field when forwarding a
  group.
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
struct and stage-derived `prims.list` are valid until the group's `read_group_id`
is released through `ovphysx_release_group`. Numeric tensors, maps, masks, and the
CUDA completion event remain valid until `ovphysx_release_read`; fetching further
groups and an intervening `ovphysx_step` invalidate neither lifetime.

### Python Warp frontend

`PhysX.read()` and `PhysX.read_tokens()` expose one numeric type on both backends:
every non-empty tensor is a `warp.array` aliasing read-session storage on the read's
native CPU or CUDA device, and `index_map` / `prim_index_map` are CPU `warp.array`
values of `uint32` whatever the read's device. No host copy is implicit. A native
`dtype.lanes` above 1 becomes a trailing Warp dimension, so a native vec3 column with
`shape=[N]` is exposed as `shape=(N, 3)`. An empty tensor is a Warp-owned empty array;
an empty index map is `None`.

```python
import warp as wp

from ovphysx.types import ObjectScope, SimObjectType

def print_read_shapes(physx):
    with physx.read(
        SimObjectType.RIGID_BODY,
        ["position", "orientation"],
        ObjectScope.ALL,
    ) as result:
        for group in result.groups:
            for tensor in group.tensors:
                assert isinstance(tensor, wp.array)
                print(tensor.device, tensor.shape)
```

A CUDA read orders each nonzero producer event onto the Warp stream current during
`read()` before returning, without blocking the host; establish a Warp stream
dependency before using an array on another stream. Non-empty arrays borrow the read
session and may outlive the `ReadResult`, so drop every array and downstream view
before `PhysX.destroy()`. Stage dictionary values (`prim_list`, `attribute`, and
`result.dictionary`) stay context-bound.

The per-type inventory of what each object type serves — attribute lists, physical meaning,
units, frames, device residency, and group shape — lives in the
[Read/Write Data Contract](read_write/index.md) section:
[Readable data](read_write/readable.md), [Writable data](read_write/writable.md),
[Data model and semantics](read_write/data_model.md), and
[Device residency and interop](read_write/device.md). This page covers only how those groups
plug into the ovstage Stage and its ordinal lifecycle.

## Worked example: one closed-loop frame

The example drives one frame of the loop above with explicit ordinals, showing
the control lane (drained by physics) and the output lane (never drained).

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>            // application owns the ovstage Stage
#include <stdbool.h>
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
        ovphysx_wait_op(
            h, step_result.op_index, OVPHYSX_TIMEOUT_INFINITE, &step_wait);
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

### Python utility: one call for step + read + write-back

The opt-in `ovphysx.utils` helper composes `PhysX.step_sync`, `PhysX.read`, and
the per-group write-back above without adding that application workflow to the
core `PhysX` API:

```python
from ovphysx.utils import step_and_write_to_ovstage


def publish_frame(physx, output_ordinal):
    return step_and_write_to_ovstage(
        physx,
        dt=1.0 / 60.0,
        output_ordinal=output_ordinal,
    )
```

The utility imports the `ovstage` Python package only when called.
It requires `PhysX.attach_ovstage()` to have received the Python
`ovstage.Stage` object, not only its raw native handle.

PhysX pose output contains world position and orientation, but deliberately
contains no scale. For fixed rigid bodies, articulation links, and vehicle
wheels, the utility reads the current row-vector float64
`omni:fabric:worldMatrix`, derives its shear-free signed scale using the same
matrix decomposition as ovphysx, releases every OVStage read view, and only
then writes the reconstructed world matrix. It never writes `omni:xform` or
changes `omni:resetXformStack`. Scale remains OVStage state, not physics state.

`omni:fabric:worldMatrix` is a direct consumer-facing value. This helper does
not turn it into authoritative local transform state, schedule hierarchy
propagation, or update descendants. Do not run a later hierarchy computation
expecting this output ordinal to become the prim's new local transform.

Rigid-body point instancers use a different representation. The output read
already converts simulated poses back into each instancer's local frame. The
utility writes those values to the native `positions` (`float32`, POINT) and
`orientations` (`float16`, QUATERNION) arrays. It preserves unsimulated holes
and authored trailing rows from the current arrays; it does not touch `scales`,
prototype indices, or other instancer arrays.

The default mode above reads the current transform and point-instancer values
on every call and retains no snapshots. Applications that prefer fewer OVStage
reads and reusable CPU/CUDA buffers can explicitly own a copy cache:

```python
from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage


def publish_frames(physx, output_ordinals):
    with OvStageOutputCache(physx) as output_cache:
        for output_ordinal in output_ordinals:
            step_and_write_to_ovstage(
                physx,
                dt=1.0 / 60.0,
                output_ordinal=output_ordinal,
                cache=output_cache,
            )
```

The cache owns copies, never persistent borrowed OVStage views. Call
`output_cache.refresh()` after transform, point-instancer pose-array, or
topology changes. It is bound to the exact OVStage attachment and cannot be
reused after a detach/reattach cycle. A lone `position` or `orientation`
selection is rejected.

The sampled `omni:fabric:worldMatrix` must already contain the current, sealed
world transform. After changing authored local transforms or hierarchy, compute
the hierarchy and advance its output write floor before calling this helper.

Every other emitted attribute is written to a shadow `sim:<name>` attribute.
Its `prim_list` forwards directly, fixed-group tensors pass through as-is, and
array-group tensors are lane-folded into OVStage's `dtype.lanes` vector shape.
Sparse index and CUDA synchronization metadata forward with unchanged data.
Pose matrices and point-instancer array merges run in Warp on the relevant CPU
or CUDA devices. The default path borrows OVStage DLPack views until the output
buffers are ready, then releases them before writing. The normal full-group
path does not clone source arrays. An optional cache owns source copies and
reuses output buffers across calls. Cached CUDA writes receive an event recorded
after the work; uncached calls finish work that uses a borrowed view before
releasing it. No path copies poses through the host. The utility then calls
`Stage.advance_write_floor(ordinal=output_ordinal)` once. `outputs` narrows the
`{SimObjectType: [attribute, ...]}` read/write set; omit it for the default
dynamic output set. `output_ordinal` must stay the never-drained lane -- never a
value passed to `update_from_ovstage`. See
`tests/python_samples/output_read.py` for the full closed loop using the
preferred application-owned cache.

## Notes

- The read is ovstage-only: it requires an attached ovstage Stage, else
  `ovphysx_query` fails. A zero `out_query` means FAILURE only — an empty match is
  still a valid, nonzero query whose read reaches end-of-iteration immediately
  (`ovphysx_fetch_query_result` then reports `total_prim_count == 0`).
- A query is a reusable selector (type + scope), not a captured membership snapshot:
  matched prims and values are evaluated lazily (count at `ovphysx_fetch_query_result`,
  columns at `ovphysx_read`), each observing the most recently completed step. A step
  between query and read is fine — the read reflects the newer step.
- Borrowed lifetime: the producer-owned group struct and its `prim_list` are valid
  until that group's `read_group_id` is released through `ovphysx_release_group`.
  Numeric tensors, maps, masks, and the CUDA event remain valid until
  `ovphysx_release_read`. Fetching further groups or stepping invalidates neither;
  copy anything that must outlive its corresponding lifetime.
- `OVPHYSX_SCOPE_ACTIVE` is single-frame; re-open the query each step.

Refer also to the [Developer Guide](developer_guide.md) for the async/ordinal execution
model, and to the [API Reference](api.md) for the full C surface.
