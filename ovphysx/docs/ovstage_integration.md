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

Attach, update and detach return task receipts. Check admission, then wait for
the task's successful outcome before depending on it. The current execution is
inline; callers must still keep Stage storage and the sealed data being read
alive through completion. Keep an attached Stage alive until successful detach
or instance destruction. See [Stage lifecycle completion](developer_guide.md#stage-lifecycle-completion).

### Register the PhysX schemas before reading USD schemas

Applications install `physx-usd-schemas` separately and register its
two resource directories before the first USD schema read, including population
or export, in each process and worker. The Python sample's
`schema_support/` recipe and native `common/schema_support.h` validate the provider
without importing USD. Population drops attributes of schemas it cannot resolve,
including authored PhysX solver, joint and deformable settings.

By default, `attach_ovstage()` refuses submission if ovstage reports that its USD
schema registry was built without either required PhysX schema family. No task
is accepted and no attachment is taken. The refusal persists for the process;
restart and register before populating or exporting. A procedural stage attached
while USD schema definitions remain unbuilt skips the family check, leaving it
active for later attaches. Once population or export builds those definitions,
even procedural stages are checked. The legacy Carbonite setting
`/ovphysx/schemas/requireRegistration = false` downgrades the refusal to a warning
without restoring missing data. See
[Physics Schemas](physics_schemas.md#attach-time-registration-guard) for setup and
the guard's detection limits.

### Seal population changes before attaching or draining

`ovstage.population.apply_usd_changes()` blocks until population work finishes,
but it does **not** advance the Stage write floor. Seal and wait for the same
ordinal before passing it to ovphysx:

```python
import ovstage


def drain_population_change(stage, physx, ordinal):
    ovstage.population.apply_usd_changes(stage, ordinal=ordinal)
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))
```

The same rule applies to initial population: seal and wait for `read_ordinal`
before `attach_ovstage()`. That initial ordinal was then already parsed by the
attach. For a later edit at ordinal 2, drain `(2, 2)`, not `(1, 2)`: including
ordinal 1 is harmless because consumed ordinals are skipped, but naming only the
new range makes producer ownership explicit. A range containing only consumed
ordinals is a successful no-op; an overlapping range applies only its unread
suffix.

OVStage defaults to `NONDECREASING`: repeated mutations may use the same ordinal
while it remains unsealed. With an explicitly selected `STRICTLY_INCREASING`
admission policy, a second mutation
of the same prim/attribute must use a newer ordinal even if the first ordinal
has not been sealed. This also applies to prim deletion/recreation and to
customizing attributes copied by a clone. Writes to disjoint cells may share an
ordinal. Both policies reject writes at or below a sealed write floor.

For example, after consuming ordinal 1, delete a prim at ordinal 2 and recreate
it at ordinal 3; wait for both operations, seal ordinal 3, and call
`update_from_ovstage(2, 3)` once. Likewise, clone at ordinal 2 and customize copied
attributes at ordinal 3 before sealing. A fresh attach reads the final sealed
ordinal. The application coordinates these ordinals across physics and rendering;
ovphysx does not silently renumber caller-owned writes.

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

Both the Python and C examples below use `scene.usda` as a required
caller-supplied USD file, relative to the working directory. Replace each
`"scene.usda"` literal with the path to an existing USD scene. For the supplied
`simple_physics_scene.usda` and its package locations, see
[Quickstart prerequisites](tutorials/quickstart.md#prerequisites).

```python
import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage

# Register PhysX and Newton schemas before the first USD schema read (population or export).
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage,
    "scene.usda",
    ordinal=1,
    domains=ovstage.PopulationDomain.ALL,
)
stage.advance_write_floor(ordinal=1).wait()
```

For the C example, copy the SDK's `samples/c_samples/common/` directory beside
the source file.

```c
#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <string.h>
#include "common/ovstage_sample.h"

static int populate_scene(ovstage_instance_t* stage)
{
    if (!ovphysx_sample_register_physx_schemas())
        return 3;

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
native scene-graph instances. Resolve the transform seeds as described below,
and do not treat "ovphysx is the only consumer" as sufficient reason to skip
`RENDERING`.

OVStage 0.3 seeds `omni:fabric:worldMatrix` with each prim's local matrix
under `PHYSICS` alone. ovphysx trusts this column when present. The pinned
release's `compute_hierarchy()` leaves those physics-only seeds unchanged, so
calling it does not resolve a transformed parent. Prefer `ALL` for the
population examples above.

A producer that deliberately uses `PHYSICS` alone can omit the seeded world
column before sealing, allowing ovphysx to compose `omni:xform` up the parent
chain, including `omni:resetXformStack`:

```python
# After population at population_ordinal, before advance_write_floor().
cleanup_ordinal = population_ordinal + 1
with stage.query() as query:
    query.wait()
    stage.delete_attributes(query, ["omni:fabric:worldMatrix"], ordinal=cleanup_ordinal).wait()
stage.advance_write_floor(ordinal=cleanup_ordinal).wait()
# Attach at cleanup_ordinal, or drain the full population-through-cleanup range.
```

For C, use `ovstage_query()`, `ovstage_delete_attributes()` and
`ovstage_release_query()`; wait, check per-operation errors and release each
operation before sealing. This omission route is for attach and local-transform
consumption only; world-transform output publication requires a resolved
world-matrix column, so use `ALL` for workflows that publish physics output.
Apply this only to population's local seeds. A producer that writes resolved
world matrices directly owns those values and should preserve them.

### The mask is chosen once, at population time

`apply_usd_changes()` and the time-update entry points (Python:
`ovstage.population.update_from_usd_time()`; C:
`ovstage_population_apply_usd_time()`) take no `domains` argument — they honor
the mask the initial `open_usd*()` selected and re-evaluate only those domains.
No call adds a domain to an already-populated Stage. If a Stage may later be
handed to a renderer, or may contain native instances, populate `ALL` up front;
otherwise the only way to change the mask is to re-populate from USD.

### Sequencing a shared Stage

Register the codeless PhysX schemas before the first USD schema read, including
population or export, and seal the population ordinal first, exactly as
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
`OVPHYSX_QUERY_TARGET_RIGID_BODY` query created with a non-NULL `ovstage_query_result_t*`
succeeds and that result reports a nonzero `total_prim_count`. Diagnose
population failures and rejected ovphysx submissions from their return codes;
use `ovphysx_get_last_error()` for rejected ovphysx submissions and
`ovphysx_get_last_task_error()` for collected attach failures. A successful query can legitimately return zero
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

> **The read and write API — and its full data contract (attributes, physical meaning, units,
> frames, device residency, ownership, lanes, and roles) — is documented in the
> [Read/Write Data Contract](read_write/index.md) pages:** [readable](read_write/readable.md),
> [writable](read_write/writable.md), [data model](read_write/data_model.md), and
> [device residency](read_write/device.md). This section covers only how a read plugs into the
> ovstage Stage — the ovstage-native `ovstage_read_group_t` you forward back into ovstage, and the
> ordinal lifecycle — not the per-type contract.

The read API mirrors the ovstage read idiom — open a query over a simulated type,
read named attributes, iterate typed column groups, release. Native borrowed
columns can feed straight back into ovstage with no repack. It is ovstage-native:
only meaningful while an ovstage Stage is attached. Query, read, fetch and release
return task receipts; their current implementation executes inline. Keep inputs and
output slots alive until completion, and inspect outputs only after a successful wait.

```
ovphysx_query(handle, type, scope, &query, &query_result) -> wait successfully
     query_result (optional, NULL skips it): creation-time type vocabulary / total-prim discovery
  -> ovphysx_read(handle, query, attrs, n, &read, &read_result) -> wait successfully
     read_result (optional, NULL skips it): per-request group counts and omission reasons
       -> ovphysx_fetch_read_next(handle, read, &group_ptr) -> wait successfully
          null group_ptr means EOF; otherwise consume the borrowed group
          -> ovphysx_release_group(handle, read, group_ptr->read_group_id) -> wait
  -> ovphysx_release_read(handle, read) -> wait
ovphysx_release_query(handle, query) -> wait
```

Interned attribute tokens and prim lists resolve through the attached Stage's
own path dictionary, `ovstage_get_path_dictionary(stage)`. The read uses ovstage's
own types directly (no ovphysx mirror): a group is an `ovstage_read_group_t`,
discovery is an `ovstage_query_result_t`, and attribute names are
`ovx_string_or_token_t` (a string name OR an interned token from discovery). The
[Session results](read_write/index.md#session-results) section describes the
optional query and read results. The ovstage-native shape passes through
unflattened:

- **`data.tensors` / `data.tensor_count`** — a borrowed `DLTensor` array: one stacked tensor for a
  fixed group, one tensor per prim for an `is_array` group. Group shape (fixed vs array) and how
  `dtype.lanes` carries tuple width are covered in
  [Data model and semantics](read_write/data_model.md).
- **`prims.list` / `prims.offset` / `prims.count` / `prims.index_map`** — the interned
  prim set (resolve through the attached Stage's path dictionary, or feed `prims.list` straight into
  `ovstage_query_from_path_list` for the write-back). `attribute` is likewise an
  interned token.
- **`data.index_map` / `data.mask`** — sparsity over the **outer** logical element
  axis (ovstage's per-prim/per-tensor axis), never rows inside a single tensor.
  Point-instancer rigid-body output therefore always emits the instancer's FULL
  instance array (by-index, `index_map == NULL`) for both ALL and ACTIVE scope, so
  the group forwards into `ovstage_query_from_path_list` verbatim; ACTIVE scope only
  selects WHICH instancers are emitted, not a sparse subset of instances.
- **`data.cuda_sync`** — the ovstage-native stream/event handoff. Host-backed groups emit `{0, 0}`;
  device-resident (`kDLCUDA`) columns emit **`{0, event}`** — event only, no stream, because a
  non-zero `stream` tells ovstage to "drain everything queued on it," a heavier barrier than this
  handoff needs. Preserve the field verbatim when forwarding a group. *Which* columns are
  device-resident, how to wait on the event, and the per-column CPU/GPU split are covered in
  [Device residency and interop](read_write/device.md). `ALL` includes sleepers;
  `ACTIVE` follows the type-specific activity rules in the [overview](read_write/index.md).
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

Ownership & lifetime: the group is **producer-owned** — `ovphysx_fetch_read_next` hands back a
borrowed `const ovstage_read_group_t*` (you do not allocate it). The two-tier lifetime — the group
struct and `prims.list` versus the numeric tensors/maps/mask/event, and what a further fetch or an
`ovphysx_step` does to each — is documented in [Device residency and interop](read_write/device.md).

### Python Warp frontend

Python read groups expose one numeric type on both backends: every non-empty tensor
is a `warp.array` aliasing read-session storage on the read's native CPU or CUDA
device, and `index_map` / `prim_index_map` are CPU `warp.array` values of `uint32`
whatever the read's device. No host copy is implicit. A native `dtype.lanes` above 1
becomes a trailing Warp dimension, so a native vec3 column with `shape=[N]` is exposed
as `shape=(N, 3)`. An empty tensor is a Warp-owned empty array; an empty index map is
`None`.

```python
import warp as wp

from ovphysx.types import ObjectScope, SimObjectType

def print_read_shapes(physx):
    with physx.read_sync(
        SimObjectType.RIGID_BODY,
        ["position", "orientation"],
        scope=ObjectScope.ALL,
    ) as result:
        for group in result.groups:
            for tensor in group.tensors:
                assert isinstance(tensor, wp.array)
                print(tensor.device, tensor.shape)
```

`read_sync()` and `read_tokens_sync()` collect every native read task, including the
EOF fetch. The default `physx.read(query, attributes)` instead returns an `Operation`
for the session open, and each `result.fetch_next()` returns an `Operation` for one
group. A group's arrays are built on the first `wait()` or `result` access of its fetch,
and a CUDA group's nonzero producer event is ordered onto the Warp stream current at
that point. That stream wait does not block the host, but native task execution can;
establish a Warp stream dependency before using an array on another stream. Non-empty
arrays borrow the read session and may outlive the `ReadResult`, so drop every array
and downstream view before `PhysX.destroy()`. Stage dictionary values (`prim_list` and
`attribute`) stay context-bound; resolve them while the session is open, for example
through `result.dictionary`, an `ovstage.PathDictionary` for the attached Stage. Detach
retires this metadata even when borrowed numerical snapshots remain alive.

## Worked example: one closed-loop frame

The example drives one frame of the loop above with explicit ordinals, showing
the control lane (drained by physics) and the output lane (never drained).
The callbacks return `void`; they must finish reading borrowed inputs before
returning, including any GPU consumers, and handle their own failures. The output callback
writes each group without sealing its ordinal; the loop seals once after all groups. If
`run_closed_loop` returns nonzero, its caller destroys the instance; a rejected
final release can retain a caller-owned resource. Accepted releases are never
retried.

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
#include <ovstage/ovstage.h>            // application owns the ovstage Stage
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef void (*author_control_fn)(ovstage_instance_t*, ovstage_ordinal_t);
typedef void (*write_output_fn)(
    ovstage_instance_t*, ovstage_ordinal_t, const ovstage_read_group_t*);

static int complete_task(ovphysx_instance_handle_t physx, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS) {
        const ovphysx_string_t error = ovphysx_get_last_error();
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
        return 0;
    }
    ovphysx_wait_task_result_t waited = { 0 };
    const ovphysx_result_t result =
        ovphysx_wait_task(physx, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    for (size_t i = 0; i < waited.num_errors; ++i) {
        const ovphysx_string_t error = ovphysx_get_last_task_error(physx, waited.error_tasks[i]);
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
    }
    if (result.status != OVPHYSX_API_SUCCESS && waited.num_errors == 0) {
        const ovphysx_string_t error = ovphysx_get_last_error();
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
    }
    ovphysx_destroy_wait_task_result(&waited);
    return result.status == OVPHYSX_API_SUCCESS;
}

int run_closed_loop(
    ovphysx_instance_handle_t h,
    ovstage_instance_t* stage,
    int num_frames,
    author_control_fn author_control_edits,
    write_output_fn write_output_to_ovstage)
{
    // Attach: the scene was authored and sealed at ordinal 1.
    if (!complete_task(h, ovphysx_attach_ovstage(h, stage, /*read_ordinal=*/1)))
        return 1;

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
        if (!complete_task(h, ovphysx_update_from_ovstage(h, ctrl)))
            return 1;

        if (!complete_task(h, ovphysx_step(h, 1.0f / 60.0f)))
            return 1;

        // Physics->app: wait for each output before using it.
        int failed = 1;
        int read_open = 0;
        ovphysx_query_handle_t q = 0;
        ovphysx_read_handle_t r = 0;
        if (!complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_RIGID_BODY,
                                          OVPHYSX_SCOPE_ACTIVE, &q, NULL)))
            return 1;

        // Attribute selectors are names or interned tokens.
        const ovx_string_or_token_t names[] = {
            { .token = 0,
              .string = { .ptr = OVPHYSX_ATTR_POSITION,
                          .length = sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
            { .token = 0,
              .string = { .ptr = OVPHYSX_ATTR_ORIENTATION,
                          .length = sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
        };
        if (!complete_task(h, ovphysx_read(h, q, names, 2, &r, NULL)))
            goto cleanup;
        read_open = 1;

        for (;;)
        {
            const ovstage_read_group_t* g = NULL; // producer-owned borrowed pointer
            if (!complete_task(h, ovphysx_fetch_read_next(h, r, &g)))
                goto cleanup;
            if (!g)
                break;
            // App writes at `output_ord` without sealing it yet, reusing
            // g->prims.list, g->data.tensors, and g->is_array with no repack.
            write_output_to_ovstage(stage, output_ord, g);
            if (!complete_task(h, ovphysx_release_group(h, r, g->read_group_id)))
                goto cleanup;
        }
        {
            const ovstage_write_floor_desc_t floor_desc = {
                .ordinal = output_ord, .scope = OVSTAGE_SCOPE_ALL
            };
            const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floor_desc);
            if (floor.status != OVSTAGE_OK)
                goto cleanup;
            ovstage_op_wait_result_t waited = { 0 };
            const ovstage_api_status_t status =
                ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &waited);
            const bool succeeded = status == OVSTAGE_OK && waited.error_op_id_count == 0;
            const ovstage_api_status_t released = ovstage_release_op(stage, floor.op_index);
            if (!succeeded || released != OVSTAGE_OK)
                goto cleanup;
        }
        failed = 0;

cleanup:
        if (read_open && !complete_task(h, ovphysx_release_read(h, r)))
            failed = 1;
        if (!complete_task(h, ovphysx_release_query(h, q)))
            failed = 1;
        if (failed)
            return 1;

        // Advance the lanes. The output ordinal is skipped forever.
        control_ord += 2;
        output_ord  += 2;
    }
    return 0;
}
```

The invariant to hold onto: **every ordinal passed to
`ovphysx_update_from_ovstage` is a control ordinal the application authored;
output ordinals are never named in a drain range.** That single rule is
what keeps physics from consuming its own output.

### C++ utility: publish fixed world transforms

`ovphysx::utils::writeWorldTransformsToOvstage()` in
`<ovphysx/experimental/OvStageOutput.hpp>` publishes fixed rigid-body and
articulation-link poses to `omni:fabric:worldMatrix`. It supports CPU and CUDA
output. The C++17 application uses the existing ovphysx library; it needs no
CUDA or PhysX SDK headers or CUDA compiler to call the utility. Point-instancer
array groups are skipped and counted in
`OvStageOutputResult::instancerGroupsSkipped`. Vehicle wheels and other output
attributes are outside this utility's selection.

CUDA processing uses the primary context of each tensor's DLPack device
ordinal and requires its data to be addressable there. It uses CUDA directly,
without accessing the simulation's CUDA manager, and restores the calling
thread's prior context.

The application supplies the actual attached stage, completes its simulation
step, and chooses a fresh `outputOrdinal` above the sealed write floor. The
utility waits for its writes and returns their result; the application then
seals the output ordinal. It does not step, seal, drain, or update an
application transform journal. Never include its output ordinals in a range
passed to `ovphysx_update_from_ovstage()`.

Scale comes from the current, sealed `omni:fabric:worldMatrix`. After authoring
local transforms or hierarchy, compute the hierarchy and seal its output
before using the utility, including before a cache's first use. Seal each
published output before the next call, and complete any pending world-matrix
edits first. A different stage is rejected before the utility accesses that
stage or binds the cache.

The utility combines that scale with physics position and orientation using
ovphysx's [signed-scale convention](read_write/data_model.md#signed-scale-and-body-orientation)
and writes a row-vector float64 MATRIX. It
does not preserve shear. CUDA poses stay on the device while matrices are
composed and published. These direct world matrices leave `omni:xform` and
`omni:resetXformStack` unchanged and do not propagate to descendants. A later
hierarchy computation can overwrite them with matrices derived from the
authored local transforms.

Omit the optional cache argument to read current scale on every call. An
application-owned `OvStageOutputCache` retains scale copies and reusable
CPU/CUDA buffers, never borrowed stage views. It binds to the instance, stage
pointer and attachment on its first use. Call `refresh()` after authored
transform or topology changes; this discards cached data but keeps the
binding. Create a new cache after detach/reattach. Destroy the cache before
the instance and stage, and serialize calls with simulation and stage edits.
ovstage supplies CPU world-matrix reads for scale capture, including for
matrices authored on CUDA. Retaining the cache avoids rereading and
decomposing them on subsequent frames.

`OvStageOutputResult::ok()` reports success, including a scene with no selected
fixed poses. `matricesWritten` counts completed matrix rows. Failures return a
native ovphysx status and a descriptive `message`; missing or malformed source
matrices are not replaced with unit scale. A later write failure can leave
earlier writes completed at the unsealed output ordinal, with their row count
reported in the result. There is no rollback.

The installed `samples/c_samples/ovstage_output_cpp` sample demonstrates a
caller-owned cache and explicit step, publication and sealing:

```{literalinclude} ../tests/c_samples/ovstage_output_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### Python utility: one call for step + read + write-back

The opt-in `ovphysx.utils` helper composes `PhysX.step_sync`, `PhysX.read_sync`, and
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
It uses the `ovstage.Stage` object passed to `PhysX.attach_ovstage()`, which
rejects a raw native handle with `TypeError`.

PhysX pose output contains world position and orientation, but deliberately
contains no scale. For fixed rigid bodies, articulation links, and vehicle
wheels, the utility reads the current row-vector float64
`omni:fabric:worldMatrix`, derives its shear-free signed scale using the same
[matrix decomposition as ovphysx](read_write/data_model.md#signed-scale-and-body-orientation),
releases every OVStage read view, and only
then writes the reconstructed world matrix. It never writes `omni:xform` or
changes `omni:resetXformStack`. Scale remains OVStage state, not physics state.

`omni:fabric:worldMatrix` is a direct consumer-facing value. This helper does
not turn it into authoritative local transform state, schedule hierarchy
propagation, or update descendants. Do not run a later hierarchy computation
expecting this output ordinal to become the prim's new local transform.

Because a fixed pose write composes a world matrix from both halves, an
`outputs` selection that names `position` without `orientation` (or the
reverse) for a rigid body, articulation link, or vehicle wheel raises
`ValueError`. Select both attributes together, or neither -- there is no
partial-pose selection for these object types.

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
preferred application-owned cache. That sample loads `boxes_falling_on_groundplane.usda`,
which ships with every package under `ovphysx/samples/data/` in the wheel,
`<sdk-root>/samples/data/` in the C/C++ SDK, and `tests/data/` in a repository checkout.

```{literalinclude} ../tests/python_samples/output_read.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## Notes

- The read is ovstage-only: it requires an attached ovstage Stage, otherwise the
  query call is rejected without a task. Only inspect `out_query` after a successful wait.
  An empty match is still a valid, nonzero query whose read reaches null-group EOF
  (its optional query result then reports `total_prim_count == 0`).
- A whole-type query is a reusable selector (type + scope), not a captured membership
  snapshot: each read or write session evaluates its membership when it opens,
  observing the most recently completed step. A step between query and read is fine —
  the read reflects the newer step. The optional query result is a snapshot taken at
  query creation and never updates. A selected query keeps the membership it resolved
  at creation.
- `OVPHYSX_SCOPE_ACTIVE` selects a type-specific active set using active-actor reports where
  available and awake-state fallbacks otherwise; the [read/write overview](read_write/index.md)
  lists the per-type rules. The query can be reused across steps; open a new read session each step
  for the current set.

Refer also to the [Developer Guide](developer_guide.md) for the async/ordinal execution
model, and to the [API Reference](api.md) for the full C surface.
