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

An ovstage Stage is a columnar data store. Every committed write has an
**ordinal** — a monotonically increasing ordering number that the
**application owns and advances**. A *write floor* seals writes at or below an
ordinal. Sealing does not preserve historical versions of their payloads.

ovphysx never advances ordinals or writes to the Stage on its own. It exposes
exactly three ordinal-aware operations:

| ovphysx call | Direction | Ordinal role |
| --- | --- | --- |
| `ovphysx_attach_ovstage(handle, stage, read_ordinal)` | attach | `stage` is an `ovstage_instance_t*`; initial scene parse reads at the sealed `read_ordinal` (`ovstage_ordinal_t`, 1 through `OVSTAGE_ORDINAL_MAX`) |
| `ovphysx_update_from_ovstage(handle, range)` | **app → physics** | drains committed edits in the `ovstage_ordinal_range_t` `range` into the sim |
| read API (`ovphysx_query` / `ovphysx_read`) | **physics → app** | reads the latest step's output; the app writes it back at an ordinal *it* chooses |

Attach, update and detach return task receipts and may finish later. Calls on
one instance stay ordered, so later instance calls need no intermediate wait.
Wait before outside code uses the result. Keep the Stage and sealed data alive
until the task finishes, and keep an attached Stage alive until detach succeeds
or the instance is destroyed. See
[Stage lifecycle completion](developer_guide.md#stage-lifecycle-completion).

On a DirectGPU scene, `update_from_ovstage` drains the device-eligible value attributes
(`physics:velocity`, `physics:angularVelocity`) straight from device memory with no host
round trip, when the producer authored them device-resident over the drained bodies; other
attributes, partial writes, and mixed DirectGPU + CPU stages stage through the host. This is
transparent -- the producer authors the same columns either way -- and does not change the
sealing rules below: a structural change such as a prim deletion must still be sealed over its
structural columns (a `SCOPE_ALL` `advance_write_floor`), not only the value floor, or the drain
of that range is held until it is.

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
even procedural stages are checked. The raw Carbonite setting
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

`read_ordinal` establishes the initial read and consumed-ordinal cursor, not a
historical Stage snapshot. OVStage's latest-snapshot reads return the latest
committed recorded-column payloads. If ordinal 2 is already sealed, attaching
with `read_ordinal=1` can see newer positions and bodies added at ordinal 2;
a lower ordinal does not reconstruct earlier values or membership. If initial
values matter, seal the initial population and complete attach before authoring
later edits. When attaching after later edits, use the final sealed ordinal so
the consumed cursor covers the initial read.

OVStage ordinals are signed (`int64_t`): a Stage starts sealed through
`OVSTAGE_ORDINAL_MIN`, and zero and negative ordinals are writable. ovphysx
attaches only at a positive `read_ordinal` (0 is reserved), so seal the initial
population at 1 or later. Update ranges may start below 1: those ordinals
precede every attach ordinal and are skipped as consumed.

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

Scene-graph instancing has an additional constraint: ovstage's public
prototype/instance topology queries and resolved instance-material data expose
the latest committed state rather than an ordinal range. Finish population
before attach and keep instance topology stable for that attachment; historical
instance topology cannot be reconstructed from `read_ordinal`.

### Transform inputs

Transform updates are triggered by edits to the local `omni:xform` or
`omni:resetXformStack` (also `omni:fabric:localMatrix` for producers that use it).
A change to `omni:fabric:worldMatrix` alone does not trigger a physics transform
update, even when its ordinal is sealed and passed to `update_from_ovstage()`.
To move a body through the stage, author the local transform and update any
existing resolved `omni:fabric:worldMatrix` to match at the same control ordinal,
then seal and drain it. Population can seed that column even if the application
has never published it. Keep affected descendants' resolved world matrices
consistent too; a stale matrix can keep the body at its old pose.

The update trigger and the pose read are separate. At initial attachment and
when applying local edits, ovphysx can read an existing resolved world matrix
instead of composing the pose from local transforms.

The host transform-update path does not move rigid actors in DirectGPU scenes.
A separate DirectGPU device pose path can apply resolved world matrices for
supported standalone bodies, but teleports them rather than setting kinematic
targets. For explicit DirectGPU pose control, use session `position` and
`orientation` writes after the first step; see
[Writing Values](simulation_data/writing_values.md). Leave DirectGPU disabled
when driving kinematic supports that must carry other bodies, as described in
[Kinematic Support Geometry](simulation_setup/kinematic_support.md). See
[Known Limitations](read_write/limitations.md) for other runtime-edit restrictions.

### Deferred mass edits and session overrides

Completion of `update_from_ovstage()` does not imply that every edited property
is already reflected in live PhysX state. Rigid mass-frame edits
(`physics:mass`, `physics:density`, `physics:centerOfMass`,
`physics:diagonalInertia`, and `physics:principalAxes`) use the parse path to
recompute mass, inertia, and center of mass together at the next step. Covered
velocity edits can be visible immediately after the drain; that timing does not
apply to the mass frame.

A session write changes live PhysX state without authoring back to OVStage or
cancelling a pending recomputation. Draining stage mass 6, committing session
mass 9, and then stepping therefore leaves mass 6. To apply a session override
after the stage-derived mass frame is installed, use this order:

1. Finish authoring the OVStage edit, seal its ordinal, and wait for the seal.
2. Call `physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))`.
3. Complete `physx.step_sync(dt)` with the application's simulation timestep.
4. Open `physx.write_sync(...)`, fill the desired mass-frame values, and call
   `session.commit_sync(group)` for each group to override.

The step in item 3 advances simulation using the stage-derived mass frame.
If the desired mass must apply in that step, author it in OVStage before the
drain instead. Later stage edits that trigger another recomputation can replace
the live override again. See [session write ordering](read_write/writable.md#mixing-ovstage-mass-edits-and-session-writes)
and [drain limitations](read_write/limitations.md) for scope and device restrictions.

### Changing a prim's type at runtime

To retype a live prim, for example a `PhysicsFixedJoint` into a
`PhysicsRevoluteJoint`, write its new `usd-prim-type` at a new ordinal, seal it
and drain it like any other edit. ovphysx re-parses the prim and rebuilds its
PhysX object from the prim's columns.

Write the new type's columns in the same ordinal, at their resolved values: the
authored value, or the USD schema fallback when nothing is authored. That is
what ovstage population publishes for a prim authored with the new type; the
[population reference](population/index.md) lists it per type under
*Producer writes*. The old type's columns do not cover the new type: a fixed
joint has no `physics:lowerLimit` / `physics:upperLimit`, so a retype to
revolute publishes `-inf` / `inf` for an unlimited hinge. ovphysx reads an
absent column with its *Parser default*. For the joint limits that default is
the schema fallback, but the reference lists columns where the two differ, so
writing the full set is what keeps a retyped prim identical to a freshly
populated one.

The `ovphysx.population` builder writes such a set:
`resolved_defaults("PhysicsRevoluteJoint", ...)` in Python or
`PhysicsRevoluteJointDefaults::resolved(units)` in C++ gives the resolved
values, and `PrimBatch.define_physics_revolute_joint(...)` /
`definePhysicsRevoluteJoint(...)` followed by `write(stage, ordinal)` in Python
or `write(ordinal)` in C++ (the C++ batch takes its instance at construction)
upserts the prim. `write()` also rewrites `usd-schemas` from the APIs applied on the
batch, so apply the prim's existing APIs again. When the old and new types
have different ancestor types, write `usd-ancestor-types` as well; ovstage does
not derive it from `usd-prim-type`.

## Populating physics: the physics population description

OVStage populates physics data from USD only through a *population description*:
selectors of prims and properties, metadata paths and derived values, passed to
the `*_with_desc` entry points (Python: `ovstage.population.open_usd_with_desc()`,
`open_usd_from_string_with_desc()`, `repopulate_usd_with_desc()`; C:
`ovstage_population_open_usd_from_file_with_desc()` / `_from_string_with_desc()`,
`ovstage_population_repopulate_usd_with_desc()`). Its only built-in domain is
`RENDERING` (meshes, lights, materials, cameras).

ovphysx publishes the description of everything it reads:
`ovphysx_get_physics_population_desc()` in C (`ovphysx_population.h`) and
`ovphysx.population.physics_population_desc()` in Python. It selects colliders,
rigid bodies, joints, articulations and the other physics prims with their
physics schema properties, the geometry of the shapes, the stage units, and the
defaults ovphysx relies on where nothing is authored. The
[physics population description reference](population/physics-population-description.md)
lists it in full. Its own `domains` is `NONE`, so you choose what goes beside it:

| To populate | Python `descs=` | C `descs`, `desc_count` |
| --- | --- | --- |
| physics only | `physics_population_descs()` | `ovphysx_get_physics_population_desc()`, 1 |
| physics and render prims | `physics_population_descs(rendering=True)` | the physics description, then one whose `domains` is `OVSTAGE_POPULATION_DOMAIN_RENDERING`; 2 |

Stage units (`metersPerUnit`, `kilogramsPerUnit`, `upAxis`) are populated onto the
root prim `/` under the reserved `usd-metadata:` column prefix, for example
`usd-metadata:metersPerUnit`. The physics description requests them, so no extra
request is needed.

> **ovstage's `open_usd()` populates `RENDERING` only.** ovphysx finds no
> simulatable content on such a Stage, so always populate through the physics
> description.

### Compatibility path: physics with RENDERING

Populate the physics description with RENDERING when a renderer shares the
Stage or the scene uses point-instancer prototype geometry.

Native USD scene-graph instances need only the physics description. ovstage
writes each prototype's content once and publishes instancing topology;
ovphysx resolves the logical instance paths through the instancing API with
and without RENDERING. Hierarchy, prim type, applied schemas and local
transforms come from the prototype. Other attributes and relationships use
values published at the logical path first, including empty relationships
and arrays, then inherit the prototype's values. A producer writing a logical
instance path therefore changes what that instance simulates.

USD class definitions containing nested instances are currently unsupported: colliders may
also be created at the class-definition paths. Support for this case is deferred to a separate
issue.

Material bindings use the published winning purpose and instance-root binding
strength. An ancestor's physics-purpose binding outranks an all-purpose
fallback inside the prototype, and a `strongerThanDescendants` instance-root
binding overrides a descendant binding of the same purpose. Prototype-internal
material targets resolve to the corresponding material in each instance.

Both the Python and C examples below use `scene.usda` as a required
caller-supplied USD file, relative to the working directory. Replace each
`"scene.usda"` literal with the path to an existing USD scene. For the supplied
`simple_physics_scene.usda` and its package locations, see
[Quickstart prerequisites](tutorials/quickstart.md#prerequisites).

```python
import ovphysx
from ovphysx.population import physics_population_descs
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage

# Register PhysX and Newton schemas before the first USD schema read (population or export).
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("scene")
ovstage.population.open_usd_with_desc(
    stage,
    "scene.usda",
    ordinal=1,
    time=ovstage.population.DEFAULT_TIME,
    descs=physics_population_descs(rendering=True),
)
stage.advance_write_floor(ordinal=1).wait()
```

For the C example, copy the SDK's `samples/c_samples/common/` directory beside
the source file.

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_population.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <string.h>
#include "common/ovstage_sample.h"

static int populate_scene(ovstage_instance_t* stage)
{
    if (!ovphysx_sample_register_physx_schemas())
        return 3;

    /* The physics description, then RENDERING beside it. */
    ovstage_population_desc_t descs[2];
    memset(descs, 0, sizeof(descs));
    descs[0] = *ovphysx_get_physics_population_desc();
    descs[1].domains = OVSTAGE_POPULATION_DOMAIN_RENDERING;

    const ovx_string_t path = { .ptr = "scene.usda", .length = strlen("scene.usda") };
    const ovstage_population_enqueue_result_t pop = ovstage_population_open_usd_from_file_with_desc(
        stage, path, /*ordinal=*/1, /*time=*/0.0, descs, 2);
    if (pop.status != OVSTAGE_OK)
        return 1;
    if (pop.op_index.raw_id != 0 &&
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

A Stage shared with a render consumer such as ovrtx needs both for a second
reason: physics alone leaves the renderer with no geometry to draw, and
rendering alone leaves ovphysx with nothing to simulate.

### When the physics description alone is enough

The physics description alone supports native scene-graph instances when no
renderer shares the Stage and the scene uses no point-instancer prototypes.
Point-instancer prototype geometry requires RENDERING; without that geometry
the prototypes get no physics actors.

World transforms need nothing extra. Without `RENDERING`, population creates
`omni:fabric:worldMatrix` holding each prim's local matrix, and ovphysx trusts
that column when present. Sealing resolves it: an `advance_write_floor()` that
covers the column runs OVStage's default hierarchy computation, so the sealed
ordinal holds composed world matrices. A body under a transformed parent
attaches at its world pose, and output publication updates the column as with
`RENDERING`; the vehicle-wheel scale `ovphysx.utils.step_and_write_to_ovstage()`
derives from this column is composed the same way. Early OVStage 0.3 builds left
these values unresolved, so the ovphysx 0.7 guide had physics-only producers
delete the column before sealing. Remove that deletion: the sealed column
already holds composed world matrices, and vehicle-wheel output needs them.

### The descriptions are chosen at population time

`apply_usd_changes()` and the time-update entry points (Python:
`ovstage.population.update_from_usd_time()`; C:
`ovstage_population_apply_usd_time()`) take no descriptions — they honor the
ones the last `open_usd*_with_desc()` or `repopulate_usd_with_desc()` took and
re-evaluate only what those select. If a Stage may later be handed to a
renderer, or may use point-instancer prototype geometry, add RENDERING up front; otherwise
change the descriptions with `repopulate_usd_with_desc()`, which populates again
from the current USD source.

### Sequencing a shared Stage

Register the codeless PhysX schemas before the first USD schema read, including
population or export, and seal the population ordinal first, exactly as
[Seal population changes before attaching or draining](#seal-population-changes-before-attaching-or-draining)
describes, then let each
consumer observe it:

1. `open_usd_with_desc()` / `apply_usd_changes()`, and wait for the operation.
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

### Cost of RENDERING / shared population

A Stage populated with RENDERING carries render geometry as well as physics.
ovphysx scopes attach-time instance-root queries to prototypes that back
physics, so render-only prototypes do not dominate attach. The physics
description alone avoids rendering population for native scene-graph instances.

## The key principle: physics must not get its own changes

Both the application's *control* edits (poses, joint targets, gravity, …) and the
physics *output* (simulated poses, velocities, mesh points, …) live in the **same
ovstage Stage**. World-matrix-only output does not trigger a physics transform
update. Other output can use input attributes, such as point-instancer
`positions` and `orientations`; draining those edits can feed simulated poses
back into physics as authored changes.

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
only meaningful while an ovstage Stage is attached. Query, read, fetch, and
release return task receipts. Their handles are available on acceptance; wait
before using other outputs.

```
ovphysx_query(handle, type, scope, &query, &query_result) -> wait successfully
     query_result (optional, NULL skips it): creation-time type vocabulary / total-prim discovery
  -> ovphysx_read(handle, query, attrs, n, cuda_sync, &read, &read_result) -> wait successfully
     read_result (optional, NULL skips it): per-request group counts and omission reasons
       -> ovphysx_fetch_read_next(handle, read, &group_ptr) -> wait successfully
          null group_ptr means EOF; otherwise consume the borrowed group
          -> ovphysx_release_group(handle, read, group_ptr->read_group_id) -> wait
  -> ovphysx_release_read(handle, read) -> wait
ovphysx_release_query(handle, query) -> wait
```

Interned attribute tokens and prim lists resolve through the attached Stage's
own path dictionary, `ovstage_get_path_dictionary(stage)`. The read uses ovstage's
own types directly (no ovphysx mirror): a group is an `ovstage_read_group_t`
carrying one attribute column, `columns[0]` (`column_count == 1`),
discovery is an `ovstage_query_result_t`, and attribute names are
`ovx_token_or_string_t` (a string name OR an interned token from discovery). A
non-NULL `string.ptr` selects the string and ignores the token; otherwise the token
names the attribute, and `OVX_INVALID_TOKEN` is not a valid name. The
[Session results](read_write/index.md#session-results) section describes the
optional query and read results. The ovstage-native shape passes through
unflattened (the `data`, `attribute`, `is_array`, `semantic`, `ordinal` and `meta` fields
below are on the group's column, `columns[0]`):

- **`data.tensors` / `data.tensor_count`** — a borrowed `DLTensor` array: one stacked tensor for a
  fixed group, one tensor per prim for an `is_array` group. Group shape (fixed vs array) and how
  `dtype.lanes` carries tuple width are covered in
  [Data model and semantics](read_write/data_model.md).
- **`prims.list` / `prims.offset` / `prims.count` / `prims.index_map`** — the interned
  prim set (resolve through the attached Stage's path dictionary, or feed `prims.list` straight into
  `ovstage_query_from_path_list` for the write-back). An articulation-joint group is the exception:
  it has one row per DOF and its prim list repeats a joint's path once per axis, so regroup it per
  joint first ([joint groups](#c-utility-joint-groups-as-one-array-per-joint)). `attribute` is
  likewise an interned token.
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
struct and `prims.list` versus the column and its numeric tensors/maps/mask/event, and what a further
fetch or an `ovphysx_step` does to each — is documented in
[Device residency and interop](read_write/device.md).

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
arrays borrow the read session and are invalid once the `ReadResult` is closed; a result
dropped without a close stays alive while its arrays are. Drop every array and downstream
view before `PhysX.destroy()`. Stage dictionary values (`prim_list` and
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
        const ovx_token_or_string_t names[] = {
            { .token = OVX_INVALID_TOKEN,
              .string = { .ptr = OVPHYSX_ATTR_POSITION,
                          .length = sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
            { .token = OVX_INVALID_TOKEN,
              .string = { .ptr = OVPHYSX_ATTR_ORIENTATION,
                          .length = sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
        };
        if (!complete_task(h, ovphysx_read(h, q, names, 2, (ovstage_cuda_sync_t){0, 0}, &r, NULL)))
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
            // g->prims.list, g->columns[0].data.tensors, and g->columns[0].is_array with no repack
            // (a joint group is regrouped per joint first; see below).
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

Scale comes from physics, not from the stage: the utility reads
[`worldScale`](read_write/readable.md#rigid-bodies) in the same physics read as
`position` and `orientation`. That is the signed world scale ovphysx stored when
each body was created, so the stage needs no computed `omni:fabric:worldMatrix`
before the first call, and direct edits to the stage's world matrices do not
change the published scale. Neither does a runtime scale edit that physics does
not apply: character controllers, DirectGPU rigid bodies, articulation links
and kinematic bodies driven by time-sampled transforms keep their creation
scale, so each publication overwrites the matrix a hierarchy recomputation
derived from the edit. A character controller's matrix is published in its
capsule actor's frame, not the controller prim's
([Known limitations](read_write/limitations.md)). Seal each published output
before the next call. A different stage is rejected before the utility accesses
that stage or binds the cache.

The utility combines that scale with physics position and orientation using
ovphysx's [signed-scale convention](read_write/data_model.md#signed-scale-and-body-orientation)
and writes a row-vector float64 MATRIX. It
does not preserve shear. Character-controller orientation includes the capsule-axis
correction, so its matrix is in the controller prim's frame. On a DirectGPU scene the pose and scale columns are
device-resident, and matrices are composed and published without a host copy.
The utility then computes the hierarchy at `outputOrdinal` with the CPU model,
whatever the runtime default. The CPU model folds a direct world matrix into
the prim's local transform at its next computation, against the parent
transforms of that time. Folding it at the output ordinal keeps local and world
transforms consistent there, so a later parent edit moves a published body
with its parent. Without it, the first CPU computation after the edit would
hold the body at its published pose instead. The GPU models do not fold: they
rederive world matrices from the local transforms, so a GPU computation
without the fold would discard the published pose. `omni:resetXformStack` is
unchanged, and the matrices do not propagate to descendants.

Omit the optional cache argument to allocate output buffers on every call. An
application-owned `OvStageOutputCache` retains reusable CPU/CUDA matrix
buffers, one per published prim set and device, never borrowed stage views. It
binds to the instance, stage pointer and attachment on its first use. Call
`refresh()` after topology changes to release buffers for prim sets that are
no longer published; it keeps the binding. Create a new cache after
detach/reattach. Destroy the cache before the instance and stage, and
serialize calls with simulation and stage edits.

`OvStageOutputResult::ok()` reports success, including a scene with no selected
fixed poses. `matricesWritten` counts completed matrix rows. Failures return a
native ovphysx status and a descriptive `message`; missing pose or scale
columns and invalid pose or scale values are not replaced with defaults. A
later write failure can leave earlier writes completed at the unsealed output
ordinal, with their row count reported in the result. There is no rollback.

The installed `samples/c_samples/ovstage_output_cpp` sample demonstrates a
caller-owned cache and explicit step, publication and sealing:

```{literalinclude} ../tests/c_samples/ovstage_output_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C utility: joint groups as one array per joint

An articulation-joint group has one row per DOF: a joint's rows are consecutive and its path
repeats once per axis in `prims.list` ([Articulation joints](read_write/readable.md)).
ovstage stores a joint's values as one array per joint prim, so forwarding the group verbatim does
not fit. `ovphysx_create_joint_arrays_from_read_group` and `ovphysx_create_joint_arrays_from_map_group`,
declared in `<ovphysx/ovphysx_utils.h>`, regroup a joint read or write group without copying
anything: one tensor per joint, each a view of that joint's rows in the group's own memory, and an
owned path list naming each joint once. `ovphysx_get_joint_arrays_write_data` returns the views as
an ovstage array write payload (`tensors`, `tensor_count` joints), carrying a read group's semantic
and CUDA sync:

```c
// Waits for an enqueued ovstage operation and releases it; true when it succeeded.
static bool wait_op(ovstage_instance_t* stage, ovstage_enqueue_result_t task)
{
    if (task.status != OVSTAGE_OK)
        return false;
    if (task.op_index.raw_id == 0)
        return true; // nothing left to run
    ovstage_op_wait_result_t wait = { 0 };
    const bool ok = ovstage_wait_op(stage, task.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait) == OVSTAGE_OK &&
                    wait.error_op_id_count == 0;
    ovstage_release_op(stage, task.op_index);
    return ok;
}

// Publishes a fetched joint read group as one array per joint prim. The views alias the group, so it stays
// unreleased until the write has completed.
static bool publish_joints(ovstage_instance_t* stage, const ovstage_read_group_t* group,
                           ovx_token_or_string_t attribute, ovstage_ordinal_t output_ord)
{
    ovphysx_joint_arrays_t* joints = NULL;
    if (ovphysx_create_joint_arrays_from_read_group(stage, group, &joints).status != OVPHYSX_API_SUCCESS)
        return false; // ovphysx_get_last_error() says why
    bool ok = false;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(stage, ovphysx_get_joint_arrays_prim_list(joints), &query) == OVSTAGE_OK)
    {
        ok = wait_op(stage, ovstage_write_attribute(stage, query, attribute, output_ord,
                                                    ovphysx_get_joint_arrays_write_data(joints),
                                                    OVSTAGE_PRIM_MODE_UPSERT));
        ok = wait_op(stage, ovstage_release_query(stage, query)) && ok;
    }
    ovphysx_destroy_joint_arrays(joints); // after the query, which uses the joints' path list
    return ok;
}
```

The views alias the group, so they are valid only while it is: until the read group is released or
the write group is committed. Writing through a view of a write group fills its column. A group not
shaped as a joint group is refused with `OVPHYSX_API_INVALID_ARGUMENT`, and
`ovphysx_get_last_error` gives the reason. The Python counterpart is
`ovphysx.utils.split_joint_group(stage, group)`, which returns the path list (an `ovstage.PathList`
the caller releases) and the views, and which `step_and_write_to_ovstage` uses for joint output.

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

PhysX pose output contains world position and orientation; physics simulates no
scale. For fixed rigid bodies and articulation links, the utility also reads
[`worldScale`](read_write/readable.md#rigid-bodies), the signed world scale
ovphysx stored when each body was created, in the same physics read, so it reads
no world matrix from OVStage for them. A runtime scale edit that physics does not
apply therefore does not reach the published matrix either
([Known limitations](read_write/limitations.md)). Vehicle wheels have no stored scale: for
them the utility reads the current row-vector float64
`omni:fabric:worldMatrix`, derives its shear-free signed scale using the same
[matrix decomposition as ovphysx](read_write/data_model.md#signed-scale-and-body-orientation),
and releases every OVStage read view first. Either way it then writes the
reconstructed world matrix. It never changes `omni:resetXformStack`.
`worldScale` is used only to compose the matrix; it is written to
`sim:worldScale` only when `outputs` selects it.

After the writes, and before sealing `output_ordinal`, the helper computes the
hierarchy at that ordinal with the CPU model, as the C++ utility does. The
published prims' local transforms then match their new world matrices, so a
later parent edit moves them with their parent.

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

The default mode above reads the current vehicle-wheel world matrices and
point-instancer values on every call and retains no snapshots. Applications that prefer fewer OVStage
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
`output_cache.refresh()` after vehicle-wheel transform, point-instancer
pose-array, or topology changes; rigid-body and link scale comes from physics
on every call. It is bound to the exact OVStage attachment and cannot be
reused after a detach/reattach cycle. A lone `position` or `orientation`
selection is rejected.

For vehicle wheels, the sampled `omni:fabric:worldMatrix` must already contain
the current, sealed world transform. After changing authored local transforms or
hierarchy, compute the hierarchy and advance its output write floor before
calling this helper. Rigid bodies and articulation links need no stage world
matrix.

Every other emitted attribute is written to a shadow `sim:<name>` attribute.
Its `prim_list` forwards directly, fixed-group tensors pass through as-is, and
array-group tensors are lane-folded into OVStage's `dtype.lanes` vector shape.
A joint group is first split with `ovphysx.utils.split_joint_group`, so each
joint prim gets one array holding its DOFs in order.
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

- The read is ovstage-only. A query accepted without a usable attached Stage
  later fails its task. Its handle is still available immediately, so dependent
  calls may be submitted and will fail in order if the query fails.
  An empty match is still a valid, nonzero query whose read reaches null-group EOF
  (its optional query result then reports `total_prim_count == 0`).
- A whole-type query is a reusable selector (type + scope), not a captured membership
  snapshot: each read or write session evaluates its membership when it opens,
  observing every step submitted before it. A step between query and read is fine —
  the read reflects the newer step. The optional query result is a snapshot taken at
  query creation and never updates. A selected query keeps the membership it resolved
  at creation.
- `OVPHYSX_SCOPE_ACTIVE` selects a type-specific active set using active-actor reports where
  available and awake-state fallbacks otherwise; the [read/write overview](read_write/index.md)
  lists the per-type rules. The query can be reused across steps; open a new read session each step
  for the current set.

Refer also to the [Developer Guide](developer_guide.md) for the async/ordinal execution
model, and to the [API Reference](api.md) for the full C surface.
