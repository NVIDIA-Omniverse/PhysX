<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Overview

This section is the authoritative reference describing how to exchange data with ovphysx through
the **read and write API**, that is, `ovphysx_read` / `ovphysx_write` for the C API and `PhysX.read`
/ `PhysX.write` for the Python API. It documents, for every simulation object type, the attributes
that can be read or written along with any necessary information, e.g., physical meaning, frame, units,
data type and layout, device residency, and when a value is valid.

## The session model

A read or a write is always three phases: **select** objects with a *query*, open a **session** over
that query for one or more attributes, then **iterate** the session's groups.

Native query, session-open, fetch, commit and release calls return an
`ovphysx_enqueue_result_t`. Check admission, then wait on the complete receipt to
collect execution success or failure before using an output handle, result, or group.
A successful fetch with a null group means iteration is complete. These operations
currently execute inline; the task contract does not promise a nonblocking host call.

In Python, `PhysX.query`, `PhysX.read`, `PhysX.write`, `fetch_next`, `commit` and `close`
return an `Operation` without waiting. `operation.wait()` collects its tasks and returns the
typed result; `PhysX.wait_all()` collects every outstanding task, including releases submitted by
`close()` or a `with` block exit. `read_sync`, `read_tokens_sync`, `write_sync`, `commit_sync` and
`close_sync` are the explicit waiting conveniences.


A typical control step reads an attribute and writes another back: for example, read each
articulation joint's `jointPosition`, then set every `jointPositionTarget` to zero.
The application must already have attached a stage. On DirectGPU, complete the first step before
calling the Python example. Serialize simulation and structural edits while either example runs.

**C (CPU scene)**

This example uses a single query for both sessions. It returns nonzero on failure, releases all
opened handles, and discards an uncommitted group. The helper reports admission
errors immediately and task errors while their wait result still owns the text.
Every cleanup outcome is checked. If this function fails, stop using its session
resources and destroy the instance before continuing; a rejected final release may
still leave a resource owned by the caller.
It requires CPU-resident joint tensors and rejects a device tensor before committing that group.
Use the Python example below for a complete CPU/CUDA fill, or fill native device tensors and supply the synchronization described in
[CUDA synchronization](device.md#cuda-synchronization).

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
#include <stdio.h>

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

int read_joints_and_zero_targets(ovphysx_instance_handle_t physx)
{
    int failed = 1;
    int read_open = 0;
    int write_open = 0;
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;
    ovphysx_write_handle_t write = 0;
    const ovx_string_or_token_t read_attr = {
        0, { OVPHYSX_ATTR_JOINT_POSITION, sizeof(OVPHYSX_ATTR_JOINT_POSITION) - 1 }
    };
    const ovx_string_or_token_t write_attr = {
        0, { OVPHYSX_ATTR_JOINT_POSITION_TARGET,
             sizeof(OVPHYSX_ATTR_JOINT_POSITION_TARGET) - 1 }
    };

    // NULL result pointers skip the optional discovery and session summaries.
    if (!complete_task(physx, ovphysx_query(physx, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT,
                                          OVPHYSX_SCOPE_ALL, &query, NULL)))
        return 1;
    if (!complete_task(physx, ovphysx_read(physx, query, &read_attr, 1, &read, NULL)))
        goto cleanup;
    read_open = 1;
    for (;;) {
        const ovstage_read_group_t* group = NULL;
        if (!complete_task(physx, ovphysx_fetch_read_next(physx, read, &group)))
            goto cleanup;
        if (!group)
            break;
        for (size_t i = 0; i < group->data.tensor_count; ++i) {
            const DLTensor* tensor = &group->data.tensors[i];
            if (tensor->device.device_type != kDLCPU) {
                fprintf(stderr, "This example requires CPU tensors\n");
                goto cleanup;
            }
            // jointPosition is contiguous float32, one row per unlocked axis.
            const float* values = (const float*)((const char*)tensor->data + tensor->byte_offset);
            for (int64_t axis = 0; axis < tensor->shape[0]; ++axis)
                printf("joint %zu, axis %lld: %g\n", i, (long long)axis, (double)values[axis]);
        }
        if (!complete_task(physx, ovphysx_release_group(physx, read, group->read_group_id)))
            goto cleanup;
    }
    if (!complete_task(physx, ovphysx_write(physx, query, &write_attr, 1, &write, NULL)))
        goto cleanup;
    write_open = 1;
    for (;;) {
        ovphysx_fetched_write_group_t fetched = { NULL, 0 };
        if (!complete_task(physx, ovphysx_fetch_write_next(physx, write, &fetched)))
            goto cleanup;
        // fetched.attribute_index is 0 here: this session has one requested attribute.
        const ovstage_map_group_t* group = fetched.group;
        if (!group)
            break;
        for (size_t i = 0; i < group->data.tensor_count; ++i) {
            const DLTensor* tensor = &group->data.tensors[i];
            if (tensor->device.device_type != kDLCPU) {
                fprintf(stderr, "This example requires CPU tensors\n");
                goto cleanup;
            } // Release discards this group without committing it.
            float* values = (float*)((char*)tensor->data + tensor->byte_offset);
            for (int64_t axis = 0; axis < tensor->shape[0]; ++axis)
                values[axis] = 0.0f;
        }
        // Every tensor was filled synchronously on the host.
        if (!complete_task(physx, ovphysx_commit_group(physx, write, group,
                                                     (ovstage_cuda_sync_t){ 0, 0 })))
            goto cleanup;
    }
    failed = 0;

cleanup:
    if (write_open && !complete_task(physx, ovphysx_release_write(physx, write)))
        failed = 1;
    if (read_open && !complete_task(physx, ovphysx_release_read(physx, read)))
        failed = 1;
    if (!complete_task(physx, ovphysx_release_query(physx, query)))
        failed = 1;
    return failed;
}
```

**Python (CPU or CUDA scene)**

This example also uses one query for both sessions. Each call returns an `Operation`; `wait()`
collects it and returns its result, and raises `TaskError` for an execution failure. Leaving a
`with` block submits that object's release without waiting. Read arrays keep their session's
storage alive, so the physical read release is submitted only after the result, its `groups`
and every array alias are gone; the helpers below drop them when they return. The final
`wait_all()` then submits that queued release and collects every release.

```python
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType


def print_joint_positions(physx, query):
    with physx.read(query, ["jointPosition"]).wait() as result:
        while (group := result.fetch_next().wait()) is not None:
            for tensor in group.tensors:
                print(tensor.numpy())  # Host access for display; CUDA arrays are copied.


def zero_joint_targets(physx, query):
    with physx.write(query, ["jointPositionTarget"]).wait() as session:
        while (group := session.fetch_next().wait()) is not None:
            cuda_device = None
            for tensor in group.tensors:
                device = tensor.device
                assert device is not None
                tensor.zero_()  # Fill every mapped element on its native device.
                if tensor.size and device.is_cuda:
                    cuda_device = device
            if cuda_device is None:
                session.commit(group).wait()
            else:
                # Each native group belongs to one device. Name the fill stream as the producer;
                # the device scope makes its CUDA context current for the null-stream value 1.
                with wp.ScopedDevice(cuda_device):
                    stream = wp.get_stream(cuda_device)
                    session.commit(group, cuda_stream=int(stream.cuda_stream or 1)).wait()


def read_joints_and_zero_targets(physx):
    with physx.query(SimObjectType.ARTICULATION_JOINT, scope=ObjectScope.ALL).wait() as query:
        print_joint_positions(physx, query)
        zero_joint_targets(physx, query)
    physx.wait_all()
```

`physx.read_sync(...)` and `physx.write_sync(...)` instead create their own query, wait for every
fetch, and return a session whose `groups` are already populated. Their `with` block exit waits
for the releases it submits. Read storage that arrays still borrow is released after the last
alias is dropped, and a later `wait_all()` collects that release.

The subsections below cover object selection, object types, scope, selecting a subset of
prims, and the optional session results. On a
DirectGPU scene, step at least once (`ovphysx_step`) before the first read or write of device state
(rigid bodies, articulations, joints, deformables) — particle reads serve their authored host values
before the first step. See [Availability and fallback](data_model.md#step-first-precondition-directgpu).

### Selecting objects — queries

The read and write API requires a query, which holds the selection of objects the API acts on:
an `ovphysx_query_handle_t` in C, a `Query` in Python. The same query can be used for both the read
and the write API. A whole-type query (`ALL` or `ACTIVE` scope) is evaluated again each time a
session opens, after earlier simulation work completes; `ACTIVE` then uses the type-specific
active set described under *Scope — all vs active* below. A query opened with a prim-set
selection keeps the membership it resolved when it was created. An empty match is a valid
(non-zero) query handle, not a failure.

The last argument of `ovphysx_query`, `ovphysx_query_selection` and `ovphysx_query_contacts` is an optional
`ovstage_query_result_t*`. When it is non-NULL, a successful query task fills it with a discovery
snapshot taken at creation: `total_prim_count` and the tokens of the attributes this object type
can read, including filtered contact names only when the query has filters. Vehicle discovery
narrows this list to the union supported by the selected drive types. For other object types
it is the query's vocabulary, not a promise that each attribute produces a group
for this match or configuration; the session result and the groups themselves answer that. Pass
NULL to skip that discovery work. The attribute array belongs to the query and stays valid until
the query release is accepted or the instance is destroyed; detach invalidates its
dictionary use. `query_result_id` and `all_handle` are zero: they are not OVStage resources, so
never pass them to OVStage query or release calls. The snapshot never updates; recreate the query
to refresh it. Python's `PhysX.query()` requests it by default and exposes it as
`Query.metadata`; pass `metadata=False` to skip it.

Attribute tokens and prim lists belong to the attached Stage's path dictionary. In C, get it from
the Stage you attached with `ovstage_get_path_dictionary(stage)` (that is,
`stage->vtable->get_path_dictionary(stage->context)`); in Python, use `ovstage.PathDictionary(stage)`
or `query.dictionary`.

Wait successfully on the query task before using the handle or its result. Release it with
`ovphysx_release_query` (Python `query.close()`) and collect that release task too. Sessions already
opened from a query stay valid after the query is released.

### Contact queries

A contact query selects contact sensors and pairs each with F **filter** bodies, whose contacts the
per-filter attributes (force matrices and filtered contact detail — see
[Contact sensors](readable.md#contact-sensors)) break out. In C, `ovphysx_query_contacts` takes an array
of `ovphysx_contact_sensor_selection_t` — a sensor selection plus F filter selections, each an
`ovphysx_query_selection_t` (a glob or native prim handles) — and the common F. In Python, pass
`contacts=[ContactSelection(sensors, filters)]` to `PhysX.query` with
`SimObjectType.CONTACT_SENSOR` and wait for its `Query`, then
open `PhysX.read(query, attributes)`. The `read_sync` / `read_tokens_sync` conveniences accept
`contacts=` directly and wait for query creation, session opening and all group fetches.

- Rows are the selected contact sensors, including eligible child colliders on CPU simulation and GPU
  dynamics with CPU readback, in selection order then match order; a sensor
  already matched by an earlier selection is not repeated.
- A child collider's `PhysxContactReportAPI` makes it eligible as a sensor but does not enable its
  owner's reporting. It reads zero when neither actor reports matching contacts; see
  [Child collider sensors](readable.md#child-collider-sensors).
- Each filter selection matches **one** body (shared by every sensor of its selection) or **one per
  sensor** (paired by position). Any other count — or a sensor naming one filter body twice — fails the
  query when it is opened, with an error naming the selection and the counts. The check depends only on
  which prims the selections match, so an opened query never starts failing later. More than 21845
  filters per sensor is refused at the call, as an invalid argument.
- Pairing by position comes after that de-duplication, as in the contact binding: if a selection
  re-matches earlier sensors and a filter happens to match as many bodies as its remaining sensors, they
  pair wrongly without an error. Keep sensor selections from overlapping.
- A contact query has no scope, and cannot be combined with `prim_paths` / `pattern`.

### SDF shapes

An `SDF_SHAPE` query selects collision shapes that carry a signed distance field: triangle-mesh
colliders with `PhysxSDFMeshCollisionAPI` and a cooked SDF. The type has no read or write attribute.
Evaluate the fields at your own points with `ovphysx_evaluate_sdf_shapes` in C (declared in
`ovphysx/ovphysx_geometry_query.h`, which `ovphysx.h` does not include), or
`PhysX.evaluate_sdf_shapes` in Python, which opens and releases the query itself.

- Points `[N, Q, 3]` go in and results `[N, Q, 4]` come out: gradient x, y, z, then signed distance,
  negative inside. Both arrays are float32, contiguous and on the scene's CUDA device. N is the number
  of selected shapes (in C, the `total_prim_count` of the summary `ovphysx_query_selection` returns);
  Q, the points per shape, is read from the arrays on each call.
- Points and gradients are in each shape's mesh-local space, the frame of the mesh's raw vertex
  positions. No prim transform or scale is applied.
- A point outside the shape's SDF grid (the mesh bounds padded by the SDF margin) is evaluated, not
  refused, as the SDF view did: its distance is the SDF value at the nearest grid point plus the
  distance to the grid, a positive upper bound of the true distance, and its gradient points away from
  the shape.
- Row *i* is the *i*-th selected shape: prim handles in the order given, a pattern in the deprecated
  SDF view's order, the whole set in engine order. A selected prim that is not an SDF shape is skipped
  with a warning.
- DirectGPU scenes only, with every selected shape in one scene, at most 65535 shapes per call (PhysX
  launches its SDF evaluation with one CUDA grid row per shape) and at most 2^31 - 1 points in all (N x Q,
  which PhysX indexes in 32-bit arithmetic). Before the first step the call runs the warmup step itself.
- The scene must have simulated since its physics objects were last created or released: PhysX uploads
  shape data to the GPU only when it simulates. Past the warmup the call never steps, so after such a
  change, step once before evaluating; until then the call fails naming the missing step.
- Points still being written on your own CUDA stream are handed off as for a write commit: pass the
  producer stream and/or completion event (`points_ready` in C, `cuda_stream=` / `cuda_wait_event=` in
  Python), and the call waits for them before reading the points.

### Object types

A query targets exactly one object type — `OVPHYSX_QUERY_TARGET_<NAME>` in C
(`ovphysx_query_target_t`), `SimObjectType.<NAME>` in Python (`ovphysx.types`):

| Value | Type | Keyed on (prim) |
|---|---|---|
| 0 | `RIGID_BODY` | dynamic rigid bodies — standalone **and** point-instancer instances |
| 1 | `ARTICULATION_LINK` | articulation link bodies |
| 2 | `ARTICULATION_JOINT` | reduced-coordinate joint DOFs (per joint prim) |
| 3 | `VEHICLE_WHEEL` | vehicle wheel attachments |
| 4 | `DEFORMABLE_VOLUME` | volume (tetrahedral) deformable sim meshes |
| 5 | `DEFORMABLE_SURFACE` | surface (triangle) deformable sim meshes |
| 6 | `PARTICLE_SET` | particle sets |
| 7 | `FIXED_TENDON` | articulation fixed tendons (root axis's joint prim) |
| 8 | `SPATIAL_TENDON` | articulation spatial tendons (root attachment's link prim) |
| 9 | `ARTICULATION` | whole articulations (the `PhysicsArticulationRootAPI` prim) |
| 10 | `DEFORMABLE_MATERIAL` | deformable materials (the bound `Material` prim) |
| 11 | `CONTACT_SENSOR` | bodies carrying `PhysxContactReportAPI`, and every link of an articulation carrying it — and, on CPU simulation and GPU dynamics with CPU readback, each collider carrying it (host-only; a DirectGPU scene omits separate child-collider rows). A child collider's schema enables selection, not its owner's reporting; see [Child collider sensors](readable.md#child-collider-sensors). A body row aggregates all of its shapes' contacts and each collider row reads only its own shape's, so the two overlap: do not sum net forces or count contacts/events across a body and its own colliders, or they double-count |
| 12 | `CHARACTER_CONTROLLER` | capsule character controllers (the prim carrying `PhysxCharacterControllerAPI`) |
| 13 | `VEHICLE` | whole vehicles (the vehicle/chassis prim) |
| 14 | `SCENE` | `PxScene` (the physics-scene prim); `physxPointer` only |
| 15 | `PHYSX_MATERIAL` | `PxMaterial`; `physxPointer` only |
| 16 | `SHAPE` | `PxShape`; `physxPointer` only |
| 17 | `COMPOUND_SHAPE` | compound-shape wrapper; `physxPointer` only |
| 18 | `RIGID_ACTOR` | dynamic and static rigid actors; `physxPointer` only |
| 19 | `JOINT` | standalone `PxJoint`; `physxPointer` only |
| 20 | `CUSTOM_JOINT` | plugin custom joint; `physxPointer` only |
| 21 | `PARTICLE_SYSTEM` | particle system; `physxPointer` only |
| 22 | `PHYSICS` | process-global `PxPhysics`; one row, path `/` |
| 23 | `DEBUG_RENDER` | this instance's debug-render target. Not a prim. Settings are one row. Point, line, and triangle positions and colors are separate columns from the same read; index `i` is the same primitive. An empty column is no geometry. Only scenes belonging to this instance are included. A query does not require an attached stage. Pass `&OVPHYSX_NAME_DEBUG_*` in C, or the string name in Python. See [Readable](readable.md#debug-render) / [Writable](writable.md#debug-render). |
| 24 | `SDF_SHAPE` | SDF collision shapes — triangle-mesh colliders carrying `PhysxSDFMeshCollisionAPI` (query-only, see [SDF shapes](#sdf-shapes)) |

An attribute name is **scoped to the object type**: `position` means a rigid body's world pose, an
articulation link's (read-only) pose, or a vehicle wheel's (read-only) pose depending on the queried
type. Each type accepts its own attribute list and refuses the rest by name.

`ARTICULATION` and `ARTICULATION_LINK` both match an articulation-root prim; the link type serves
per-body properties, the whole-articulation type serves root state — neither shadows the other.
Character controllers are rows of `RIGID_BODY` `position` and `orientation` for the controller
prim. That pose is the backing actor's global pose. Its orientation includes the capsule-axis
rotation and is not the fixup-corrected orientation written back to the prim. `move` is applied
once at commit; the rigid-body pose updates on the next step.

For one car at `/World/Car`, select that prim as `RIGID_BODY` for chassis pose/velocity, as
`VEHICLE` for controls/transmission, and `/World/Car/**` as `VEHICLE_WHEEL` for its attachments.
These are independent queries: an `ALL` rigid-body query also includes unrelated bodies, and an
`ALL` wheel query includes every vehicle's wheels. A vehicle query does not implicitly scope
another query. See the complete [vehicle examples](vehicles.md).

### Scope — all vs active

A query also takes a scope, which controls the selected objects:

- In C: `OVPHYSX_SCOPE_<NAME>` (`ovphysx_object_scope_t`).
- In Python: `ObjectScope.<NAME>`.

Two scopes are available today:

- `ALL` — every object of the type. Stable until a structural change.
- `ACTIVE` — a type-specific active set. Where a scene reports active actors, rigid bodies and
  articulation links use their own activity from the **last completed** step, vehicles and vehicle wheels use
  their chassis, and a whole articulation is active when any link is. Without those reports,
  rigid bodies use current awake, non-kinematic state; links and whole articulations use
  articulation awake state; vehicles and wheels use chassis awake state. Deformables use current awake
  state. Character controllers use their proxy actor's active-actor membership when reported,
  otherwise that actor's awake state. Each session opened from the reusable query evaluates that
  type's current set. Its
  creation-time result and already-open sessions retain their earlier snapshots, so open a new
  session for fresh membership.

`ACTIVE` has no meaning for joints, tendons, particle sets, deformable materials, contact sensors, or SDF shapes —
for those it behaves as `ALL`. On a DirectGPU scene sleeping is disabled, so a whole-articulation `ACTIVE` query
equals `ALL` there.

To act on specific prims instead of a whole scope, see
[Selecting a subset of prims](#selecting-a-subset-of-prims).

### Selecting a subset of prims

To act on specific prims instead of every object of the type, open the query with a prim-set
selection: `ovphysx_query_selection` in C, or the `prim_paths=` or `pattern=` argument of
`PhysX.query` (and of `read_sync` / `read_tokens_sync` / `write_sync`) in Python. The selection
narrows the read **and** a write opened over the same query.

- **`prim_paths`** is an explicit list of native `ovx_primpath_t` handles the caller already holds:
  from a read group's prim list, or interned through the attached Stage's path dictionary
  (`ovstage.PathDictionary.intern_path` in Python). Handles name objects by identity; a handle with
  no live object matches nothing.
- **`pattern`** is a path pattern such as `/envs/*/Robot/base_link`. Its matching rules — how `*`,
  `**` and the last component behave — are exact and differ from a shell glob; see
  [Path patterns](path_patterns.md).

When both are given, `prim_paths` wins and `pattern` is ignored. A selected query has **no scope**:
it covers every selected object, sleeping ones included, and stays valid across steps until a
structural change. In Python, combining a selection with `scope=ACTIVE` raises `ValueError`; for
only the objects that moved, use the whole-set `ACTIVE` query. A selection that matches no object
is a valid empty query.

Either kind of selection names prims, and the query keeps the objects of its type among them. An
`ARTICULATION` query also keeps each articulation one of whose links or joints is selected; every
other type keeps only the objects whose own prim is selected. See
[What a pattern selects](path_patterns.md#what-a-pattern-selects).

A selection filters the rows; it does not order them. Rows come in engine order, which is neither
the order of `prim_paths` nor the tensor-binding order, so identify each row through its group's
prim list (row *i* is prim *i*), not by position.

### Session results

`ovphysx_read` and `ovphysx_write` take a final optional `out_summary` pointer, filled only when the
open task succeeds. Pass NULL to skip the summary work. Python's `read()` and `write()` skip it by
default and expose it as `metadata` when called with `metadata=True`.

- `ovphysx_read_result_t` reports `total_prim_count`, the query membership this session evaluated
  (prim identities, not emitted rows), and one `ovphysx_read_attribute_result_t` row per requested
  attribute, in request order. A row holds the canonical `attribute` token (`OVX_INVALID_TOKEN` for
  a name the object type cannot read), the number of groups the session emits for that request, and
  `omission_reasons`, a bitmask of `ovphysx_read_omission_t` values (Python `ReadOmission`):
  - `OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE` — the name is not readable on this object type.
  - `OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION` — the attribute exists, but some selected
    objects cannot produce it in this scene or device configuration.
  - `OVPHYSX_READ_OMISSION_NOT_READY` — some selected objects can produce it later, for example
    after the first step of a DirectGPU scene or after pending scene insertion.

  Bits combine across applicable selected partitions; they do not identify or count the affected
  objects. Zero bits with zero groups is a valid empty answer; a
  nonzero group count with bits set is a partial answer. A known backend failure fails the open
  task instead of setting a bit, and a failed fetch is never end of iteration. The rows stay valid
  until the read release is accepted.
- `ovphysx_write_result_t` reports `total_prim_count` and one `ovphysx_write_attribute_result_t` row
  per requested attribute, in request order: the canonical token and the planned group count. It
  does not predict whether a later commit succeeds.

In both summaries, `group_count` counts batches, not objects or tensor rows. A group can contain
many objects. The C/C++ summary structs contain borrowed `attributes` arrays: copying a struct
copies its pointer, not the rows. To retain the rows, copy all `attribute_count` entries before
their session's release is accepted or the instance is destroyed. Write arrays also expire on
stage detach. Python's `ReadMetadata` and `WriteMetadata` already own immutable
copies of these rows. In either language, copying a token does not preserve the dictionary needed
to resolve it.

This C helper prints a read summary after the `ovphysx_read` task that filled it succeeded:

```c
#include <ovphysx/ovphysx.h>
#include <stdio.h>

static void print_read_summary(const ovphysx_read_result_t* summary)
{
    printf("%zu matched prims\n", summary->total_prim_count);
    for (size_t i = 0; i < summary->attribute_count; ++i) {
        const ovphysx_read_attribute_result_t* row = &summary->attributes[i];
        printf("request %zu: %zu groups", i, row->group_count);
        if (row->omission_reasons & OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE)
            printf(", unsupported attribute");
        if (row->omission_reasons & OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION)
            printf(", unsupported configuration");
        if (row->omission_reasons & OVPHYSX_READ_OMISSION_NOT_READY)
            printf(", not ready yet");
        printf("\n");
    }
}
```

Both calls validate every requested attribute before publishing a session. A token takes
precedence over a string in the same entry and must belong to the attached Stage's dictionary; a
token that does not resolve, or a duplicate (including a string and a token naming the same
attribute), fails the open task. A read reports an unknown name as an unsupported row, and an empty
attribute list is a valid empty read. A write rejects an empty list before admission, and an
unsupported or read-only name fails the open task, so a write session is never published with a
subset of its request.

`ovphysx_fetch_write_next` fills a caller-owned `ovphysx_fetched_write_group_t`. Its `group` is the
producer-owned group to fill and pass unchanged to `ovphysx_commit_group`. Its `attribute_index` is
the position of the group's attribute in the original request array — the only reliable way to tell
same-shaped attributes such as `linearVelocity` and `angularVelocity` apart. Fetch order is not
guaranteed. End of iteration is a successful fetch with `group == NULL`
and `attribute_index == SIZE_MAX`. Python's `WriteGroup` carries the same `attribute_index` and the
canonical `attribute` token.

A write session is not a transaction. Each accepted commit spends its group, commits run in the
order the caller issues them, and an earlier successful commit stays applied when a later one
fails.
