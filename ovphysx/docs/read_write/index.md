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


A typical control step reads an attribute and writes another back: for example, read each
articulation joint's `jointPosition`, then set every `jointPositionTarget` to zero.
The application must already have attached a stage. On DirectGPU, complete the first step before
calling the Python example. Serialize simulation and structural edits while either example runs.

**C (CPU scene)**

This example uses a single query for both sessions. It returns nonzero on failure, releases all
opened handles, and discards an uncommitted group. On an API failure it reports
`ovphysx_get_last_error()` on the calling thread before cleanup can clear the diagnostic.
It requires CPU-resident joint tensors and rejects a device tensor before committing that group.
Use the Python example below for a complete CPU/CUDA fill, or fill native device tensors and supply the synchronization described in
[CUDA synchronization](device.md#cuda-synchronization).

```c
#include <ovphysx/ovphysx.h>
#include <stdio.h>

int read_joints_and_zero_targets(ovphysx_handle_t physx)
{
    int failed = 1;
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

    if (ovphysx_query(physx, OVPHYSX_OBJECT_ARTICULATION_JOINT,
                     OVPHYSX_SCOPE_ALL, &query).status != OVPHYSX_API_SUCCESS)
        goto cleanup;
    if (ovphysx_read(physx, query, &read_attr, 1, &read).status != OVPHYSX_API_SUCCESS)
        goto cleanup;
    for (;;) {
        const ovstage_read_group_t* group = NULL;
        const ovphysx_result_t result = ovphysx_fetch_read_next(physx, read, &group);
        if (result.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        if (result.status != OVPHYSX_API_SUCCESS || !group)
            goto cleanup;
        for (size_t i = 0; i < group->data.tensor_count; ++i) {
            const DLTensor* tensor = &group->data.tensors[i];
            if (tensor->device.device_type != kDLCPU)
                goto cleanup;
            // jointPosition is contiguous float32, one row per unlocked axis.
            const float* values = (const float*)((const char*)tensor->data + tensor->byte_offset);
            for (int64_t axis = 0; axis < tensor->shape[0]; ++axis)
                printf("joint %zu, axis %lld: %g\n", i, (long long)axis, (double)values[axis]);
        }
        if (ovphysx_release_group(physx, read, group->read_group_id).status != OVPHYSX_API_SUCCESS)
            goto cleanup;
    }
    if (ovphysx_release_read(physx, read).status != OVPHYSX_API_SUCCESS)
        goto cleanup;
    read = 0;

    if (ovphysx_write(physx, query, &write_attr, &write).status != OVPHYSX_API_SUCCESS)
        goto cleanup;
    for (;;) {
        const ovstage_map_group_t* group = NULL;
        const ovphysx_result_t result = ovphysx_fetch_write_next(physx, write, &group);
        if (result.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        if (result.status != OVPHYSX_API_SUCCESS || !group)
            goto cleanup;
        for (size_t i = 0; i < group->data.tensor_count; ++i) {
            const DLTensor* tensor = &group->data.tensors[i];
            if (tensor->device.device_type != kDLCPU)
                goto cleanup; // Release discards this group without committing it.
            float* values = (float*)((char*)tensor->data + tensor->byte_offset);
            for (int64_t axis = 0; axis < tensor->shape[0]; ++axis)
                values[axis] = 0.0f;
        }
        // Every tensor was filled synchronously on the host.
        if (ovphysx_commit_group(physx, write, group,
                                (ovstage_cuda_sync_t){ 0, 0 }).status != OVPHYSX_API_SUCCESS)
            goto cleanup;
    }
    failed = 0;

cleanup:
    if (failed) {
        const ovphysx_string_t error = ovphysx_get_last_error();
        if (error.length != 0)
            fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
    }
    if (write)
        ovphysx_release_write(physx, write);
    if (read)
        ovphysx_release_read(physx, read);
    if (query)
        ovphysx_release_query(physx, query);
    return failed;
}
```

**Python (CPU or CUDA scene)**

The Python wrappers each create and release their own query. The same object type and `ALL` scope
select the same joints while the scene's structure is unchanged.

```python
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType


def read_joints_and_zero_targets(physx):
    with physx.read(
        SimObjectType.ARTICULATION_JOINT, ["jointPosition"], scope=ObjectScope.ALL
    ) as result:
        for group in result.groups:
            for tensor in group.tensors:
                print(tensor.numpy())  # Explicit host copy for display.

    with physx.write(
        SimObjectType.ARTICULATION_JOINT, "jointPositionTarget", scope=ObjectScope.ALL
    ) as session:
        for group in session.groups:
            for tensor in group.tensors:
                tensor.zero_()  # Fill every mapped element on its native device.
            cuda = next((t for t in group.tensors if t.size and t.device.is_cuda), None)
            if cuda is None:
                session.commit(group)
            else:
                # Each native group belongs to one device; order commit after its fill stream.
                stream = wp.get_stream(cuda.device)
                session.commit(group, cuda_stream=int(stream.cuda_stream or 1))
```

The subsections below cover object selection, object types, and scope. On a
DirectGPU scene, step at least once (`ovphysx_step`) before the first read or write; see
[Availability and fallback](data_model.md#step-first-precondition-directgpu).

### Selecting objects — queries

In C, the read and write API requires a query, which holds the selection of objects the API acts on.
The same query can be used for both the read and the write API.
The selection is resolved against the most-recently-completed step.
An empty match is a valid (non-zero) query handle, not a failure.
The query can be inspected with `ovphysx_fetch_query_result`, which gives the number of selected
objects and the names of their attributes; their prim paths come through
`ovphysx_query_shared_dictionary`.
The query must be released with `ovphysx_release_query`.

### Object types

A query targets exactly one object type — `OVPHYSX_OBJECT_<NAME>` in C
(`ovphysx_sim_object_type_t`), `SimObjectType.<NAME>` in Python (`ovphysx.types`):

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

An attribute name is **scoped to the object type**: `position` means a rigid body's world pose, an
articulation link's (read-only) pose, or a vehicle wheel's (read-only) pose depending on the queried
type. Each type accepts its own attribute list and refuses the rest by name.

`ARTICULATION` and `ARTICULATION_LINK` both match an articulation-root prim; the link type serves
per-body properties, the whole-articulation type serves root state — neither shadows the other.

### Scope — all vs active

A query also takes a scope, which controls the selected objects:

- In C: `OVPHYSX_SCOPE_<NAME>` (`ovphysx_object_scope_t`).
- In Python: `ObjectScope.<NAME>`.

Two scopes are available today:

- `ALL` — every object of the type. Stable until a structural change.
- `ACTIVE` — only objects the solver moved on the **last** step. This set is **single-frame**: it is
  recomputed every step, so do not cache an `ACTIVE` query across steps.

`ACTIVE` has no meaning for joints, tendons, particle sets, or deformable materials — for those it
behaves as `ALL`. On a DirectGPU scene sleeping is disabled, so a whole-articulation `ACTIVE` query
equals `ALL` there.

A future release will extend these options to give the user full control over which objects a query
selects.
