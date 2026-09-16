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


A typical control step reads an attribute and writes another back through the **same query** — for
example, read each articulation joint's `jointPosition`, then write its `jointPositionTarget`:

**C**

```c
// One query drives both the read and the write.
ovphysx_query_handle_t query = 0;
ovphysx_query(physx, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &query);

// READ jointPosition — open a session, fetch each column group, release it.
const ovx_string_or_token_t read_attrs[] = {
    { 0, { OVPHYSX_ATTR_JOINT_POSITION, sizeof(OVPHYSX_ATTR_JOINT_POSITION) - 1 } },
};
ovphysx_read_handle_t read = 0;
ovphysx_read(physx, query, read_attrs, 1, &read);
for (;;) {
    const ovstage_read_group_t* g = NULL;
    if (ovphysx_fetch_read_next(physx, read, &g).status == OVPHYSX_API_END_OF_ITERATION)
        break;
    // consume g->data.tensors[...] — DLPack tensors on the group's native device
    ovphysx_release_group(physx, read, g->read_group_id);
}
ovphysx_release_read(physx, read);

// WRITE jointPositionTarget — one attribute per session; fill each group, then commit.
const ovx_string_or_token_t write_attr =
    { 0, { OVPHYSX_ATTR_JOINT_POSITION_TARGET, sizeof(OVPHYSX_ATTR_JOINT_POSITION_TARGET) - 1 } };
ovphysx_write_handle_t write = 0;
ovphysx_write(physx, query, &write_attr, &write);
for (;;) {
    const ovstage_map_group_t* g = NULL;
    if (ovphysx_fetch_write_next(physx, write, &g).status == OVPHYSX_API_END_OF_ITERATION)
        break;
    // fill g->data.tensors[...].data; {0, 0} = no outstanding CUDA work to wait on
    ovphysx_commit_group(physx, write, g, (ovstage_cuda_sync_t){ 0, 0 });
}
ovphysx_release_write(physx, write);

ovphysx_release_query(physx, query);
```

**Python**

```python
from ovphysx.types import ObjectScope, SimObjectType

# READ jointPosition — a context-managed session; groups are warp.array columns.
with physx.read(SimObjectType.ARTICULATION_JOINT, ["jointPosition"],
                scope=ObjectScope.ALL) as result:
    for group in result.groups:
        # group.tensors[i] is a warp.array on the group's native (CPU or CUDA) device
        ...

# WRITE jointPositionTarget — one attribute per session; fill, then commit each group.
with physx.write(SimObjectType.ARTICULATION_JOINT, "jointPositionTarget") as session:
    for group in session.groups:
        # group.tensors[i] are mutable warp.array views onto the mapped write memory
        session.commit(group)
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
