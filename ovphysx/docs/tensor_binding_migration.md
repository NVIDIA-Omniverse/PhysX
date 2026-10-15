<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Migrating off Tensor Bindings

> **The tensor-binding API is deprecated since ovphysx 0.6.** It is superseded by the session
> read/write API — `ovphysx_read` / `ovphysx_write` in C, `PhysX.read` / `PhysX.write` in Python.
> Bindings keep working through the deprecation window; new code should use the session API.

This guide maps each tensor-binding capability to its session read/write equivalent. For the full
data contract — which attributes exist on each object type, their semantics, units, format and
device — refer to the [read/write reference](read_write/index.md); this page is the *migration
path*, not the contract.

## What changes across the board

These differences apply to every capability below; the capability map only repeats a note where a
row diverges from them.

- **A binding becomes a reusable query plus per-call sessions.** `query(object_type)` returns a
  selector you can reuse with `read` / `write`. Close each session and release the query separately
  with `ovphysx_release_query` in C or `Query.close().wait()` in Python when finished. Closing a
  session does not release a caller-owned query. Python's `read_sync` / `write_sync` conveniences
  create an internal query per call and release it with their session.
- **One packed `TensorType` becomes an `(object type, attribute)` pair.** A binding's single packed
  column splits into per-attribute columns. The [Data attribute map](#data-attribute-map) below
  lists the split for the common columns; for example:
  - `RIGID_BODY_POSE` `(N, 7)` => `position` `(N, 3)` + `orientation` `(N, 4)`
  - `RIGID_BODY_VELOCITY` `(N, 6)` => `linearVelocity` `(N, 3)` + `angularVelocity` `(N, 3)`
- **Angular units are per-axis, not per column.** An `ARTICULATION_JOINT` joint row holds one value
  per unlocked reduced-coordinate DOF *axis*, so the unit of `jointPosition`, `jointVelocity`,
  `jointPositionTarget`, and `jointVelocityTarget` follows the axis kind, not the column: an **angular
  axis (revolute, or a D6 rotational DOF) is always in degrees**, and a **prismatic axis stays in the
  stage's base length unit** (no conversion). The joint's axis kind fixes this — authoring a
  `JointStateAPI` does not change it. A binding value on an angular axis must be scaled to degrees,
  but scaling a prismatic axis by 57.3× is a bug — so convert by the axis kind you authored in USD
  (no per-row attribute reports it back), not by blanket-converting the column. Refer to
  [ovstage integration](ovstage_integration.md) for the full per-axis rule. Everything else,
  including the body / link / root `angularVelocity` columns, is the engine's native **radians** with
  **no conversion** — scaling those by 57.3× is likewise a bug.
- **A subset is a prim selection, not a device index/mask.** `binding.write(tensor, indices=indices)`
  or `binding.write(tensor, mask=mask)` addressed rows by device position. The session API narrows a
  query by prim identity instead — Python
  `query(object_type, prim_paths=..., pattern=...)`, C `ovphysx_query_selection`. Selection is
  host-resolved by prim, so there is no GPU-resident row mask capturable inside a CUDA graph.
  Python `prim_paths` takes interned prim handles, not the binding's path strings. Groups return
  rows in engine order, not input-selection or binding order; resolve each group's prim list
  before mapping values to rows. See [Selecting a subset of prims](read_write/index.md#selecting-a-subset-of-prims).
- **Writability is queryable at runtime.** With bindings, a `TensorType`'s read/write capability was
  a fixed flag. In the session API, which `(object type, attribute)` pairs are writable is checked at
  write time: `write_sync()` raises `TaskError` (a `RuntimeError`) for an unwritable attribute, and a
  default `write()` reports it when its operation is waited. The
  [Writable data](read_write/writable.md) reference lists the writable set.
- **Session calls are asynchronous — wait before you use the result.** A binding's `read()` /
  `write()` handed you data (or applied it) immediately. A session `query`, `read`, `write`,
  `fetch_next`, `commit` or `close` instead returns a receipt you wait on before its output is valid;
  in Python the receipt is an `Operation` you `.wait()`. The `read_sync` / `write_sync` variants do
  the waiting for you and return a session with its groups already fetched. Fetching past the last
  group is not an error: the wait succeeds with a null group, which marks the end. A write session can
  carry several attributes; each fetched group's `attribute_index` names which one it holds. The calls
  run inline today, but GPU work still needs its own CUDA event/stream ordering.

## Capability map

Each row maps a tensor-binding API to its session read/write equivalent; the cross-cutting changes
above are not repeated here.

| Tensor-binding API | Session read/write equivalent |
|---|---|
| `create_tensor_binding(type, paths)` | `query(object_type)` for the whole set; `query(object_type, prim_paths=…, pattern=…)` / C `ovphysx_query_selection` for a subset |
| `binding.read()` | `read(query, attributes)` / `read_sync(object_type, attributes, …)` |
| `binding.write()` | `write(query, attributes)` / `write_sync(object_type, attributes, …)` |
| `binding.write(tensor, indices=indices)` / `binding.write(tensor, mask=mask)` | a selection query (`prim_paths` / `pattern`) feeding `write`; resolve group prim identities before assigning rows |
| `tensor_binding_get_prim_paths()` | resolve each fetched group's prim-list handle through the query's path dictionary: `prim_list` / `prim_offset` / `prim_count` in Python, `prims.list` / `prims.offset` / `prims.count` in C |
| `get_tensor_binding_spec()` / `get_tensor_binding_native_device()` | each fetched group's `prim_count` and each tensor's `shape`, `dtype`, and `device` in Python; `prims.count` and `data.tensors` descriptors in C. These do not require `metadata=True`, which adds count/availability summaries. See the [data contract](read_write/index.md). |
| `get_articulation_metadata()` | read the read-only `dofCount`, `linkCount`, `jointCount`, `fixedTendonCount`, `spatialTendonCount`, `isFixedBase` attributes on `ARTICULATION` |
| `articulation_get_dof_names()` | read `dofJointPath` + `dofAxisIndex` on `ARTICULATION`; key each DOF column by the full joint path and its unlocked-axis ordinal, since several DOFs can share a joint and different joints can share a leaf name |
| `articulation_get_body_names()` | resolve the prim paths of fetched `ARTICULATION_LINK` groups and take their leaf names |
| `articulation_get_joint_names()` | resolve the prim paths of fetched `ARTICULATION_JOINT` groups and take their leaf names. Fully locked (zero-DOF) joints have no group or descriptor entry; their names are not recoverable through the read API. See [Readable data](read_write/readable.md). |
| `articulation_update_kinematic()` | `update_articulations_kinematic(flags=…, scope=…, prim_paths=…, pattern=…)` — a full drop-in that adds kinematic flags and subset selection |
| `rigid_body_view_wake_up()` / `_sleep()` | write the `wake` attribute on `RIGID_BODY` (`uint8`: nonzero wakes, zero sleeps; write-only) — a data write, not a method call |
| `destroy_tensor_binding()` | release the reusable query with `ovphysx_release_query` / `Query.close().wait()`; close read/write sessions separately. Python `read_sync` / `write_sync` sessions also release their internal query. |

## Data attribute map

This table maps each tensor-binding `TensorType` to its session object type and attributes:

| Tensor binding `TensorType` | Session object type | Session attribute(s) |
|---|---|---|
| `RIGID_BODY_POSE` | `RIGID_BODY` | `position`, `orientation` |
| `RIGID_BODY_VELOCITY` | `RIGID_BODY` | `linearVelocity`, `angularVelocity` (rad/s — no conversion) |
| `RIGID_BODY_FORCE` / `RIGID_BODY_WRENCH` | `RIGID_BODY` | `force` / `wrench` (write-only) |
| `RIGID_BODY_MASS` / `RIGID_BODY_INERTIA` | `RIGID_BODY` | `mass` / `inertia` |
| `RIGID_BODY_COM_POSE` | `RIGID_BODY` | `centerOfMassPosition`, `centerOfMassOrientation` |
| `ARTICULATION_DOF_POSITION` / `ARTICULATION_DOF_VELOCITY` | `ARTICULATION_JOINT` | `jointPosition` / `jointVelocity` (per-axis units — refer to the per-axis angular-unit rule above) |
| `ARTICULATION_DOF_POSITION_TARGET` / `..._VELOCITY_TARGET` | `ARTICULATION_JOINT` | `jointPositionTarget` / `jointVelocityTarget` (per-axis units — refer to the per-axis angular-unit rule above) |
| `ARTICULATION_ROOT_POSE` | `ARTICULATION` | `rootPosition`, `rootOrientation` |
| `ARTICULATION_ROOT_VELOCITY` | `ARTICULATION` | `rootLinearVelocity`, `rootAngularVelocity` (rad/s — no conversion) |
| `ARTICULATION_LINK_POSE` / `ARTICULATION_LINK_VELOCITY` | `ARTICULATION_LINK` | `position` + `orientation` / `linearVelocity` + `angularVelocity` (rad/s — no conversion) |
| `ARTICULATION_LINK_WRENCH` | `ARTICULATION_LINK` | `wrench` (write-only) |

This is the common subset. The full attribute vocabulary is the `OVPHYSX_ATTR_*` macros in
`ovphysx_attributes.h` (include it alongside `ovphysx.h`, which does not include it); the macros are
C-only, and a Python caller passes the plain attribute-name string directly, e.g. `"position"`. Which `(object type, attribute)` pairs are writable is queryable at runtime
through `ovphysx_writability` (no *public* Python binding) and listed in the
[Writable data](read_write/writable.md) reference; the full set of readable columns is in
[Readable data](read_write/readable.md).

## Using the session API

A minimal write on DirectGPU (`suppressReadback`): zero every rigid body's linear velocity by filling its mapped device
tensor in place. Complete a warmup first because DirectGPU writes require a first simulation step:

```python
import warp as wp
from ovphysx.types import SimObjectType

# `physx` has an attached DirectGPU scene; its mapped tensors are Warp arrays on the GPU.
physx.warmup()
physx.wait_all()

with physx.write_sync(SimObjectType.RIGID_BODY, "linearVelocity") as session:
    for group in session.groups:
        if not group.tensors:
            continue
        for tensor in group.tensors:
            tensor.zero_()  # fill in place on the tensor's native (CUDA) device
        device = group.tensors[0].device
        with wp.ScopedDevice(device):
            session.commit_sync(group, cuda_stream=int(wp.get_stream(device).cuda_stream or 1))

physx.step_sync(1.0 / 60.0)
```

For the full query → read/write flow — opening a query, fetching its groups, and filling the mapped
buffers on their native device — follow [The session model](read_write/index.md#the-session-model)
and [Session results](read_write/index.md#session-results); the
[ovstage integration](ovstage_integration.md) guide walks a complete closed-loop read. Runnable
samples: `tests/python_samples/session_write.py` (CPU write) and `tests/python_samples/output_read.py`
/ `tests/c_samples/output_read_c/` (reads), `tests/c_samples/vehicle_read_write_cpp/` (C++ write).
These are source-checkout paths relative to `ovphysx/`; see the
[tensor-bindings tutorial](tutorials/tensor_bindings.md) for their installed wheel and SDK locations.
