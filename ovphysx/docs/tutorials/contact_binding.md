<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Contact Binding -- Reading Contact Forces

Contact bindings let you read contact forces between **sensor** bodies and
**filter** bodies. A sensor is a rigid body prim (or a set of prims matched by a
USD path pattern) whose contacts you want to measure. A filter is a second set of
bodies whose contacts with each sensor you want to isolate.

Contact reporting is opt-in: every prim matched by `sensor_patterns` must have
`PhysxContactReportAPI` applied. A matched prim without it is dropped from the
binding — the runtime logs `Failed to find contact report API at '<path>'` — and if
that leaves no sensors at all, `create_contact_binding()` fails. Filter prims need
no extra schema, and runtime-only clones (which have no USD prim) inherit contact
reporting from the source actor.

## Prerequisites

- Complete the [Tensor Bindings (deprecated)](tensor_bindings.md) tutorial.
- Your USD scene has rigid body prims in contact (or that come into contact
  during simulation).
- Every prim you name in `sensor_patterns` has `PhysxContactReportAPI` applied:

  ```usda
  def Mesh "box" (
      prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysxContactReportAPI"]
  )
  {
      float physxContactReport:threshold = 0
  }
  ```

  The optional `physxContactReport:threshold` is the force below which contacts are
  not reported; the bundled sample scene uses `0` so every contact is reported.

  The schema must sit on the prim you actually name as the sensor. When the rigid
  body and its collider are separate prims, applying it to the body but naming the
  collider (or the reverse) matches nothing — the two must agree.
- For CUDA output tensors, enable DirectGPU TensorAPI before creating the
  `PhysX` instance; `physxScene:enableGPUDynamics=true` alone only selects GPU
  dynamics. Refer to
  [Warmup and Determinism](../developer_guide.md#warmup-and-determinism).

## Key Concepts

- **Create the binding before the first step** whose contacts you want to observe.
  The binding registers an internal contact-report callback. No contact data exists
  until at least one `step()`, `step_sync()`, or `step_n_sync()` call has
  completed.
- Create contact bindings once outside simulation loops and reuse them. In Python,
  use the context-manager form or call `cb.destroy()` when finished; otherwise
  garbage collection emits `ResourceWarning` when it eventually releases the
  native binding.
- Reading before the first step returns all-zeros tensors.
- `dt` for the impulse-to-force conversion (`force = impulse / dt`) is taken
  automatically from the last successful `step()`, `step_sync()`, or
  `step_n_sync()` call. You do not pass it manually.
- Result tensor shapes:
  - Net forces: `[S, 3]` -- one 3-D force vector per matched sensor object.
  - Force matrix: `[S, F, 3]` — force vectors per (sensor, filter) pair.
  - Detailed contact data: contact forces and separations use `[C, 1]`;
    positions and normals use `[C, 3]`; all are indexed by `[S, F]`
    count/start-index tensors.
  - Detailed friction data: friction forces and points use `[C, 3]` buffers
    indexed by `[S, F]` count/start-index tensors.

## Python

### Full Binding + Destroy

This sample creates a contact binding on `boxes_falling_on_groundplane.usda`,
steps until the sensor lands, reads both the net forces and the force matrix,
then destroys the binding explicitly:

```{literalinclude} ../../tests/python_samples/contact_binding.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

After 120 steps both `read_net_forces()` and `read_force_matrix()` print a
non-zero vector on the order of Cube1's weight (about 9810 N in Z). In
`boxes_falling_on_groundplane.usda`, Cube1 lands on `/World/BigBase`, not the
ground plane, so the sample's filter is that prim. A filter naming the ground
still creates a valid 1x1 binding; the matrix is then all zeros while net
force is not, because the pair never touches.

### Recommended Context-Manager Form

The context-manager form releases the native binding on scope exit, so no
explicit `destroy()` call is required:

```{literalinclude} ../../tests/python_samples/contact_binding.py
:language: python
:start-after: [tutorial-context-manager]
:end-before: [tutorial-context-manager-end]
```

## C

The C sample performs the same create, step, read, and destroy sequence through
the C API:

```{literalinclude} ../../tests/c_samples/contact_binding_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## Unfiltered Contacts

Pass `filter_patterns=None` and `filters_per_sensor=0` to collect contacts with
all bodies. The binding-creation functions in this and the next section are
standalone helpers; substitute your own scene's prim paths for the illustrative
`/World/robot/ee` and `/World/obstacle_*` paths:

```python
def create_unfiltered_binding(physx):
    return physx.create_contact_binding(
        sensor_patterns=["/World/robot/ee"],
        max_contact_data_count=512,
    )
```

In C:

```c
#include <ovphysx/ovphysx.h>

static ovphysx_result_t create_unfiltered_binding(
    ovphysx_handle_t handle,
    ovphysx_contact_binding_handle_t* out_binding)
{
    ovphysx_string_t sensors[] = { ovphysx_cstr("/World/robot/ee") };
    return ovphysx_create_contact_binding(
        handle, sensors, 1, NULL, 0, 512, out_binding);
}
```

## Multiple Sensors and Filters

The `filter_patterns` array is **flat** and must have length
`len(sensor_patterns) * filters_per_sensor`. Each block of `filters_per_sensor`
entries corresponds to one sensor:

```python
def create_filtered_binding(physx):
    # 2 sensors, 2 filters each -> 4 filter entries total
    return physx.create_contact_binding(
        sensor_patterns=["/World/robot_0/ee", "/World/robot_1/ee"],
        filter_patterns=[
            "/World/obstacle_A", "/World/obstacle_B",  # robot_0/ee filters
            "/World/obstacle_A", "/World/obstacle_B",  # robot_1/ee filters
        ],
        filters_per_sensor=2,
    )
```

## Detailed Contact and Friction Data

Use `cb.max_contact_data_count` to allocate reusable flat buffers. For each
sensor/filter pair, `counts[s, f]` and `start_indices[s, f]` identify the valid
slice inside the flat buffers.

`cb.sensor_paths` returns the resolved sensor paths in row order.
`cb.filter_paths` returns a nested `[sensor][filter]` list in column order.

Create the binding with `filter_patterns`, `filters_per_sensor > 0`, and
`max_contact_data_count > 0` before calling `read_contact_data()` or
`read_friction_data()`. The aggregate `read_net_forces()` and
`read_force_matrix()` calls do not require this detailed-contact capacity.
`counts` and `start_indices` can be `int32` or `uint32`; NumPy's default integer
dtype is usually `int64`, so allocate these arrays with an explicit dtype.

```python
import numpy as np

def read_detailed_data(binding):
    capacity = binding.max_contact_data_count
    contact_forces = np.zeros((capacity, 1), dtype=np.float32)
    positions = np.zeros((capacity, 3), dtype=np.float32)
    normals = np.zeros((capacity, 3), dtype=np.float32)
    separations = np.zeros((capacity, 1), dtype=np.float32)
    shape = (binding.sensor_count, binding.filter_count)
    counts = np.zeros(shape, dtype=np.int32)
    starts = np.zeros(shape, dtype=np.int32)

    binding.read_contact_data(
        contact_forces,
        positions,
        normals,
        separations,
        counts,
        starts,
    )

    sensor = 0
    contact_filter = 0
    start = starts[sensor, contact_filter]
    stop = start + counts[sensor, contact_filter]
    sensor_filter_positions = positions[start:stop]

    friction_forces = np.zeros((capacity, 3), dtype=np.float32)
    friction_points = np.zeros((capacity, 3), dtype=np.float32)
    friction_counts = np.zeros(shape, dtype=np.int32)
    friction_starts = np.zeros(shape, dtype=np.int32)

    binding.read_friction_data(
        friction_forces,
        friction_points,
        friction_counts,
        friction_starts,
    )
    return sensor_filter_positions, friction_forces, friction_points
```

Friction data is per friction anchor, not a pre-summed `[S, F, 3]` pair
force. To build a pair-level friction force tensor, sum
`friction_forces[start:stop]` for each `(sensor, filter)` pair using the
matching `friction_counts` and `friction_starts` entries.

This flat representation matches the underlying PhysX tensor API and avoids a
fixed per-pair contact-point dimension. Build a padded `[S, F, K, D]` view in
application code only if that layout is useful for a specific algorithm.

## Actor Identities in Raw Contact Data

`read_raw_contact_data()` tells you **which actor is touching which** without a separate call.
It takes six tensors: the four per-contact value buffers, plus two tensors whose columns carry
the quantities that are only meaningful in pairs:

- `sensor_layout` — `[S, 2]`, per sensor: column 0 the contact count, column 1 its start index
- `actor_ids` — `[C, 2]`, per contact: column 0 the reporting sensor's actor, column 1 the
  actor it contacted

Pairing them this way means the caller cannot get a count out of step with its start index, or
a sensor id out of step with its other id. No filter dimension is required for an unfiltered
binding. Slice the columns out as views when you want them separately — that costs no copy.

The examples in this section are sequential fragments. The path-resolution
example reuses `binding`, `sensor_actor_ids`, and `other_actor_ids` from the raw
read; the stale-ID example reuses its `ids`; and the filtering helper reuses the
earlier `numpy as np` import.

```python
import numpy as np

def read_raw_contact_data_with_ids(binding):
    capacity = binding.max_contact_data_count  # C
    S = binding.sensor_count                   # S (no filter dim needed)

    contact_forces = np.zeros((capacity, 1), dtype=np.float32)
    positions      = np.zeros((capacity, 3), dtype=np.float32)
    normals        = np.zeros((capacity, 3), dtype=np.float32)
    separations    = np.zeros((capacity, 1), dtype=np.float32)
    sensor_layout  = np.zeros((S, 2), dtype=np.int32)         # count, start
    actor_ids      = np.zeros((capacity, 2), dtype=np.uint64)  # sensor, other

    binding.read_raw_contact_data(
        contact_forces, positions, normals, separations,
        sensor_layout, actor_ids,
    )

    # Column views -- no copy.
    counts, starts = sensor_layout[:, 0], sensor_layout[:, 1]
    sensor_actor_ids, other_actor_ids = actor_ids[:, 0], actor_ids[:, 1]

    # Print the sensor and other-actor path for every contact point
    for sensor_idx in range(S):
        n = int(counts[sensor_idx])
        if n == 0:
            continue
        start = int(starts[sensor_idx])
        sensor_ids_slice = sensor_actor_ids[start : start + n]
        other_ids_slice  = other_actor_ids[start : start + n]

        # Resolve identities — both sensor and other IDs use the same namespace.
        # A column of actor_ids is a strided view, and the DLPack boundary requires
        # C-contiguous input, so make the slice contiguous before passing it.
        sensor_paths = binding.get_other_actor_paths_from_ids(np.ascontiguousarray(sensor_ids_slice))
        other_paths  = binding.get_other_actor_paths_from_ids(np.ascontiguousarray(other_ids_slice))

        for i in range(n):
            force = contact_forces[start + i, 0]
            print(f"  contact {start + i}: {sensor_paths[i]} ↔ {other_paths[i]}  force={force:.2f}")
```

### Resolving Actor Paths

`get_other_actor_paths_from_ids()` accepts a 1-D int64/uint64 array of actor IDs and
returns a list of USD path strings in the same order. Note that a column of `actor_ids` is
a strided view: the DLPack boundary requires C-contiguous input, so wrap a column slice in
`np.ascontiguousarray()` (or `.copy()`) before passing it. That copy is on the diagnostic
path only — the per-contact read itself takes the whole contiguous tensor. It works for both `sensor_actor_ids` and
`other_actor_ids` since both use the same identity namespace:

```python
ids = np.array([sensor_actor_ids[0], other_actor_ids[0]], dtype=np.uint64)
paths = binding.get_other_actor_paths_from_ids(ids)
# paths[0] → "/World/sensor_cube"
# paths[1] → "/World/other_cube"
```

**ID encoding**: actor IDs are opaque runtime handles, not encoded paths. Do not try to
decode one yourself — the only supported way to get a path back is
`get_other_actor_paths_from_ids()`. The encoding is the same regardless of how the stage
was attached, so IDs from a CPU scene and a GPU scene mean the same thing.

**ID lifetime**: handles are valid for the lifetime of the attach. Do not use an ID after
the stage is detached or replaced, and do not persist one across runs.

**Stale IDs resolve to empty, not to their old path.** Every non-zero ID is checked
against the attached stage before it is resolved, so if you hold an ID and the actor is
later removed, `get_other_actor_paths_from_ids()` returns `""` rather than the path it used
to name. Because you hold the IDs, that is an unambiguous signal:

```python
paths = binding.get_other_actor_paths_from_ids(ids)
for actor_id, path in zip(ids, paths):
    if actor_id == 0:
        continue          # no actor for this slot
    if not path:
        ...               # the actor is gone — the ID is stale
```

The check is as precise as the backend's notion of existence. On an ovstage attach a
removed prim reports stale. On a USD stage a merely *deactivated* prim still resolves,
because existence there follows prim validity and USD returns a valid prim for an inactive
one — so do not rely on deactivation alone to invalidate an ID.

**Truncation**: when a step produces more contacts than `max_contact_data_count`, the runtime
fills as many entries as fit and logs a warning. Per-sensor counts report only the contacts
actually written and start indices are clamped to `max_contact_data_count`, so
`[start, start + count)` is always a valid (possibly empty) slice. Increase
`max_contact_data_count` at binding creation if you see this warning.

**Ordering**: contacts are grouped by sensor. Within a sensor, the order of contact points
is whatever the simulation produced for that step and is not stable across steps.

**Threading**: contact reads follow the general ovphysx rule — a single instance is not
thread-safe, so serialize `read_raw_contact_data()` against `step()` and against other
reads on the same instance. Refer to
[Threading](../developer_guide.md#threading). `get_other_actor_paths_from_ids()`
additionally populates a per-binding path cache, so concurrent calls on the *same* binding
must be serialized even when no step is in flight.

**Filtering is caller-owned.** There is no binding-creation option to exclude expected
support contacts (a floor, a conveyor). Sensor and filter patterns select which *bodies*
participate; they do not suppress individual contact points, so dropping the ones you expect
is application policy.

Resolution only goes ID → path, so build the decision table lazily rather than up front:
resolve each ID the first time you see it, record the verdict against the ID, and from then
on filter with an integer lookup. IDs are stable for the lifetime of the attach, so this
settles after the first step and does no string work in steady state.

```python
ignored: dict[int, bool] = {}   # actor id -> "expected support contact"

def is_ignored(binding, actor_id: int) -> bool:
    verdict = ignored.get(actor_id)
    if verdict is None:
        path = binding.get_other_actor_paths_from_ids(
            np.array([actor_id], dtype=np.uint64)
        )[0]
        verdict = path.endswith("/floor") or path.endswith("/conveyor")
        ignored[actor_id] = verdict
    return verdict
```

Rebuild the table if you attach a different stage — an ID only means anything against the
attach it was read from. Note the memoized verdict outlives the actor: if a body can be
removed mid-run, re-resolve rather than trusting a cached entry, since a removed actor's ID
resolves to `""` and would not match the path test that produced the original verdict.

In C, `ovphysx_read_raw_contact_data` takes the same six tensors in the same order —
`sensor_layout_tensor` then `actor_ids_tensor` after the four value buffers. Refer to
the [`ovphysx_read_raw_contact_data` declaration](../api.md) for the full parameter
documentation.
