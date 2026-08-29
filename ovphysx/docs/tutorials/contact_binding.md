<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Contact Binding: Reading Contact Forces

Contact bindings let you read contact forces between **sensor** bodies and
**filter** bodies. A sensor is a rigid body (or a set of bodies matched by a
physics-object path pattern) whose contacts you want to measure. A filter is a
second set of bodies whose contacts with each sensor you want to isolate.

Contact reporting is opt-in: every authored USD prim matched by `sensor_patterns`
must have `PhysxContactReportAPI` applied. A matched prim without it is dropped
from the binding — the runtime logs `Failed to find contact report API at
'<path>'` — and if that leaves no sensors at all, `create_contact_binding()`
fails. Filter prims need no extra schema, and runtime-only clones (which have no
USD prim) inherit contact reporting from the source actor.

## Prerequisites

- Complete the [Tensor Bindings](tensor_bindings.md) tutorial.
- Your USD scene has rigid body prims in contact (or that will come into contact
  during simulation).
- Every authored USD prim you name in `sensor_patterns` has
  `PhysxContactReportAPI` applied:

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
  [GPU Warmup and Determinism](../developer_guide.md#gpu-warmup-and-determinism).

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

```{literalinclude} ../../tests/python_samples/contact_binding.py
:language: python
```

### Context-Manager Form (Recommended)

The complete sample above also demonstrates context-manager cleanup after
reloading the stage.

## C

```{literalinclude} ../../tests/c_samples/contact_binding_c/main.c
:language: c
```

## Unfiltered Contacts

Pass `filter_patterns=None` and `filters_per_sensor=0` to collect contacts with
all bodies:

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
`counts` and `start_indices` may be `int32` or `uint32`; NumPy's default integer
dtype is usually `int64`, so allocate these arrays with an explicit dtype.

If the flat buffers are too small, the read raises `RuntimeError` instead of
returning incomplete data. Do not use the payload arrays after that error.
`counts` and `start_indices` still contain the full required layout. Use
`int(np.max(start_indices.astype(np.int64) + counts.astype(np.int64)))` as
`max_contact_data_count` when
recreating the binding for subsequent simulation steps. Recreating a binding
does not recover the overflowing step's payload.

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
