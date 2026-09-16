---
name: tensor-bindings-cpu
description: Exchange CPU simulation state — poses, velocities, joint targets — as caller-owned NumPy arrays through tensor bindings. The tensor-binding CODE API is deprecated in ovphysx 0.6 in favor of the session read/write API (Python PhysX.read / PhysX.write, C ovphysx_read / ovphysx_write); use the ovphysx-session-write skill to write control inputs and state, and the ovphysx-output-read skill (ovstage-native identity-preserving reads and write-back) to read output back. Tensor bindings are kept only for maintaining existing caller-owned bulk-NumPy binding code during the deprecation period.
license: Apache-2.0
compatibility: "ovphysx >=0.6.0 wheel or SDK; Python examples require NumPy, and C examples require SDK headers and libraries."
allowed-tools: Read Shell
metadata:
  version: "0.1.4"
  author: NVIDIA Omniverse Physics
  tags: "ovphysx, physics, tensor-bindings, cpu"
---

# Tensor Bindings: CPU Read and Write

> **Tensor-binding code API deprecated (ovphysx 0.6).** The tensor-binding CODE API is deprecated in
> favor of the session read/write API — Python `PhysX.read` / `PhysX.write`, C `ovphysx_read` /
> `ovphysx_write`. Prefer the session API where it fits (`ovphysx-output-read` covers ovstage-native
> identity-preserving reads and write-back). For writing control inputs and state, use the
> `ovphysx-session-write` skill; tensor bindings are kept only for maintaining existing
> caller-owned bulk-NumPy binding code.

Tensor bindings map physics-object path patterns to typed tensor views, including
authored USD objects and runtime-only clones. This enables bulk data exchange with
NumPy, PyTorch, Warp, or any other DLPack-compatible framework.

## When to Use

Use this skill when a caller needs bulk CPU tensor reads or writes for simulation state, such as poses, velocities, or joint targets, through the public TensorBindingsAPI.

## Instructions

1. Read `docs/tutorials/tensor_bindings.md` and the sample for the caller's language before changing code.
2. Populate an ovstage, advance and wait for its write floor, attach it at that
   ordinal, create bindings once from stable physics-object path patterns, then
   reuse them to read or write tensors with the binding shape and dtype. Inspect
   `binding.native_device` (or the C getter) before allocating when the process
   is not forced to CPU mode.
3. Use Shell to run the Python sample or compile the C sample after adapting the scene path and tensor type.

## Python

```python
from ovphysx import PhysX, codeless_schema_root
from ovphysx.types import TensorType
import numpy as np
import ovstage

PhysX.set_cpu_mode(True)
physx = PhysX()
# Register the codeless PhysX schemas before the first population call.
ovstage.population.register_usd_schemas([str(codeless_schema_root())])
stage = ovstage.Stage("ovphysx-tensors")
ovstage.population.open_usd(stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.attach_ovstage(stage, read_ordinal=1)

# Write a control-input binding for targets you set.
velocity_target_binding = physx.create_tensor_binding(
    pattern="/World/articulation/articulationLink*",
    tensor_type=TensorType.ARTICULATION_DOF_VELOCITY_TARGET,
)

# Use a separate binding for the simulated state you read back.
link_pose_binding = physx.create_tensor_binding(
    pattern="/World/articulation/articulationLink*",
    tensor_type=TensorType.ARTICULATION_LINK_POSE,
)

# Write control inputs
targets = np.zeros(velocity_target_binding.shape, dtype=np.float32)
targets[0, 0] = 25.0  # set first DOF velocity target
velocity_target_binding.write(targets)

# step_sync steps and waits in one call
physx.step_sync(0.01)

# Read simulated state from the pose binding (not the target binding)
link_poses = np.zeros(link_pose_binding.shape, dtype=np.float32)
link_pose_binding.read(link_poses)

# Clean up
velocity_target_binding.destroy()
link_pose_binding.destroy()
physx.detach_ovstage()
stage.destroy()
physx.destroy()
```

Read simulated results from a *state* binding (poses, positions), not from a
*target* binding: a velocity-target binding reads back the control inputs you
wrote, not the physics outcome.

The physics-only `domains` mask above is fine for this skill's non-instanced
sample USD. For arbitrary content prefer `ALL` — see
[Population domains](../../docs/ovstage_integration.md#population-domains).

Full sample:
- `samples/python_samples/tensor_bindings.py` (wheel)
- Source checkout: `tests/python_samples/tensor_bindings.py`

## C

Full sample:
- `samples/c_samples/tensor_bindings_c/main.c` (SDK)
- Source checkout: `tests/c_samples/tensor_bindings_c/main.c`

## Common tensor types

| Constant | Data |
|----------|------|
| `TensorType.RIGID_BODY_POSE` | Rigid body positions + quaternions |
| `TensorType.ARTICULATION_DOF_POSITION` | Joint positions |
| `TensorType.ARTICULATION_DOF_VELOCITY_TARGET` | Joint velocity drive targets |
| `TensorType.ARTICULATION_LINK_POSE` | Articulation link poses |

See `include/ovphysx/ovphysx_types.h` for the full list. In C the same types use
the `OVPHYSX_TENSOR_*_F32` enum spelling (for example Python
`TensorType.RIGID_BODY_POSE` is C `OVPHYSX_TENSOR_RIGID_BODY_POSE_F32`).

## Key APIs

| Python | C |
|--------|---|
| `physx.create_tensor_binding(pattern, tensor_type)` | `ovphysx_create_tensor_binding()` |
| `binding.native_device` | `ovphysx_get_tensor_binding_native_device()` |
| `binding.read(output)` | `ovphysx_read_tensor_binding()` |
| `binding.write(input)` | `ovphysx_write_tensor_binding()` |
| `binding.destroy()` | `ovphysx_destroy_tensor_binding()` |

## Partial updates (RL-style)

TensorBindings supports selectively applying actions without changing the binding:
- **Masked write**: pass a bool/uint8 mask of shape `[N]` (1 = update, 0 = keep old value).
  - Python: `binding.write(tensor, mask=mask)`
  - C: `ovphysx_write_tensor_binding_masked()`
- **Indexed write**: pass an int32 index tensor of shape `[K]` (rows to update).
  - Python: `binding.write(tensor, indices=indices)`
  - C: `ovphysx_write_tensor_binding(handle, binding_handle, &src_tensor, &index_tensor)`

## References

- Docs: `docs/tutorials/tensor_bindings.md`
- Python sample: `samples/python_samples/tensor_bindings.py` (wheel; source: `tests/python_samples/tensor_bindings.py`)
- C sample: `samples/c_samples/tensor_bindings_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_c/main.c`)
