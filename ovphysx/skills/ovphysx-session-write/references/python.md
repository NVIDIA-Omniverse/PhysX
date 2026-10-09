# Python Session Writes

Read this file for the Python `PhysX.write()` branch.

Before running either helper, attach a populated `ovstage.Stage` to `PhysX`
using [Basic Workflow](../../basic-workflow/SKILL.md). Keep both alive until
the helper returns. DirectGPU means GPU simulation with
`/physics/suppressReadback` enabled; it needs warmup before the first write.

## The write session

`PhysX.write(object_type, attribute)` returns a context-managed `WriteSession`
carrying exactly ONE attribute. Its `groups` are the mirror of a `ReadResult`:
the same prims in the same order. Each `WriteGroup.tensors[i]` is a `warp.array`
that is a MUTABLE VIEW onto runtime-owned storage -- fill it in place, then hand
the group to `commit()`. Anything not committed when the block exits is
discarded.

Create `examples/session_write.py` with this helper and its imports. Call
`drive_all_bodies_x(physx, 2.0)` from your attached simulation before its next
step. Use a scene whose queried rigid bodies are dynamic; the kinematic-body
limitation is described after the example.

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def drive_all_bodies_x(physx, vx: float):
    """Set every rigid body's linear velocity to (vx, 0, 0)."""
    physx.warmup()          # portable: DirectGPU refuses a pre-step write; CPU/GPU apply it
    physx.wait_all()
    committed = 0
    with physx.write(SimObjectType.RIGID_BODY, "linearVelocity") as w:
        for g in w.groups:
            # Fill EVERY tensor of the group: commit publishes the whole group,
            # so an unfilled tensor drives its prims with stale memory.
            for tensor in g.tensors:
                buf = np.zeros(tuple(tensor.shape), dtype=np.float32)
                buf[:, 0] = vx
                tensor.assign(buf)          # numpy source, host or device tensor
            if g.tensors and g.tensors[0].device.is_cuda:
                stream = wp.get_stream(g.tensors[0].device)
                w.commit(g, cuda_stream=int(stream.cuda_stream or 1))
            else:
                w.commit(g)
            committed += 1
    assert committed > 0, "write produced no groups; nothing was driven"
    physx.wait_all()
```

The default rigid-body query includes kinematic bodies. On CPU or GPU-with-readback
scenes, PhysX rejects this velocity setter on standalone kinematic bodies, and
commit raises with the SDK error. Valid dynamic rows in the same group can still be
written, including rows after the rejected body; the kinematic body's velocity stays
unchanged. The failed group is spent, with no rollback or applied/rejected row count.
There is no per-body selection or kinematic-body filter.

`tensor.assign(src)` copies a NumPy / Warp / DLPack source shaped like
`tensor.shape`, staging host->device as needed, so one code path serves CPU and
CUDA. Inspect `tensor.device` to decide the commit stream, not to choose how to
fill.

## Fill every mapped entry

A group has no fill mask; never commit a group with some tensors unfilled. A
mid-fill exception that abandons the `with` block discards the uncommitted group.
Previously committed groups remain applied.

For an array object type, `ARTICULATION_JOINT` carries one tensor per joint.
Fill every tensor in the group, not just `tensors[0]`.

Add this helper to `examples/session_write.py` alongside `drive_all_bodies_x`.
Use it only when **every selected joint axis is angular**, for example in an
all-revolute articulation. Each degree of freedom (DOF) is one independently
movable joint axis. This helper sets joint state directly; it does not set a
drive target.

A prismatic coordinate uses the stage's base length unit, not degrees. For mixed
angular and prismatic joints, build per-axis values from your authored joint
metadata and match them to the returned joint order. The type query cannot
filter to angular joints. Do not call this uniform-degrees helper for that scene.
The unit rules are listed in [Shared Contracts](../SKILL.md#shared-contracts).

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def set_all_angular_joint_positions(physx, degrees: float):
    """Set joint state in degrees; requires an all-angular articulation scene."""
    physx.warmup()
    physx.wait_all()
    committed = 0
    with physx.write(SimObjectType.ARTICULATION_JOINT, "jointPosition") as w:
        for g in w.groups:
            # Angular DOF columns are DEGREES, per axis -- no radian conversion.
            for tensor in g.tensors:
                tensor.assign(np.full(tuple(tensor.shape), degrees, dtype=np.float32))
            # Commit with the group's stream on a GPU scene, as in drive_all_bodies_x.
            if g.tensors and g.tensors[0].device.is_cuda:
                w.commit(g, cuda_stream=int(wp.get_stream(g.tensors[0].device).cuda_stream or 1))
            else:
                w.commit(g)
            committed += 1
    assert committed > 0, "write produced no groups; no angular joint was set"
```

## Forces and wrenches

`force` is a vec3 applied at the centre of mass. `wrench` is nine wide,
`[fx,fy,fz, tx,ty,tz, px,py,pz]` -- force, torque, and a WORLD application
point. Use `wrench` for a load away from the COM (force at the point, torque
zero) or a pure torque (force zero, torque set). Both are WRITE-ONLY: the solver
consumes and clears them each step, so read back the CONSEQUENCE (a state
column), never the load itself.

## Read the outcome from a state column

A control-target write reads back the target you wrote, not the physics result.
After stepping, read a STATE attribute -- for example write `jointVelocityTarget`,
`step_sync()`, then read `jointVelocity` -- through the [Output Read](../../ovphysx-output-read/SKILL.md)
skill.

## Input and Group Contracts

The [Python API Reference](../../../docs/python_api.rst) defines the full API.
The fields used by these helpers have this contract:

| Field | Type | Required | Valid Values and Meaning |
|---|---|---|---|
| `object_type` | `SimObjectType` | Yes | One simulated type, such as `RIGID_BODY` or `ARTICULATION_JOINT`. |
| `attribute_name` | `str` | Yes | One attribute writable for that type; refer to [Writable Attributes](../SKILL.md#writable-attributes-by-object-type). |
| `scope` | `ObjectScope` | No | `ALL` by default or `ACTIVE`, subject to [Scope Constraints](../../ovphysx-output-read/references/scope_and_layout.md). Neither filters by joint axis or individual body. |
| `WriteSession.groups` | `list[WriteGroup]` | Returned | May be empty. Fill and commit each intended group once. |
| `WriteGroup.prim_list` | `int` | Returned | Opaque prim-list handle in the attached Stage dictionary, valid while the session is open. |
| `WriteGroup.prim_offset`, `prim_count` | `int` | Returned | Offset and row count in the group's prim list. Use the returned mapping. |
| `WriteGroup.tensors` | `list[warp.array]` | Returned | Fixed group: one stacked tensor. Array group: one tensor per prim. Preserve shape, dtype, and device; fill every entry before commit. |
| `cuda_stream`, `cuda_wait_event` | `int` | No | Commit synchronization handles, default `0`. `cuda_stream=0` means no synchronization; `1` denotes the default stream. Pass the stream used for a CUDA fill. |

A non-empty write tensor is valid only until its group commits or its session
closes. Do not retain or use it afterward. Unlike read arrays, write arrays do
not extend the session lifetime.

## Validation

- Warm up or step before the first write for a recipe that works on DirectGPU too.
  On CPU and GPU-with-readback a pre-step write is applied; on DirectGPU it is
  refused. The write never auto-warms.
- Every group intended to publish is fully filled and committed; abandoned fills
  publish nothing.
- CUDA groups commit with their Warp stream; inspect `tensor.device` rather than
  assuming the write device equals the read device.
- The test asserts a physical consequence (motion, pose change), not the
  write-only input read back.
- Every `WriteSession` closes (its `with` block exits) before `PhysX.destroy()`.

Worked example: `samples/python_samples/session_write.py` inside the installed
`ovphysx` package opens a `PhysX.write()` session for `linearVelocity`, commits it, steps, and
verifies the consequence by reading `position` back -- the pattern above,
end to end.
