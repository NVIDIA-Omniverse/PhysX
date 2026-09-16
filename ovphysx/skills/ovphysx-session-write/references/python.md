# Python Session Writes

Read this file for the Python `PhysX.write()` branch.

## The write session

`PhysX.write(object_type, attribute)` returns a context-managed `WriteSession`
carrying exactly ONE attribute. Its `groups` are the mirror of a `ReadResult`:
the same prims in the same order. Each `WriteGroup.tensors[i]` is a `warp.array`
that is a MUTABLE VIEW onto runtime-owned storage -- fill it in place, then hand
the group to `commit()`. Anything not committed when the block exits is
discarded.

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def drive_all_bodies_x(physx, vx: float):
    """Set every rigid body's linear velocity to (vx, 0, 0)."""
    physx.warmup()          # a write before the first step is refused, not warmed
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

`tensor.assign(src)` copies a NumPy / Warp / DLPack source shaped like
`tensor.shape`, staging host->device as needed, so one code path serves CPU and
CUDA. Inspect `tensor.device` to decide the commit stream, not to choose how to
fill.

## Fill exactly, or query fewer

A group has no fill mask. To publish fewer prims, open the session against a
query that selects fewer -- never commit a group with some tensors unfilled. A
mid-fill exception that abandons the `with` block publishes nothing, which is
the intended safety, not a partial write.

For an array object type -- `ARTICULATION_JOINT` carries one tensor per joint --
fill every tensor in the group, not just `tensors[0]`:

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def drive_every_dof_position(physx, degrees: float):
    physx.warmup()
    physx.wait_all()
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
`step_sync()`, then read `jointVelocity` -- through the `ovphysx-output-read`
skill.

## Validation

- Warm up or step before the first write; a pre-step write is refused.
- Every group intended to publish is fully filled and committed; abandoned fills
  publish nothing.
- CUDA groups commit with their Warp stream; inspect `tensor.device` rather than
  assuming the write device equals the read device.
- The test asserts a physical consequence (motion, pose change), not the
  write-only input read back.
- Every `WriteSession` closes (its `with` block exits) before `PhysX.destroy()`.

Worked example (source checkout, not shipped in the wheel):
`tests/python_samples_internal/rigid_body_falling_tensors.py` writes rigid-body
`position`, steps, and reads the settled poses back.
