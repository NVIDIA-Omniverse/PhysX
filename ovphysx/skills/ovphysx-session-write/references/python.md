# Python Session Writes

Read this file for the Python `PhysX.write_sync()` / `PhysX.write()` branch.

## The write session

`PhysX.write_sync(object_type, attributes)` creates its own query, opens a
context-managed `WriteSession` for one or more attributes (a single name is also
accepted), fetches every group into `groups`, and waits for its releases when the
`with` block exits. The default `PhysX.write(query, attributes)` instead returns an
`Operation` for the session open over an explicit, completed `Query`; the session's
`fetch_next()`, `commit()` and `close()` return operations too, and `wait()` collects
each one.

Identify mapped objects through each write group's own prim list; read and write
partitions and fetch order need not match. Each `WriteGroup.tensors[i]` is a `warp.array` that is a MUTABLE
VIEW onto runtime-owned storage -- fill it in place, then hand the group to
`commit()` / `commit_sync()`. `WriteGroup.attribute_index` is the group's position
in the request list and `WriteGroup.attribute` its canonical token. Anything not
committed when the session closes is discarded.

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def drive_all_bodies_x(physx, vx: float):
    """Set every rigid body's linear velocity to (vx, 0, 0)."""
    physx.warmup()          # portable: DirectGPU refuses a pre-step write; CPU/GPU apply it
    physx.wait_all()
    committed = 0
    with physx.write_sync(SimObjectType.RIGID_BODY, ["linearVelocity"]) as w:
        for g in w.groups:
            # Fill EVERY tensor of the group: commit publishes the whole group,
            # so an unfilled tensor drives its prims with stale memory.
            device = None
            for tensor in g.tensors:
                device = tensor.device
                assert device is not None
                buf = np.zeros(tuple(tensor.shape), dtype=np.float32)
                buf[:, 0] = vx
                tensor.assign(buf)          # numpy source, host or device tensor
            if device is not None and device.is_cuda:
                # The device scope makes the fill's CUDA context current for the value 1.
                with wp.ScopedDevice(device):
                    stream = wp.get_stream(device)
                    w.commit_sync(g, cuda_stream=int(stream.cuda_stream or 1))
            else:
                w.commit_sync(g)
            committed += 1
    assert committed > 0, "write produced no groups; nothing was driven"
```

The default rigid-body query includes kinematic bodies. On CPU or GPU-with-readback
scenes, PhysX rejects this velocity setter on standalone kinematic bodies, and the
commit raises `TaskError` with the SDK error -- from `commit_sync()`, or from `wait()`
on a default `commit()`. Valid dynamic rows in the same group can still be written,
including rows after the rejected body; the kinematic body's velocity stays
unchanged. The failed group is spent, with no rollback or applied/rejected row count.
There is no kinematic-body filter; to write only some bodies, select their prims
with `prim_paths=` or `pattern=` on `write_sync()` or `query()`.
A selected write includes sleeping members and uses `scope=ALL` (the default).

`tensor.assign(src)` copies a NumPy / Warp / DLPack source shaped like
`tensor.shape`, staging host->device as needed, so one code path serves CPU and
CUDA. Inspect `tensor.device` to decide the commit stream, not to choose how to
fill.

The commit stream and event name your PRODUCER work; they do not select the
stream PhysX consumes on. Both are honored when both are nonzero, and only two
zeros assert that nothing is outstanding. A Warp stream's `cuda_stream` is `None`
for a device's null stream, which the examples pass as the legacy default stream `1`;
that value refers to the CUDA context current during the commit, which is why the
commit runs inside `wp.ScopedDevice`. The current native implementation can block
the host while it completes these dependencies, so an `Operation` returned
without collection is not a promise of host-nonblocking execution. A failed
handoff fails the commit task and spends the group. During the wait, the public
ovphysx API rejects overlapping query/read/write/fetch/commit/release submissions
with `OVPHYSX_API_INVALID_STATE`, surfaced as a Python `RuntimeError`. Producer
work must not depend on code (such as a CUDA host callback) that needs one of those
calls to succeed or waits for this commit to finish -- that cyclic dependency can
deadlock. Keep public API calls serialized.

## Fill every mapped entry

A group has no fill mask; never commit a group with some tensors unfilled. A
mid-fill exception that abandons the `with` block discards the uncommitted group.
Previously committed groups remain applied: commits are not transactional, and they
run in the order you issue them.

For an array object type -- `ARTICULATION_JOINT` carries one tensor per joint --
fill every tensor in the group, not just `tensors[0]`. One session can carry several
attributes; identify each group's attribute by `attribute_index`, never by shape.
This example uses the default operations with an explicit query:

```python
import numpy as np
import warp as wp
from ovphysx.types import SimObjectType


def set_every_dof_state(physx, degrees: float, degrees_per_second: float):
    physx.warmup()
    physx.wait_all()
    values = (degrees, degrees_per_second)  # indexed like the request list below
    with physx.query(SimObjectType.ARTICULATION_JOINT).wait() as query:
        with physx.write(query, ["jointPosition", "jointVelocity"]).wait() as w:
            while (g := w.fetch_next().wait()) is not None:
                # Angular DOF columns are DEGREES, per axis -- no radian conversion.
                # Both attributes have the same shape; attribute_index says which one this is.
                device = None
                for tensor in g.tensors:
                    device = tensor.device
                    assert device is not None
                    tensor.assign(np.full(tuple(tensor.shape), values[g.attribute_index], dtype=np.float32))
                # Commit with the group's stream on a GPU scene, as in drive_all_bodies_x.
                if device is not None and device.is_cuda:
                    with wp.ScopedDevice(device):
                        stream = wp.get_stream(device)
                        w.commit(g, cuda_stream=int(stream.cuda_stream or 1)).wait()
                else:
                    w.commit(g).wait()
    physx.wait_all()  # collect the releases submitted when the blocks exited
```

Opening the session validates every requested name first: an unsupported,
read-only or duplicate attribute fails the open and exposes no group. A token in
the list must come from the attached Stage's dictionary.

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

- Warm up or step before the first write for a recipe that works on DirectGPU too.
  On CPU and GPU-with-readback a pre-step write is applied; on DirectGPU it is
  refused. The write never auto-warms.
- Every group intended to publish is fully filled and committed; abandoned fills
  publish nothing.
- Each group's attribute is identified by `attribute_index`, never by tensor
  shape or fetch order.
- CUDA groups commit with their Warp stream; inspect `tensor.device` rather than
  assuming the write device equals the read device.
- The test asserts a physical consequence (motion, pose change), not the
  write-only input read back.
- Every `WriteSession` and `Query` closes before `PhysX.destroy()`; with the
  default operations, collect the submitted releases with `wait_all()`.

Worked example: `samples/python_samples/session_write.py` in the installed
wheel (`tests/python_samples/session_write.py` in a source checkout)
writes `linearVelocity` through a session, commits it, steps, and verifies the
consequence by reading `position` back -- the pattern above, end to end.
