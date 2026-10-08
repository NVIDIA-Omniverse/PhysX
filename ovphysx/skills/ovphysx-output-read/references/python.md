# Python Output Reads

Read this file for the Python `PhysX.read()` or `PhysX.read_tokens()` branch.

## Open the read

Use semantic strings when the requested columns are known:

Every tensor is a `warp.array`, so callers never branch on frontend type. Residency
is per COLUMN, not per read: on a GPU scene the backend-sourced columns are CUDA
arrays while host-resident ones (rest points, element indices, per-shape properties)
stay on the CPU, so one group can mix the two. A non-empty index map is always a CPU
`warp.array` of `uint32`, whatever the read's device:

```python
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType

def inspect_active_rigid_bodies(physx):
    with physx.read(
        SimObjectType.RIGID_BODY,
        ["position", "orientation"],
        scope=ObjectScope.ACTIVE,
    ) as result:
        for group in result.groups:
            for tensor in group.tensors:
                assert isinstance(tensor, wp.array)
                print(group.prim_list, group.attribute, tensor.device, tensor.shape)
```

On a **DirectGPU** sim, state columns for the covered types (rigid bodies,
articulation links, whole-articulation roots, and joint DOFs) are CUDA Warp
arrays. Whole-articulation per-shape columns and `shapeCount` remain CPU Warp
arrays, so one result can mix devices. On a **CPU** sim all columns are CPU Warp
arrays. Inspect `tensor.device` rather than assuming one residence for a result.

Use the Warp array directly for compute. `tensor.numpy().copy()` makes an independent
host copy for local inspection. If another framework is the destination, use Warp's
supported interoperability path and keep the source array alive as required by
that framework.

### Readiness on a GPU sim

A device column is handed over **before its producing work has necessarily
finished**. `PhysX.read()` orders the group's producer event onto the current
Warp stream for the column's device before returning, without blocking the host.
Select a non-default stream around the read when that is where the first work
will run:

```python
import warp as wp
from ovphysx.types import SimObjectType

def read_positions_on_stream(physx, stream):
    with wp.ScopedStream(stream, sync_exit=True):
        with physx.read(SimObjectType.RIGID_BODY, ["position"]) as result:
            return [group.tensors[0] for group in result.groups if group.tensors]
```

The frontend orders only the stream current during `read()`. Here,
`sync_exit=True` also orders the restored stream before the arrays leave the
scope. Establish an explicit Warp stream dependency before using them on any
other stream. `group.cuda_stream` remains `0`; the read never asks consumers to
drain a producer stream.

`cuda_wait_event` stays exposed for advanced or native consumers. The ordering
call is the C entry point `ovphysx_cuda_stream_wait_event(stream, event)`; normal
Python callers do not call it because the Warp frontend has already issued the
wait.

The event belongs to the read session and stays valid until the session is
released; do not destroy it.

### Step first

Rigid bodies, articulation links, whole articulations, and articulation joint
DOFs require **at least one `step()`** on a DirectGPU sim before they produce
anything: PhysX sizes its GPU structures during that step and rejects direct-GPU
reads until it has run. A pre-step read yields an empty `groups` list for that
scene partition, which is currently indistinguishable from a query that matched
nothing. For whole articulations, the partition also omits the otherwise
CPU-backed per-shape columns so the result cannot look like a complete mixed read.

Use `read_tokens(object_type, attribute_tokens, scope=ObjectScope.ALL)` when the
caller already has interned attribute tokens. Both entry points open the native
query/read sessions and drain all groups before returning the context-managed
result.
Attributes that the selected type does not emit are skipped. A type with no
matching objects yields an empty `groups` list.

## Ownership

Every non-empty tensor, `group.index_map`, and `group.prim_index_map` is a
borrowed Warp array: tensors on the read's device, index maps always on CPU. Each
may outlive the context and keeps the native read-session buffer alive for as long
as the array, its Warp views, or retaining downstream views are referenced. An
empty tensor is a Warp-owned empty array that retains no read session; an empty
index map is `None`.

The following values belong to the attached Stage's shared dictionary and are used
only while `ReadResult` is open:

- `group.prim_list`
- `group.attribute`
- `result.dictionary`

The context manager releases context-bound identity handles and drops its session
reference. Numeric storage is released after the last array/view is gone. Drop
all aliases before calling `PhysX.destroy()`. Destroying first emits a
`ResourceWarning` and leaves that read session allocated so its pointers do not
dangle.

## Scope and Layout

- The rigid-body `ACTIVE` example above is supported. Before using `ACTIVE` for
  another object type, or caching prim order and tensor shapes, read
  [Scope and layout constraints](scope_and_layout.md). In particular, do not
  wait for `group.layout_generation` to change before rebuilding after a known
  structural edit; the current output-read producer reports zero.
- Fixed groups contain one stacked tensor. Array groups contain one tensor per
  prim. A tuple width above 1 (native DLPack `dtype.lanes`) appears as a trailing
  Warp dimension; a single-lane column has none.
- Arrays are mutable snapshots of the read, not live write-through views of
  PhysX simulation state.
- Point-instancer `position` and `orientation` requests emit the attribute
  tokens `positions` and `orientations` in instancer-local space.
