# Python Output Reads

Read this file for the Python branch: the waiting `PhysX.read_sync()` /
`PhysX.read_tokens_sync()` conveniences, or the default `PhysX.query()` +
`PhysX.read()` operations.

## Open the read

`physx.read_sync(object_type, attribute_names, scope=...)` creates its own query,
waits for the session open and every fetch, and returns a context-managed
`ReadResult` whose `groups` are already populated. Leaving the `with` block waits
for the releases it submits. Read storage that arrays still borrow is released only
after the result, its `groups` and every array alias are dropped; a later
`physx.wait_all()` submits and collects that release.

Every tensor is a `warp.array`, so callers never branch on frontend type. Residency
is per COLUMN, not per read: on a GPU scene the backend-sourced columns are CUDA
arrays while host-resident ones (rest points, element indices, per-shape properties)
stay on the CPU, so one result can mix the two. A non-empty index map is always a CPU
`warp.array` of `uint32`, whatever the read's device:

```python
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType

def inspect_active_rigid_bodies(physx):
    with physx.read_sync(
        SimObjectType.RIGID_BODY,
        ["position", "orientation"],
        scope=ObjectScope.ACTIVE,
    ) as result:
        for group in result.groups:
            for tensor in group.tensors:
                assert isinstance(tensor, wp.array)
                print(group.prim_list, group.attribute, tensor.device, tensor.shape)
```

The default operations let the caller choose where outcomes are collected.
`physx.query(...)` returns an `Operation` whose result is a reusable `Query`;
`physx.read(query, attributes)` returns an `Operation` whose result is a
`ReadResult` that has fetched nothing yet; each `result.fetch_next()` returns an
`Operation` whose result is one `ReadGroup`, or `None` at the end. `wait()` collects
an operation and returns its result, raising `TaskError` for a failure. Leaving a
`with` block submits the release without waiting; `physx.wait_all()` collects it.
The helper below keeps only host copies, so its result and arrays are dropped when
it returns, before `wait_all()` submits the queued read release. Pass
`metadata=True` to receive the open summary as `result.metadata`:

```python
from ovphysx import ReadOmission
from ovphysx.types import ObjectScope, SimObjectType

def copy_positions(physx, query):
    positions = []
    with physx.read(query, ["position"], metadata=True).wait() as result:
        summary = result.metadata
        assert summary is not None  # requested with metadata=True
        if summary.attributes[0].omission_reasons & ReadOmission.NOT_READY:
            print("some selected bodies become readable after a later step")
        while (group := result.fetch_next().wait()) is not None:
            positions.extend(tensor.numpy().copy() for tensor in group.tensors)
    return positions

def read_positions_with_summary(physx):
    with physx.query(SimObjectType.RIGID_BODY, scope=ObjectScope.ALL).wait() as query:
        positions = copy_positions(physx, query)
    physx.wait_all()
    return positions
```

`result.metadata.attributes` has one row per requested attribute, in request order:
the canonical `attribute` token (`0` for a name the type cannot read), the
`group_count`, and `omission_reasons`, a `ReadOmission` flag of
`UNSUPPORTED_ATTRIBUTE`, `UNSUPPORTED_CONFIGURATION` and `NOT_READY`. No flag with no
groups is a valid empty answer. `metadata` is `None` unless requested, and reading it
never triggers native work. `query.metadata` is the creation-time discovery snapshot
-- the matched prim count and the tokens of the attributes the type can read, not a
promise that each one emits a group; `physx.query()` requests it by default.

On a **DirectGPU** sim, simulated pose and velocity columns for the covered types
(rigid bodies, articulation links, whole-articulation roots, and joint DOFs) are
CUDA Warp arrays. Authored properties, per-shape columns, `shapeCount`, and
`isSleeping` remain CPU Warp arrays, so one result can mix devices. `isSleeping`
reports 0 from the sleeping-disabled scene flag. On a **CPU** sim all columns are
CPU Warp arrays. Inspect `tensor.device` rather than assuming one residence for a result.

Use the Warp array directly for compute. `tensor.numpy().copy()` makes an independent
host copy for local inspection. If another framework is the destination, use Warp's
supported interoperability path and keep the source array alive as required by
that framework.

### Readiness on a GPU sim

A device column is handed over **before its producing work has necessarily
finished**. Building a group's arrays orders the group's producer event onto the
Warp stream current at that moment, without blocking the host. `read_sync()` builds
them while it fetches; the default path builds them at the first `wait()` or
`result` access of each `fetch_next()` operation. `wait_all()` never builds arrays.
Select a non-default stream around that point when that is where the first work
will run:

```python
import warp as wp
from ovphysx.types import SimObjectType

def read_positions_on_stream(physx, stream):
    with wp.ScopedStream(stream, sync_exit=True):
        with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
            return [group.tensors[0] for group in result.groups if group.tensors]
```

The frontend orders only the stream current when the arrays are built. Here,
`sync_exit=True` also orders the restored stream before the arrays leave the
scope. Establish an explicit Warp stream dependency before using them on any
other stream. `group.cuda_stream` remains `0`; the read never asks consumers to
drain a producer stream.

`cuda_wait_event` stays exposed for advanced or native consumers. The frontend
uses the bundled native runtime to check the wait on the current Warp stream
without adding a Python CUDA package. Host groups and zero events skip the wait.
A failed wait raises `RuntimeError` before the arrays are exposed. Normal Python
callers need no extra wait on the stream where the arrays were built. Native
consumers call CUDA directly.

The event belongs to the read session and stays valid until the session is
released; do not destroy it.

### Step first

Rigid bodies, articulation links, whole articulations, and articulation joint
DOFs require **at least one `step()`** on a DirectGPU sim before they produce
anything: PhysX sizes its GPU structures during that step and rejects direct-GPU
reads until it has run. A pre-step read yields no groups for that scene partition;
with `metadata=True` the requested rows report `ReadOmission.NOT_READY`, which
tells it apart from a query that matched nothing. For whole articulations, the
partition also omits the otherwise CPU-backed per-shape columns so the result
cannot look like a complete mixed read.

Use `read_tokens_sync(object_type, attribute_tokens, scope=ObjectScope.ALL)`, or
pass tokens in the attribute list of `read()`, when the caller already has interned
attribute tokens from the attached Stage's dictionary, for example
`query.metadata.attributes` or an emitted `ReadGroup.attribute`. A token takes
precedence over a name, a token that does not resolve fails the read, and so does a
duplicate request. Attributes that the selected type does not emit are skipped.
A type with no matching objects yields an empty `groups` list.

## Ownership

Every non-empty tensor, `group.index_map`, and `group.prim_index_map` is a
borrowed Warp array: tensors on the read's device, index maps always on CPU. Each
may outlive the context and keeps the native read-session buffer alive for as long
as the array, its Warp views, or retaining downstream views are referenced. An
empty tensor is a Warp-owned empty array that retains no read session; an empty
index map is `None`.

`group.prim_list` and `group.attribute` belong to the attached Stage's path
dictionary and are used only while the `ReadResult` is open and its attachment remains
live. Detach retires this metadata even when numeric arrays survive. Each is an **opaque
interned integer**, not a path or a name: `prim_list` is a prim-path-list handle and
`attribute` is an attribute token. Resolve them with `result.dictionary` (or
`query.dictionary`), an `ovstage.PathDictionary` for the attached Stage, so nothing
is rebuilt:

```python
from ovphysx.types import ObjectScope, SimObjectType

def print_prim_paths(physx):
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        paths = result.dictionary
        for group in result.groups:
            prim_paths = paths.get_path_strings(group.prim_list)
            attribute = paths.token_to_string(group.attribute)     # 'position' (the EMITTED name)
            print(attribute, prim_paths)  # row i of each tensor belongs to prim_paths[i]
```

`group.attribute` is the *emitted* token and may differ from the requested name (a
`"position"` request on a point-instancer emits `"positions"`), so resolve it rather than
assuming the requested string. `group.prim_list` also feeds
`stage.query_from_path_list(group.prim_list)` directly for a no-repack write-back. See
`tests/python_samples/output_read.py` for the same pattern end to end.

Closing the result releases context-bound identity handles and drops its session
reference. The caller's `Query` is not owned by a `ReadResult` and stays open until
closed; a session opened from it stays valid after the query is closed. Numeric
storage is released after the last array/view is gone. Drop all aliases before
calling `PhysX.destroy()`. With live aliases, destruction raises `RuntimeError`
before teardown and leaves the instance and arrays usable. Drop the aliases, then
retry destruction.

## Scope and Layout

- The rigid-body `ACTIVE` example above is supported. Before using `ACTIVE` for
  another object type, or caching prim order and tensor shapes, read
  [Scope and layout constraints](scope_and_layout.md). In particular, do not
  wait for `group.layout_generation` to change before rebuilding after a known
  structural edit; the current output-read producer reports zero.
- A whole-type query is evaluated again at every session open; reuse it across
  steps, but reopen the read each frame for fresh values. For rigid bodies,
  `ACTIVE` uses the latest active-actor report where available, or current awake,
  non-kinematic state otherwise; other types follow
  the per-type rules in [Scope and layout constraints](scope_and_layout.md).
- Fixed groups contain one stacked tensor. Array groups contain one tensor per
  prim. A tuple width above 1 (native DLPack `dtype.lanes`) appears as a trailing
  Warp dimension; a single-lane column has none.
- Arrays are mutable snapshots of the read, not live write-through views of
  PhysX simulation state.
- Point-instancer `position` and `orientation` requests emit the attribute
  tokens `positions` and `orientations` in instancer-local space.
