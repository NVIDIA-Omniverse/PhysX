<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-READ-001
title: Warp Output-Read Frontend
status: implemented
owner: ovphysx
---

## Description

Python exposes physics output through one directly usable array type on CPU and
CUDA: `warp.array`. `PhysX.read(query, attributes, *, metadata=False)` takes a
completed `Query` and returns an `Operation` whose result is a `ReadResult`; opening
fetches nothing. Each `ReadResult.fetch_next()` returns an `Operation` whose result is
a `ReadGroup`, or `None` at exhaustion. A group's arrays are created once, on the first
typed result access on the submitting thread. `PhysX.read_sync()` and
`read_tokens_sync()` are explicit waiting adapters that own an internal query and
fetch every group. Operation collection semantics are
[REQ-PYTHON-TASK-001](REQ-PYTHON-TASK-001.md).

Non-empty arrays alias the native read session's snapshot storage. Closing the
`ReadResult` ends the read, and its arrays are invalid afterwards; a result
dropped without a close is retained until the last Warp or downstream framework view
releases it, or the library's process-exit cleanup runs. The native C
exchange descriptor remains DLTensor-based and is not part of the Python result type.
CUDA arrays are ordered after the native producer event on the Warp stream current
when the group is first materialized. This is an asynchronous stream wait, not a host
synchronization; the caller establishes a Warp stream dependency before using the
result on another stream.

## Acceptance Criteria

- AC-1: Every returned tensor is a `warp.array` on the native CPU or CUDA
  device. `index_map` and `prim_index_map`, when present, are CPU
  `warp.array(dtype=wp.uint32)` values whatever the read's device. NumPy is not an
  output-read result type, and the public ovphysx `ManagedDLTensor` wrapper is
  removed rather than kept as an alternative one. Whatever DLPack typing remains
  in Python belongs to the compatibility APIs and is transitional.

- AC-2: DLPack scalar code and bits map to the matching Warp scalar dtype, and a
  `dtype.lanes` above 1 becomes a trailing shape dimension while a single-lane
  column gains none. A tensor's own dimensions are kept, so the single-lane
  inverse-dynamics columns arrive as `[N, R, C]` (`jacobian`), `[N, M, M]`
  (`massMatrix`), `[N, M]` (`coriolisForce`, `gravityForce`) and `[N, 6, numDofs + 7]`
  (`centroidalMomentum`). Unsupported dtype or device fails explicitly. Native output
  columns remain dense with `byte_offset = 0`.

- AC-3: A non-empty array aliases the native pointer without a conversion copy
  and owns one read-session lease, which keeps a result dropped without a close
  alive: its arrays, Warp slices, and downstream DLPack consumers remain valid until
  their last reference is dropped or library exit cleanup runs. Closing the
  `ReadResult` invalidates them (AC-5). Exit handlers registered after `import ovphysx` run before that
  cleanup; handlers registered before import must not use borrowed arrays.
  An empty tensor
  column is a valid Warp-owned empty array that retains no native lease; an empty
  index map is `None`.

- AC-4: When a fetched CUDA group is first materialized, Python enqueues that group's
  non-zero producer event onto the Warp stream then current under the matching device
  context, once per group. Collecting the fetch through `wait_all` or `wait_task` creates
  no array and orders no stream. A private native binding checks the CUDA wait
  status without blocking the host or taking event ownership. If the wait cannot
  be established, Python raises `RuntimeError` before exposing any group arrays.
  No public ovphysx CUDA-wait entry point is exposed. Host groups and zero events
  skip the wait.

- AC-5: `ReadResult.close()` ends the read: the caller no longer needs its data.
  It submits every remaining group release without waiting; the result's arrays and
  their views are invalid afterwards. A read over the caller's query with no host array
  alive is released once all are admitted and no fetch is pending, and the query's next
  read hands out the same device storage. It returns an `Operation` over the group
  releases' real receipts. A rejected group release keeps the read so close can retry; an accepted
  release is never reissued and its failure is reported when collected. A result
  dropped without a close is released after its last alias drops, queued for an
  owning-thread drain that also submits without waiting.
  Default context exit behaves like `close()`; `close_sync()` and a `read_sync` result's
  context exit wait. Materializing a fetch after its parent closed or its attachment was
  invalidated raises and retires the unexposed group. Finalization on another thread
  performs bookkeeping only. A cleanup failure raised on exit is chained without
  replacing an exception already propagating from the context body. Explicit
  `PhysX.destroy()` rejects before teardown while arrays or downstream views that still
  hold their read-session lease are borrowed, preserving their storage and the instance
  until all aliases are dropped.

- AC-6: The wheel declares one bounded, tested Warp feature line as a normal
  dependency, and runtime types, public type stubs, docs, samples, and tests all
  describe the same Warp return contract. Warp's transitive NumPy dependency is
  not presented as a direct NumPy output-read contract. CUDA waits use the bundled
  native runtime and the existing Warp dependency without adding a Python CUDA
  package, as specified by AC-4.

- AC-7: **Selector and metadata ownership.** A default `ReadResult` never owns or
  releases the caller's `Query`; only a `read_sync` result owns its internal query.
  `metadata=True` requests the native open summary and exposes it as an immutable
  `ReadMetadata` of `ReadAttributeResult` rows with `ReadOmission` flags; otherwise
  `metadata` is `None` and reading it never triggers a native call. `read_sync` and
  `read_tokens_sync` open their internal query without a discovery result. A result's
  `dictionary` is an owner-aware `ovstage.PathDictionary` over the attached Stage and
  raises once the attachment is invalidated.

- AC-8: **Stream-ordered and captured reads.** `PhysX.read(..., cuda_stream=0,
  cuda_wait_event=0, event_owner=None)` passes the producer handoff to `ovphysx_read`, as
  `WriteSession.commit` does, and retains `event_owner` until the open is collected
  ([ADR-0055](../../../ovruntime/plc/adr/ADR-0055-capturable-session-reads.md)). A read given
  a non-zero `cuda_stream` performs no deferred read teardown, which may wait on or free device
  memory where a capture of the read cannot; a later call without a stream performs it. A read
  on a stream being captured, after a closed read over the same query with the same
  attributes, records its gathers, and each replay refreshes its arrays. Without
  such a read, holding a host-resident or contact group, on a stream of another CUDA
  context, or on the legacy default stream (the default `cuda_stream=0`) while a blocking
  stream of its context is being captured, it fails when collected and the capture goes on.
  Centre-of-mass and inverse-dynamics arrays replay the gravity of the capture.

- AC-9: **Interned identifiers.** A `ReadGroup`'s `attribute` is an `ovstage.Token` and its
  `prim_list` an `ovstage.PrimPathList`, the handle types the attached Stage's
  `ovstage.PathDictionary` resolves. `WriteGroup`, `ReadAttributeResult`,
  `WriteAttributeResult` and `QueryMetadata` carry the same types. Attribute lists,
  `prim_paths` and contact selections accept these handles or their raw `uint64` value and
  reject other types.

- AC-10: **Flattened single column.** `ReadGroup` keeps its flat fields over the native
  group's one column (REQ-CAPI-SESSION-001 AC-10): `attribute`, `ordinal`, `is_array`,
  `is_delete`, `semantic`, `tensors`, `index_map`, `layout_generation`,
  `write_floor_ordinal`, `cuda_stream` and `cuda_wait_event` come from `columns[0]`, and
  `prim_list`, `prim_offset`, `prim_count` and `prim_index_map` from the group's prims. The
  CUDA stream wait of AC-4 uses that column's event. A native group that does not carry
  exactly one column fails materialization with `RuntimeError` before any array is created.

## Test References

- TEST-PYTHON-READ-001
- [TEST-PYTHON-READ-002](../../tests/python/TEST-PYTHON-READ-002.md)

## Code References

- ovphysx/python/ovphysx/api.py (`ReadGroup`, `ReadResult`, `ReadMetadata`,
  `ReadAttributeResult`, `ReadOmission`, `PhysX.read`, `PhysX.read_sync`,
  `PhysX.read_tokens_sync`, `PhysX._materialize_read_group`, `PhysX._dltensor_to_warp_array`)
- ovphysx/python/ovphysx/_operations.py (`Operation`, first-access materialization)
- ovphysx/python/ovphysx/_bindings.py (private checked CUDA wait binding;
  `ovstage_read_group_t` / `ovstage_read_column_t` mirrors -- AC-10)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_internal_cuda_stream_wait_event`)
- ovphysx/python/ovphysx/api.pyi
- ovphysx/python/pyproject.toml
- ovphysx/tests/python_tests/test_output_read.py
- ovphysx/tests/python_tests/test_output_read_device.py
- ovphysx/tests/python_tests/test_output_read_device_stream.py
- ovphysx/tests/python_tests/cpu_tests/test_output_read_cpu.py
- ovphysx/tests/python_tests/test_output_read_lifetime.py
- ovphysx/tests/python_tests/test_read_capture_gpu.py
- ovphysx/tests/python_samples/rl_step_capture.py (runnable IsaacLab-style captured control step)
- ovphysx/tests/python_tests/test_session_operations.py
- ovphysx/tests/python_tests/test_dlpack_metadata.py
- ovphysx/tests/python_tests/test_type_stubs.py

## Dependencies

- [REQ-CAPI-SESSION-001](../capi/REQ-CAPI-SESSION-001.md)
- [REQ-PYTHON-TASK-001](REQ-PYTHON-TASK-001.md)
- [Read/write data model](../../../docs/read_write/data_model.md)
- [Device residency and borrowed-buffer lifetime](../../../docs/read_write/device.md)
