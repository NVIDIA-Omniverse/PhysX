<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-READ-001
maps_to: REQ-PYTHON-READ-001
type: integration
---

## Scenario

The high-level output-read API is exercised on CPU and CUDA scenes through the
real native runtime. Focused tests cover scalar dtype and lane mapping.

## Given

- CPU and DirectGPU scenes with fixed-width and array-valued output columns.
- Native DLTensor descriptors covering supported and unsupported dtype, device,
  lane, and empty combinations.
- A real non-default Warp CUDA stream when a CUDA device is available.

## When

- `PhysX.read_sync()` and `read_tokens_sync()` drain tensors and index maps.
- A result is dropped without a close while returned arrays and Warp slices remain
  referenced.
- A held query is read three times on a DirectGPU scene, each result closed while its
  array is still referenced.
- A CUDA result is created under a non-default current Warp stream.
- An explicit `query` / `read` / `fetch_next` operation on CUDA is collected by
  `wait_all` under one Warp stream and first accessed under another.
- With native operations replaced by delayed doubles, a fetch is collected and then
  accessed, a fetch is materialized after its parent closed, a group release is
  rejected during close, deferred cleanup is drained, and `metadata=True` is requested.
- The converter encounters an unsupported dtype or device.
- The current stream is Warp's null stream, or a group is host-resident or has no event.
- The native CUDA wait reports a failure when a fetched group is first materialized.
- Runtime dependencies and public type stubs are inspected.
- Pass Python and NumPy booleans as prim handles, contact filters and attribute
  tokens. Also pass a zero-dimensional integer array as a prim handle or contact filter.

## Then

- CPU and CUDA tensors and maps use the Warp types, dtype, shape, pointer, and
  device required by REQ AC-1 and AC-2.
- Non-empty arrays share the native pointer and keep a result dropped without a close
  usable; empty arrays do not retain a native lease (REQ AC-3).
- The real or recorded current Warp stream waits once on the matching producer
  event without a host-side synchronization. For the explicit fetch, collection
  issues no stream wait and the only wait targets the stream current at first
  result access (REQ AC-4).
- The private native wait receives the borrowed event and current Warp stream
  handles under the matching device context, once per group. It does not own or
  destroy the native event. Warp's null stream is also accepted. Host groups and
  zero events skip the wait. A failed native wait raises `RuntimeError` before
  any group array is exposed (REQ AC-4).
- Close submits group releases, then the read's release, without waiting, while the
  closed read's array is still referenced, and each read of the held query hands out
  the same pointer; a rejected release keeps the read for retry; deferred drains submit
  without waiting; a fetch materialized after its parent closed raises and retires its
  group; context-body failures remain primary (REQ AC-5). **Foreign-thread finalization is not covered here**: the two
  threaded cases drive `release_borrow` directly against a stand-in holder rather than
  letting a real Warp deleter run on another thread, so the deferral path itself is
  asserted only on the owning thread.
- Package metadata and public stubs require and expose the bounded Warp frontend
  without a direct NumPy read-result declaration or an added Python CUDA package
  dependency (REQ AC-6).
- A default session leaves the caller's query open and reports `metadata is None`;
  `metadata=True` yields an immutable copied snapshot; the waiting adapters request no
  internal query summary (REQ AC-7).
- A real read's groups carry an `ovstage.Token` attribute and an `ovstage.PrimPathList`
  prim list; metadata rows carry `ovstage.Token` values. Attribute lists, `read_tokens_sync`,
  `write_sync` and contact selections accept an `ovstage` handle or its raw value, and
  a token of the wrong handle kind, `bool`, 0 or `2**64 - 1` is rejected (REQ AC-9).
- Booleans and integer arrays fail before native submission; Python and NumPy
  integer scalar prim handles remain accepted, including with NumPy 2.2.6 (REQ AC-9).
- A native group with one column materializes a `ReadGroup` whose per-attribute fields,
  tensors and CUDA handoff equal that column's and whose prim fields equal the group's,
  and the Warp stream wait is issued for that column; a native group without a column
  raises `RuntimeError` before any wait or array (REQ AC-10).

## Test location

`ovphysx/tests/python_tests/test_output_read.py` (AC-1..AC-3, AC-6),
`test_output_read_device.py` and `test_output_read_device_stream.py` (AC-4, the CUDA
event handoff), `test_output_read_lifetime.py` (AC-5, borrow and session lifetime), and
`test_type_stubs.py` for the stub surface AC-6 names. Its
`test_wheel_declares_warp_and_not_numpy` checks that the wheel adds no
`cuda-bindings` or `cuda-python` dependency.

`cpu_tests/test_session_properties.py` (`test_jacobian_dimensions_fixed_base`,
`test_mass_matrix_dimension_fixed_base`) and `test_session_properties_gpu.py`
(`test_jacobian_readable`) read the inverse-dynamics groups through `PhysX.read` and
check that the arrays keep their `[rows, cols]` and `[M, M]` row dimensions (AC-2).

`test_session_tasks.py` additionally checks group-release rejection retry,
accepted-release failure collection, deferred-drain progress after a rejection,
and context-body exception preservation with mocked native operations (AC-5).

`test_output_read_device_stream.py::test_async_fetch_materializes_on_first_result_stream_not_collection_stream`
checks the explicit-fetch stream rule on a real CUDA device (AC-4).
`test_output_read_lifetime.py::test_closing_a_read_releases_it_while_its_arrays_are_held`
checks the close and pointer reuse on a real CUDA device (AC-5).
`test_output_read_lifetime.py::test_failed_stream_wait_raises_before_any_array_is_returned`
checks that a failed native wait raises before tensor conversion (AC-4).
`test_output_read_device_stream.py::test_private_wait_zero_event_is_noop`
checks that a zero event succeeds without consulting CUDA (AC-4).
`test_session_operations.py` checks the delayed-binding cases with native operations
replaced: `test_read_fetch_materializes_once_at_first_result_access` (AC-4),
`test_fetch_materialization_after_parent_close_fails_and_retires_group`,
`test_close_rejected_group_retains_owner_for_retry` and
`test_deferred_cleanup_drain_submits_without_waiting` (AC-5),
`test_default_session_open_neither_fetches_nor_collects`,
`test_opt_in_metadata_is_copied_immutable_snapshot` and
`test_sync_convenience_omits_internal_query_summary` (AC-7). These doubles verify Python
ownership and ordering, not a native scheduler. The handle types (AC-9) are checked by
`test_output_read.py` on a real read and, with native operations replaced, by
`test_session_operations.py`: `test_opt_in_metadata_is_copied_immutable_snapshot`,
`test_write_sync_accepts_a_single_name_or_token`,
`test_path_selections_accept_handles_and_numpy_integers`,
`test_path_selections_reject_non_integers_and_other_handle_kinds`,
`test_boolean_empty_and_invalid_values_are_not_attribute_tokens` and
`test_contact_query_collects_output_and_optional_metadata`. The flattened single column
(AC-10) is checked by `test_read_group_flattens_its_single_native_column` and
`test_read_group_without_exactly_one_column_fails_materialization`.

Evidence status: an earlier revision of the CUDA stream case was reported passing on an
installed runtime; no result for the current revision of it or of the delayed-binding
cases is recorded in this specification.

`lifecycle_tests/test_read_borrow_destroy_cpu.py` and
`lifecycle_tests/test_read_borrow_destroy_gpu.py` verify explicit instance
destruction rejects while a real borrowed Warp view of a result dropped without a
close is alive, preserves its
storage and the instance, and succeeds after the last alias is dropped (AC-5).
