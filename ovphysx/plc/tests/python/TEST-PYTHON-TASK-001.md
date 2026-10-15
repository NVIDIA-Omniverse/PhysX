<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-TASK-001
maps_to: REQ-PYTHON-TASK-001
type: integration
---

## Scenario

Python Task values preserve native identity and collected exceptions retain their
messages after native result cleanup. Session operations submit without hidden waits,
cache collected outcomes, convert payloads once on first access, and retain borrowed
storage until collection.

## Given

- Native successful and failed tasks from a CPU instance with a loaded stage.
- Mock native results for deterministic partial timeouts and translation errors.
- The installed package's public Task and task exception exports.
- Mock session outputs published only during wait, with independent admission and
  execution failures for session operations and cleanup.
- A delayed native test double that publishes outputs only when a wait collects them
  and records every submission and wait. It verifies binding ownership and ordering,
  not a native scheduler.

## When

- Obtain receipts from step, clone, attach, update and detach; attempt to mutate a receipt.
- Wait individually and in aggregate, including polling and a mocked partial
  timeout containing one completed failure and one pending task.
- Pass negative and oversized timeouts, plus zero, finite, maximum and None values.
- Pass non-Task arguments and invalid receipts; retain raised exceptions while
  later waits and instance destruction run.
- Inject an error while translating a native result.
- Submit query, read and write operations and inspect submissions, waits and
  `result` before and after collection; build operations with zero, one and several
  receipts.
- Collect operations through `wait_all`, `wait_task` and a partial-timeout
  `wait_all`, including a failed operation whose wait result is then destroyed.
- Drop creator operations and multi-task operations holding inputs before collection.
- Open sessions through the waiting adapters to null-group EOF; reject or fail opening,
  fetching, committing and releasing resources; retry rejected cleanup and drain
  several deferred holders when the first release is rejected.
- Drop a query or write session, or materialize a fetched read group after closing its
  session; fail the accepted implicit release, then collect through `wait_all`.
- Run articulation updates with default and explicit flags/scopes, and with a
  `prim_paths=` or `pattern=` selection naming one of two articulations.

## Then

- Task is immutable and owns no simulation resource (REQ AC-1).
- Successful waits return None; non-Task values raise TypeError and native
  validation/internal errors remain ordinary runtime errors. Out-of-range timeouts
  raise ValueError before native calls; valid values retain their timeout semantics
  (REQ AC-2).
- TaskError and TaskTimeoutError contain immutable copied `(Task, message)`
  entries, with pending_task on timeout and timeout precedence (REQ AC-3).
- Native cleanup runs even when translation raises, and retained messages survive
  later waits and teardown (REQ AC-4).
- Producers distinguish rejection from accepted execution failure (REQ AC-5).
- Default operations issue no wait and raise from `result` until collected; a
  default session open neither fetches nor collects; `tasks` holds exactly the real
  receipts and `task` rejects zero or several; the waiting adapters collect every
  task they submit, including EOF (REQ AC-6).
- Cleanup ownership transfers only on acceptance, accepted failures are not retried,
  rejected holders survive a drain, deferred drains and default releases issue no
  wait, and cleanup errors preserve the original cause. Discarding an implicit release
  receipt does not discard its native failure: `wait_all` reports it once (REQ AC-7).
- `wait_all` records outcomes without converting payloads; a later `result` converts
  once; `wait_task` collects only its target; a failed outcome raises again after its
  native wait result is destroyed; a partial timeout marks only the collected prefix;
  a cached conversion failure keeps no traceback reference to the unexposed array
  (REQ AC-9).
- Inputs of a dropped operation stay alive until every outcome is collected, and a
  dropped creator's output is released after collection without materialization
  (REQ AC-10).
- The articulation convenience waits query, update, and release tasks and exposes
  POSITION/VELOCITY flags with ALL as the default scope; a selection updates only the
  articulations it names (REQ AC-8).

- Stage lifecycle producers return without collecting. Dropped attach/detach
  receipts retain the Stage through pending waits and release it only after
  collection confirms native detachment; a failed query retains it (REQ AC-5, AC-7).

## Automated coverage

- [test_task_completion.py](../../../tests/python_tests/test_task_completion.py)
  uses mocked native functions for
  `test_task_is_an_immutable_native_receipt_copy`,
  `test_wait_task_rejects_non_task_before_calling_native`,
  `test_wait_rejects_out_of_range_timeout_before_calling_native`,
  `test_wait_preserves_timeout_boundaries`,
  `test_partial_timeout_copies_errors_and_pending_before_cleanup`,
  `test_wait_validation_and_internal_failures_are_runtime_errors`,
  `test_error_copy_failure_still_destroys_native_result`,
  `test_task_error_entries_are_immutable` and
  `test_detach_retains_stage_until_collected_and_native_detach_is_confirmed`. The timeout case
  verifies both wait entrypoints, precedence, copied values and pending-task
  retry without depending on native scheduling (REQ AC-1 through AC-5).
- [cpu_tests/test_task_completion.py](../../../tests/python_tests/cpu_tests/test_task_completion.py)
  exercises the real runtime with
  `test_failed_inline_clone_returns_task_and_survives_waiting_for_step`,
  `test_discarded_receipts_and_multiple_errors_remain_in_submission_order`,
  `test_direct_wait_collects_only_its_failure`,
  `test_successful_inline_clone_returns_immutable_task`,
  `test_wrong_instance_does_not_collect_task`,
  `test_invalid_receipts_are_runtime_errors`,
  `test_rejected_step_does_not_create_a_deferred_failure`,
  `test_sync_step_retires_execution_without_collecting_async_receipts` and
  `test_inline_failure_cause_survives_disabled_logging` (REQ AC-1 through AC-5).
- [lifecycle_tests/test_task_completion_lifetime.py](../../../tests/python_tests/lifecycle_tests/test_task_completion_lifetime.py)::`test_task_error_survives_instance_destruction`
  runs in an isolated process, collects a real missing-source clone failure,
  destroys its instance and checks the retained exception's receipts and cause
  (REQ AC-4).
- [test_simulation.py](../../../tests/python_tests/test_simulation.py)::`test_wait_task_timeout_variants`
  covers real-runtime finite and infinite waits, while
  `test_wait_task_timeout_boundary` covers polling with either immediate success
  or timeout followed by retry (REQ AC-2).
- [test_python_keyword_only.py](../../../tests/python_tests/test_python_keyword_only.py)
  checks both wait signatures;
  [test_module_exports.py](../../../tests/python_tests/test_module_exports.py)::`test_task_types_are_exported`
  checks the public receipt and exception exports (REQ AC-1 and AC-2).

- [test_session_tasks.py](../../../tests/python_tests/test_session_tasks.py)
  checks delayed output publication, EOF collection, execution failures, ctypes
  receipt returns, rejected-versus-accepted cleanup ownership, draining after an
  earlier rejection, and exception chaining (REQ AC-6 and AC-7). These are wrapper
  tests with native operations replaced, including the query/update/release
  articulation sequence (REQ AC-6 through AC-8); runtime stream ordering and buffer
  lifetime remain covered by TEST-PYTHON-READ-001.

- [test_articulation_kinematic_selection_gpu.py](../../../tests/python_tests/test_articulation_kinematic_selection_gpu.py)
  runs the update with `prim_paths=` and `pattern=` selections on DirectGPU and checks that
  only the selected articulations' links move (REQ AC-8).
  [cpu_tests/test_articulation_kinematic_selection.py](../../../tests/python_tests/cpu_tests/test_articulation_kinematic_selection.py)
  checks that a selection with `scope=ACTIVE`, or an empty pattern, raises `ValueError`
  (REQ AC-8).

- [cpu_tests/test_task_completion.py](../../../tests/python_tests/cpu_tests/test_task_completion.py)::`test_waiting_session_calls_collect_only_their_own_tasks`
  checks that the explicit waiting adapters and the articulation call collect their own
  receipts while leaving an earlier unrelated failure for the caller (REQ AC-6).

- [test_session_operations.py](../../../tests/python_tests/test_session_operations.py)
  checks lifecycle submission and ownership with
  `test_attach_retains_stage_until_collected_ownership_is_known`,
  `test_lifecycle_receipts_separate_admission_from_execution`,
  `test_dropped_detach_receipt_retains_stage_through_timeout`,
  `test_rejected_detach_does_not_retire_live_write_session`, and
  `test_detach_leaves_earlier_failure_for_explicit_collection` (REQ AC-5, AC-7).
  It also runs the delayed-double cases: `test_default_query_does_not_wait_or_expose_uncollected_output`,
  `test_default_session_open_neither_fetches_nor_collects` and
  `test_zero_one_and_many_real_receipts` (REQ AC-6);
  `test_deferred_cleanup_drain_submits_without_waiting`,
  `test_write_release_has_only_real_receipts_and_no_wait` and
  `test_close_rejected_group_retains_owner_for_retry` (REQ AC-7);
  `test_wait_all_records_without_materializing_payload`,
  `test_single_task_wait_does_not_collect_other_task`,
  `test_cached_failure_survives_wait_result_destruction_and_repeats` and
  `test_wait_all_timeout_caches_only_completed_prefix` (REQ AC-9);
  `test_operation_keeps_inputs_until_collection`,
  `test_dropped_multi_task_retains_inputs_until_all_outcomes_collected` and
  `test_dropped_creator_releases_orphan_without_materialization` (REQ AC-10).
  The same file also checks that a cached conversion failure holds no traceback to its
  unexposed array (`test_conversion_error_does_not_keep_unexposed_array_in_cached_traceback`,
  REQ AC-9) and that earlier failures stay primary when cleanup admission or a
  materialization cleanup also fails
  (`test_partial_release_admission_preserves_prior_execution_failure`,
  `test_materialization_error_requeues_rejected_cleanup`, REQ AC-7).
  `test_deferred_write_release_failure_survives_dropped_receipt` and
  `test_fetch_cleanup_failure_survives_dropped_group_receipt` verify that implicit
  release failures remain reportable through `wait_all` without retained Python
  receipts; `test_destroy_reports_last_stage_group_release_failure` verifies the
  same collection during stage-bound group teardown (REQ AC-7).
- [cpu_tests/test_session_async.py](../../../tests/python_tests/cpu_tests/test_session_async.py)
  runs `test_default_read_uses_completed_query_and_preserves_its_snapshot`,
  `test_multi_attribute_write_keeps_request_identity_without_metadata` and
  `test_failed_multi_attribute_open_is_cached_without_a_session` against the real CPU
  runtime (REQ AC-6, AC-9).
- [test_output_read_device_stream.py](../../../tests/python_tests/test_output_read_device_stream.py)::`test_async_fetch_materializes_on_first_result_stream_not_collection_stream`
  shows on a real CUDA device that `wait_all` under one stream converts nothing and the
  first `result` under another performs the single checked handoff (REQ AC-9).
- [test_session_result_abi.py](../../../tests/python_tests/test_session_result_abi.py)
  compares the five session-result ctypes layouts with the linked native library
  (REQ AC-6).

Evidence status: an earlier revision of the session-async and CUDA stream cases was
reported passing on an installed runtime; no result for the current revision of these
files, the delayed-double cases or the ABI case is recorded in this specification.
