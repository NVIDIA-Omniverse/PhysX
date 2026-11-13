<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-TASK-002
maps_to: REQ-PYTHON-TASK-001
type: integration
---

## Scenario

Python retains pending lifecycle resources and materializes results only after
successful collection.

## Given

- `tests/python_tests/cpu_tests/test_async_lifecycle_cpu.py`, CPU instances,
  caller-owned Stages and deterministic worker holds.

## When

- Queue attach/detach, drop ordinary Stage references and collect out of order.
- Queue query/read after attach or step, or detach before result materialization.
- Mutate clone inputs after submission and trigger finalizers from worker callbacks.

## Then

- Lifecycle calls return pending tasks; failed execution is reported at collection
  while Stage ownership is retained until known safe (REQ AC-5, AC-7).
- Typed results reflect their submitted attachment and reject invalidated
  materialization; task status does not collect (REQ AC-6, AC-11).
- Clone uses its native copies, and callback finalizers defer destruction safely
  (REQ AC-10).

## Automated coverage

- `test_attach_a_then_b_collect_in_order`, `test_attach_a_then_b_collect_in_reverse`,
  `test_failed_attach_rollback_keeps_stage_until_ownership_known`,
  `test_detach_keeps_stage_until_detach_completes`.
- `test_query_after_unwaited_attach_succeeds`,
  `test_query_and_read_queued_behind_a_step_materialize_after_it`,
  `test_detach_before_materialization_invalidates_the_query`,
  `test_query_task_status_does_not_collect`.
- `test_clone_inputs_may_change_after_submit`, `test_finalizer_on_worker_thread_only_defers`.
