<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-005
maps_to: REQ-CAPI-TASK-001
type: integration
---

## Scenario

Polls and finite waits observe completion; infinite waits may execute unstarted
jobs. The task status
query reports progress without collecting.

## Given

- `ExecutorTasks` in `tests/c_unittests/test_executor_tasks.cpp` on the shared CPU
  instance, using the private worker hold and test jobs that touch no engine state.
- The boxes-on-ground-plane scene for the pending-step case.
- `test_task_status_reports_without_collecting` in
  `tests/python_tests/cpu_tests/test_task_completion.py` for the Python binding.

## When

- Hold the worker, queue a job, query its status, poll it, wait 20 ms for it, then
  release the worker and wait without a deadline; wait for it again.
- Queue a sleeping job and query its status until it runs.
- Queue a job that fails; query its status, collect it, read its retained cause.
- Complete an inline task, hold the worker, queue a successful and a failing job,
  poll `ovphysx_wait_all`, release the worker and wait for all.
- Query status with a null output, a zero receipt, a receipt of another instance,
  an invalid instance and a never-issued index.
- Submit `ovphysx_step` on an attached scene and query its status before and after
  its wait.

## Then

- A held job reports `PENDING`; the poll returns `TIMEOUT` with the job as
  `pending_task`; the finite wait returns `TIMEOUT` after its deadline and does not
  collect; the infinite wait succeeds after release; the collected job reports
  `NOT_FOUND` and a repeat wait returns `NOT_FOUND` (REQ AC-9, AC-10).
- A sleeping job reports `RUNNING` while it executes (REQ AC-10).
- A failed job reports `FAILED` until collected; the wait returns `ERROR` with its
  receipt and the cause `test_job (task <index>): planted failure` (REQ AC-5, AC-9,
  AC-10).
- The polled `ovphysx_wait_all` collects the inline task, stops at the first held job
  and leaves the later one `PENDING`; the final wait reports only the failing job
  (REQ AC-3, AC-9).
- Argument errors return `INVALID_ARGUMENT` and set the output to `NOT_FOUND`; an
  invalid instance returns `NOT_FOUND`; a never-issued index succeeds with
  `NOT_FOUND`; the query never collects (REQ AC-10).
- A step reports `PENDING` before execution, `RUNNING` during execution and
  `SUCCEEDED` or `FAILED` after publication, then `NOT_FOUND` after collection
  (REQ AC-6, AC-10).
- Python `PhysX.task_status` mirrors these states, rejects non-Task arguments with
  `TypeError` and invalid receipts with `RuntimeError`.

The takeover assertions are in `WorkerWaits.InfiniteWaitRunsAJobTheWorkerHasNotStartedOnTheWaitingThread`,
`FiniteWaitsAndPollsNeverRunJobs` and `WaitRunsJobsUpToItsOwnAndTheWorkerRunsTheRest`
in `tests/c_unittests/test_executor_tasks.cpp` (REQ AC-2, AC-3, AC-9).
