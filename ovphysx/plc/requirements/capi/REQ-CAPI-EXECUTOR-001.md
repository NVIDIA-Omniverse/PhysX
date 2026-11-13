<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-EXECUTOR-001
title: Per-instance executor - Carbonite worker, FIFO and lane
status: implemented
owner: ovphysx
---

## Description

Each instance owns an ordered job queue, a task table and a lane that permits
one caller or worker to execute engine code. An asynchronous instance has one
worker thread supplied by Carbonite; a synchronous instance executes submitted
jobs on the caller. API-specific scheduling is in REQ-CAPI-ASYNC-002/003.

## Acceptance Criteria

- **AC-1: Lifecycle.** An asynchronous executor obtains one dedicated thread
  through `carb::tasking::IThreadPool::createEx(1)` and submits its worker loop
  there. Pool creation or enqueue failure fails instance creation and releases
  any pool already created. Explicit teardown stops the loop, lets running work
  finish, cancels unstarted jobs and destroys the pool exactly once. A worker
  retains its executor until it exits; destruction alone never joins a thread.
  Stopping from the worker leaves pool destruction to a later external stop.
  Hosts complete `ovphysx_shutdown()` before unloading the library or Carbonite.
- **AC-2: Lane.** One atomic word records free, caller or worker ownership and
  whether locked coordination is needed. Only the holder executes engine work.
  A free lane implies an empty queue. Queue publication and worker wake-up are
  coordinated under the executor mutex, so a later call cannot overtake queued
  work. An uncontended idle acquire/release uses compare-and-swap.
- **AC-3: Order.** Jobs run one at a time in submission order. A blocking lane
  acquire waits for earlier jobs; a holder's nested acquire is a no-op. A
  nonblocking idle acquire fails while work is queued, running or held. A
  waiting native thread may take unstarted jobs as specified in AC-6 without
  changing their order.
- **AC-4: Task table.** Queued submission reserves identity, failure storage and
  queue storage before publication; failed reservation leaves no task. Jobs
  publish outcomes under the mutex before releasing the lane. Caller execution
  shares the same receipt sequence and may reuse a reserved failure slot.
  Externally serialized callers may access the table without its mutex only
  when the idle lane or their lane hold excludes concurrent job publication.
- **AC-5: Job execution.** Bodies run outside the executor mutex, inside a
  profiler zone and runtime error scope. Exceptions or captured runtime errors
  become task failures with an API/task-specific cause. Captured resources are
  destroyed outside executor locks after outcome publication. An empty queue
  permits returning the lane before job destruction, but teardown waits until
  destruction finishes.
- **AC-6: Waits.** Polls and finite waits never run jobs. An infinite task wait
  on a native thread may execute unstarted jobs through its target, or through
  the wait-all entry cutoff, and leaves later jobs queued. It does not take over
  a running job or execute work while the legacy CUDA stream is held by capture.
  Carbonite tasks do not take over jobs: waits suspend the task with balanced
  OS-thread pinning, preserving caller context without starving the task pool.
  Native waits otherwise use condition variables. Wait hooks observe the
  already-started task pool and never start or resize it.
- **AC-7: Teardown control.** Preemption takes the lane between jobs, after
  running-job cleanup, without draining the queue. Releasing an uncommitted
  teardown resumes queued work. Committed cancellation retires unstarted jobs
  as failures; stopped executors reject submissions. Queue exhaustion rejects
  without waiting. A workerless executor runs accepted jobs on the submitting
  thread before return.
- **AC-8: Test gate.** Private test hooks hold the worker and submit jobs that
  touch no engine state. A held worker starts no job; an infinite native wait
  may still execute held, unstarted work, permitting deterministic takeover tests.

## Test References

- [TEST-CAPI-EXECUTOR-001](../../tests/capi/TEST-CAPI-EXECUTOR-001.md)

## Code References

- `ovphysx/src/include/internal/sdk/InstanceExecutor.hpp`: lane, task table and waits.
- `ovphysx/src/ovphysx/InstanceExecutor.cpp`: worker, takeover and teardown.
- `ovphysx/src/ovphysx/ovphysx.cpp`: lifecycle, wait hooks and job wrapper.
- `ovphysx/src/include/internal/sdk/ovphysxSDK.hpp`: instance jobs.
- `ovphysx/src/include/internal/sdk/TaskContext.hpp`: caller receipts.
- `ovphysx/src/ovphysx/ovphysxTestHelpers.cpp`: test gates.

## Dependencies

- [REQ-CAPI-TASK-001](REQ-CAPI-TASK-001.md)
- [REQ-CAPI-THREADING-001](REQ-CAPI-THREADING-001.md)
