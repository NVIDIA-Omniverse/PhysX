<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-EXECUTOR-001
maps_to: REQ-CAPI-EXECUTOR-001
type: unit
---

## Scenario

The per-instance executor keeps order, never loses a wake-up, publishes outcomes
before releasing the lane, and tears down without joining from the wrong place,
both in isolation and behind a real instance.

## Given

- The isolated `instance_executor_unittests` binary
  (`tests/internal_unittests/test_instance_executor.cpp`), which compiles only the
  executor source with a fake `IThreadPool`, linking no Carbonite runtime, PhysX or stage, so lane bugs surface
  directly. A fake Carbonite task (thread-local flag plus futex-style hooks)
  exercises the task-aware wait path.
- `ExecutorTasks` and `ExecutorTasksLifecycle` in
  `tests/c_unittests/test_executor_tasks.cpp`, which use the private worker hold and
  test jobs on a real CPU instance.

## When

- Take and release a free lane; submit jobs and observe where and in which order
  they run; hold the worker and poll, time out and then release it.
- Have caller threads take the lane while jobs are queued, and run
  a stress loop that interleaves submissions, caller holds and a concurrent
  caller thousands of times.
- Acquire the lane again from its holder and from inside a job; try-acquire while the
  worker runs.
- Throw from a job, record a failure, and throw a non-standard exception.
- Destroy a job whose destructor takes the executor mutex.
- Preempt while a job runs, then release with jobs queued; preempt, cancel the
  queue and stop; fill a bounded queue.
- Wait from a fake Carbonite task; drop an executor without stopping it; stop the
  executor from inside its own job; convert timeouts to deadlines.
- On an instance: wait on held jobs from more Carbonite tasks than the pool has
  workers while a probe task must still run; destroy an instance with held jobs.

- Keep the worker held and let an infinite native wait execute jobs up to its
  target; repeat from a fake Carbonite task and verify it does not execute jobs.
- Inspect the fake thread-pool calls, force creation/enqueue failure, and exercise
  a workerless executor.

## Then

- An idle lane is taken and released without the worker; jobs run on another
  thread in submission order; a held job stays pending, poll and finite waits time
  out without collecting, and release lets it succeed (REQ AC-2, AC-3, AC-6, AC-8).
- A caller never takes the lane before earlier jobs finished, and the stress loop
  observes every earlier job at each caller hold, ends with a free lane and no
  unfinished task (REQ AC-2, AC-3).
- Nested holds change nothing; try-acquire fails while the worker owns the lane
  and succeeds when it is free (REQ AC-3).
- Exceptions and recorded failures become task failures with their text, and the
  job destructor observes the published outcome without deadlocking (REQ AC-4, AC-5).
- Preemption waits for the running job and keeps queued jobs, which resume in order
  on release; cancellation retires queued jobs as failures and destroys them; a
  stopped executor rejects jobs; a full queue rejects without taking the job
  (REQ AC-7).
- Task-aware waits suspend through the hooks with balanced pins; an abandoned
  executor stays alive through its parked worker; stopping from the worker defers pool destruction to a later external stop;
  zero and maximum timeouts map to poll and infinite (REQ AC-1, AC-6).
- Waiting Carbonite tasks release their workers, the probe runs, and every wait
  succeeds after release; destroying an instance with held jobs succeeds and later
  status queries and waits report `NOT_FOUND` (REQ AC-1, AC-6).
- An infinite native wait takes only unstarted jobs through its target; Carbonite
  waits and finite/poll waits execute none (REQ AC-3, AC-6).
- The executor creates one one-thread pool and enqueues one loop; teardown destroys
  the pool once, and a failed enqueue releases it. Workerless submission runs on
  the caller (REQ AC-1, AC-7).

## Automated coverage

In `tests/internal_unittests/test_instance_executor.cpp`:
`ExecutorThreads.WorkerIsTheOneJobOfAOneThreadPool`,
`ExecutorThreads.CreateFailsWithoutAPoolThread`,
`ExecutorTest.WaiterRunsUnstartedJobsUpToItsOwnAndHandsTheLaneBack`,
`ExecutorTest.WaitingTaskRunsNoJob`, `ExecutorWithoutWorker.SubmittingThreadRunsItsJobs`,
`ExecutorStress.IdleCallerTableAccessNeverRacesTheWorker`, and
`ExecutorTest.PreemptWaitsUntilTheFinishedJobIsDestroyed` cover the final execution model.
