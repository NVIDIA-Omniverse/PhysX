<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-THREADING-002
maps_to: REQ-CAPI-THREADING-001
type: integration
---

## Scenario

A one-thread Carbonite tasking pool disables asynchronous instance execution.

## Given

- A fresh process, the async setting on and a tasking thread-count override of one.

## When

- Create an instance, submit a test job and inspect its execution thread/status.
- Read the configured async setting and flush the warning callback.

## Then

- The job has completed on the submitting thread before return, the setting
  remains true, and exactly one warning names synchronous fallback (REQ AC-6).

## Automated coverage

- `WorkerPoolOneThreadTest.InstancesRunSynchronously` in
  `tests/c_unittests/test_worker_pool.cpp`.

## Coverage limits

Missing `IThreadPool` fallback is source-inspected; this integration case exercises
small-pool fallback. `ExecutorThreads.CreateFailsWithoutAPoolThread` tests pool
creation/enqueue failure, which fails creation rather than selecting fallback.
