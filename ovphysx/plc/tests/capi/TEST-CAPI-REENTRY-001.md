<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-REENTRY-001
maps_to: REQ-CAPI-REENTRY-001
type: integration
---

## Scenario

Callbacks and instance workers cannot wait for their own engine work.

## Given

- `Lanes.CallbackRefusesBlockingCalls` in `tests/c_unittests/test_instance_lanes.cpp`,
  with real log and object-change callbacks and an independent client thread.
- Worker callback cases in `test_worker_step.cpp`, `test_short_call_queue.cpp`
  and `test_worker_teardown.cpp` in the same directory.

## When

- Attempt output, lifecycle, wait, configuration and destruction calls inside
  callbacks and on an instance's own worker.
- Query configuration and attachment status, then use an idle instance from
  another thread before the callback returns.
- Retry a refused wait outside the callback.

## Then

- Blocking calls return `INVALID_STATE` before taking lanes or collecting tasks,
  with an immediate explanation of reentry (REQ AC-1, AC-2).
- Nonblocking metadata reads remain available (REQ AC-3).
- Other-thread work proceeds according to its own instance ordering, and the
  refused wait remains collectible (REQ AC-2, AC-4).

## Automated coverage

- `WorkerStep.CallbackOnWorkerCannotWaitOwnInstance`.
- `WorkerShortCalls.CallOnTheOwnWorkerIsRefused`.
- `WorkerTeardown.DestroyFromOwnWorkerCallbackRefused`.
- `Lanes.CallbackRefusesBlockingCalls`.
