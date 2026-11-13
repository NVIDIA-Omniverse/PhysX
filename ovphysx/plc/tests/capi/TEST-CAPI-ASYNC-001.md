<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-001
maps_to: REQ-CAPI-ASYNC-001
type: integration
---

## Scenario

Native callers use the named timeout contract to poll, wait for a bounded
interval, and wait indefinitely for an asynchronous simulation operation.

## Given

- A pure-C compatibility translation unit that includes the public OVPhysX
  header.
- An asynchronous simulation step held pending by a deterministic test gate.
- An operation whose task has already reached a terminal state.
- The executor deadline conversion checks in
  `tests/internal_unittests/test_instance_executor.cpp` and a completed task.

## When

- The compatibility translation unit checks the timeout type, values, and
  timeout-bearing C function signatures.
- The pending step is waited with `OVPHYSX_TIMEOUT_POLL` and with a finite
  positive timeout before the test gate is released.
- The terminal operation is waited with a finite budget shorter than ordinary
  call overhead.
- Poll, finite and infinite deadline conversions are checked, including a large
  finite value that must not overflow the clock representation.
- The gate is released and the same operation is waited with
  `OVPHYSX_TIMEOUT_INFINITE`.

## Then

- The type size and alignment, named values, and C signatures satisfy AC-1 and
  AC-2.
- The poll returns promptly with `OVPHYSX_API_TIMEOUT`, reports the step as the
  lowest pending operation, and does not consume it, satisfying AC-3.
- The finite wait returns `OVPHYSX_API_TIMEOUT` within its bounded readiness
  wait, reports the step as the lowest pending operation, and does not consume
  it, satisfying AC-4.
- The terminal operation reports and consumes its actual result instead of
  being downgraded to a pending timeout at the deadline boundary, satisfying
  AC-4.
- A completed task wins over an expired readiness budget and is collected;
  source inspection of `InstanceExecutor::waitLocked` verifies the final
  readiness check at the deadline (AC-4). There is no deterministic injected-clock
  test for simultaneous publication at that exact boundary.
- The infinite wait treats the sentinel as unbounded and consumes the operation
  only after the released step reaches a terminal state, satisfying AC-5.
