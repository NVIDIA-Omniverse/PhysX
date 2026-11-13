<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-REENTRY-001
title: Calls from callbacks never block
status: implemented
owner: ovphysx
---

## Description

Callbacks run on the thread that runs the triggering work: the calling thread, the
instance's worker thread, or an engine task thread during a step. A callback that
waited for its own instance would wait for itself, and one that waited for another
instance could deadlock against it. An ovphysx call that would wait is therefore
refused inside a callback, before it takes any lane or waits.

The refusal is per thread, or per Carbonite task when the callback runs as one. It
never blocks another thread.

## Acceptance Criteria

- **AC-1: Callback context.** A thread or Carbonite task is in callback context while
  it runs an ovphysx log callback, a runtime application callback, including the SDK
  and solver-worker callbacks the runtime guards, or an object-change callback. The
  instance's own worker thread is in callback context for calls on that instance,
  since it runs that instance's jobs.
- **AC-2: Refused calls.** In callback context, every call that would take an
  instance lane, wait for a task or for earlier work, or take every instance's lane
  returns `OVPHYSX_API_INVALID_STATE` before it takes a lane or waits, and
  `ovphysx_get_last_error()` names the callback context. This covers session and
  output calls including releases, the private query-validity check, the long
  operations, `ovphysx_step_sync`, `ovphysx_warmup`, legacy instance calls,
  `ovphysx_set_config`, `ovphysx_subscribe_object_changes`, `ovphysx_wait_task`,
  `ovphysx_wait_all`, recording start and stop, contact-processing global
  configuration and `ovphysx_destroy_instance`. A refused call changes nothing and
  may be retried after the callback returns, subject to ordinary validation.
- **AC-3: Calls without engine work.** Inside callbacks these calls behave as
  outside: version, configuration, CPU-mode and log-level getters,
  `ovphysx_query_task_status`, `ovphysx_get_last_error`,
  `ovphysx_get_last_task_error`, `ovphysx_is_stage_attached` and
  `ovphysx_writability` and recording status. Logging configuration keeps its own rule
  ([REQ-CAPI-LOG-001](REQ-CAPI-LOG-001.md) AC-5) and unsubscribe keeps
  [REQ-CAPI-OBJCHANGE-001](REQ-CAPI-OBJCHANGE-001.md) AC-2.
- **AC-4: Other threads proceed.** While a callback runs, calls from other threads on
  the same or another instance are not refused; they wait for the lane when they need
  it.

## Test References

- [TEST-CAPI-REENTRY-001](../../tests/capi/TEST-CAPI-REENTRY-001.md)

## Code References

- `ovphysx/src/ovphysx/LogManager.cpp` (`isInLogCallback`): log-callback context (AC-1).
- `ovphysx/src/ovphysx/ovphysxObjectChangeCallbacks.cpp`: object-change callback context (AC-1, AC-2).
- `ovphysx/src/ovphysx/ovphysx.cpp`: worker-thread context, refusal of lifecycle,
  wait, configuration, recording and destroy calls (AC-1 to AC-4).
- `ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp`, `ovphysx/src/ovphysx/ovphysxTensorBinding.cpp`,
  `ovphysx/src/ovphysx/ovphysxContactBinding.cpp`, `ovphysx/src/ovphysx/ovphysxSdfView.cpp`,
  `ovphysx/src/ovphysx/ovphysxSceneQuery.cpp`: refusal before the lane (AC-2).
- `ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h` (`isInRuntimeCallback`): the
  runtime-callback predicate this frontend relies on.

## Dependencies

- [REQ-CAPI-LANE-001](REQ-CAPI-LANE-001.md)
