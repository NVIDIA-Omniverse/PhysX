<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-LANE-001
title: Calls on the calling thread run after queued work
status: implemented
owner: ovphysx
---

## Description

Calls that execute engine code on the caller hold their instance's lane so they
observe earlier jobs fully applied. Short asynchronous calls instead queue when
busy under REQ-CAPI-ASYNC-002; this requirement covers caller execution and
process-wide transitions.

## Acceptance Criteria

- **AC-1: Whole-call lane.** These calls take the instance lane for their whole
  duration: caller-stream operations listed in REQ-CAPI-ASYNC-002 AC-12,
  `ovphysx_update_articulations_kinematic`, `ovphysx_evaluate_sdf_shapes`,
  `ovphysx_step_sync`, `ovphysx_warmup`, the legacy tensor-binding, contact-binding,
  SDF-view and scene-query calls, `ovphysx_set_config`,
  `ovphysx_subscribe_object_changes` and the private
  `ovphysx_internal_is_query_valid`. Behind a queued or running job the call returns
  only after that job finished, and its result reflects that job. On a free lane it
  runs on the calling thread. A call never waits for another instance's jobs.
- **AC-2: State checks after the lane.** Checks of state that a job can change run
  after the lane is held: runtime busy, attachment present, articulation query type,
  recording and preview state. A call made while a job runs therefore waits and then
  checks the resulting state instead of rejecting solely because that job is busy. Checks of the call's own arguments
  still reject before the lane is taken.
- **AC-3: No collection, no second acquisition.** Waiting for the lane never collects
  a task outcome: uncollected failures stay available to `ovphysx_wait_task` and
  `ovphysx_wait_all`. Internal steps that run while their caller holds the lane
  (failed-attach rollback, the detach body run by destroy, lazy attach, warmup) do not
  take it again.
- **AC-4: Process-wide calls.** `ovphysx_ovd_start_recording`,
  `ovphysx_ovd_stop_recording`, and global configuration that changes contact processing on live
  attachments or needs a detached runtime, take the lane of every registered instance
  in ascending handle order, each after earlier jobs. They acquire the full set
  before taking lifecycle, create or registry locks; they do not enter this path
  while already holding an individual lane. Recording-status queries read only
  recording state and do not wait for instance work.

## Test References

- [TEST-CAPI-LANE-001](../../tests/capi/TEST-CAPI-LANE-001.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: lane helper, `ovphysx_step_sync`,
  `ovphysx_set_config`, process-wide transitions (`enter_shared_runtime_safe_point`,
  `updateLiveContactProcessing`) and internal helpers that assume a held lane
  (AC-1, AC-3, AC-4).
- `ovphysx/src/include/internal/sdk/ovphysxSDK.hpp`: lane helper declaration (AC-1).
- `ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp`: session, release, articulation and
  query-validity calls; state checks after the lane (AC-1, AC-2).
- `ovphysx/src/ovphysx/ovphysxTensorBinding.cpp`, `ovphysx/src/ovphysx/ovphysxContactBinding.cpp`,
  `ovphysx/src/ovphysx/ovphysxSdfView.cpp`, `ovphysx/src/ovphysx/ovphysxSceneQuery.cpp`:
  legacy calls and warmup (AC-1, AC-3).
- `ovphysx/src/ovphysx/ovphysxObjectChangeCallbacks.cpp`: subscribe (AC-1).

## Dependencies

- [REQ-CAPI-EXECUTOR-001](REQ-CAPI-EXECUTOR-001.md) (lane, FIFO and waits)
- [REQ-CAPI-ASYNC-003](REQ-CAPI-ASYNC-003.md) (which calls are queued)
- [REQ-CAPI-REENTRY-001](REQ-CAPI-REENTRY-001.md) (calls from callbacks are refused before the lane)
