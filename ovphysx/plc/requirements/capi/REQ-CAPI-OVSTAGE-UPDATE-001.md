<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-UPDATE-001
title: Monotonic OVStage Update API
status: implemented
owner: ovphysx
---

## Description

`ovphysx_update_from_ovstage()` shall expose monotonic consumption of
caller-owned OVStage ordinals. The ordinal parsed by attachment and every
successfully drained ordinal are consumed once; repeating them does not repeat
physics changes or object-change callbacks.

## Acceptance Criteria

- AC-1: A valid update range containing only consumed ordinals returns an
  accepted task that completes successfully as a no-op and emits no
  object-change callbacks.
- AC-2: A valid range overlapping consumed and unread ordinals applies only its
  unread suffix. Post-attach population remains observable, and replaying the
  successfully consumed ordinal does not deliver its callbacks again.
- AC-3: Update returns `ovphysx_enqueue_result_t`. Invalid arguments, an invalid
  instance reject without a task or mutation. Missing attachment and runtime
  failures are evaluated during execution and belong to the task outcome.
  The Stage and sealed data named by the range remain readable through task
  completion. The range is passed by value; no caller-owned range storage is
  retained.

- AC-4: An incremental scan of unreadable added-prim data fails the accepted
  update task without consuming its ordinal range. Repeating the unreadable
  range fails again. After the caller seals the missing data, retrying the same
  range creates the additions while preserving unaffected objects; replay after
  success does not duplicate them.

- AC-5: **Signed OVStage ordinals.** The range carries OVStage's `int64_t`
  ordinals. Ordinals below 1 precede every attach ordinal and count as
  consumed: a range wholly below 1 is a successful no-op, and one that starts
  below 1 drains from 1. Python rejects range ordinals outside OVStage's
  `int64_t` domain with `ValueError` before native submission; the runtime
  update fails for an unsigned ordinal above `OVSTAGE_ORDINAL_MAX`.

- AC-6: **Refused inside a CUDA graph capture.** A graph replay does not repeat
  the update. A call made while a capture of a blocking stream holds the legacy
  default stream of the calling thread's current context is rejected with
  `OVPHYSX_API_INVALID_STATE`, without a task or mutation, and the capture stays
  valid. This includes a capture that another thread runs in the same context. A
  capture of a non-blocking stream, such as the side stream PyTorch's CUDA graphs
  capture on, is not detected: the update is accepted and runs once, at capture
  time.

## Test References

- TEST-CAPI-OVSTAGE-UPDATE-001
- TEST-CAPI-OVSTAGE-UPDATE-003
- TEST-CAPI-OVSTAGE-UPDATE-004

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_update_from_ovstage` contract)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_update_from_ovstage` forwarding; capture refusal, AC-6)
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp (`InitialPopulationDeliversNoCreatedCallbacks`)

- ovphysx/tests/c_unittests/test_usd_loading.cpp (`PartialSealIncrementalScanCanRetryAfterSealingJointRelationships`)
- ovphysx/tests/c_unittests/test_worker_lifecycle.cpp (`UpdateFromOvstageInsideACaptureIsRefusedGpuTest`; AC-6)
- ovphysx/tests/python_tests/test_write_capture_gpu.py (`test_update_from_ovstage_refused_inside_a_capture`; AC-6)
- ovphysx/python/ovphysx/api.py (`PhysX.update_from_ovstage` range validation; AC-5)
- ovphysx/ovruntime/source/omni.physx/plugins/PhysX.cpp (`physxSimulationUpdateFromOvStage` ordinal bound; AC-5)

## Dependencies

- REQ-SIM-OVSTAGE-UPDATE-001
