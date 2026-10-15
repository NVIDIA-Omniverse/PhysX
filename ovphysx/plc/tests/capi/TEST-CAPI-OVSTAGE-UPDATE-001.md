<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-UPDATE-001
maps_to: REQ-CAPI-OVSTAGE-UPDATE-001
type: integration
---

## Scenario

The public C API does not replay the initial OVStage population when the caller
redundantly updates the ordinal already consumed by attachment.

## Given

- A subscriber registered through `ovphysx_subscribe_object_changes()` before
  any stage is attached.
- A populated and sealed OVStage attached at its initial read ordinal.

## When

- `ovphysx_update_from_ovstage()` is called for exactly the attach ordinal.
- The simulation performs its first step.

## Then

- The update returns an accepted task whose wait succeeds (AC-1).
- No object-created or object-destroyed callbacks are delivered for the attach
  snapshot (AC-1).
- Runtime coverage additionally drains an overlapping range, observes a
  post-attach creation, and verifies replay does not repeat it (AC-2).
- An update without an attachment is rejected without a task; a valid consumed
  range produces a separately collectible successful task (AC-3).

## Test locations

- `ovphysx/tests/c_unittests/test_object_change_callbacks.cpp`:
  `InitialPopulationDeliversNoCreatedCallbacks`.
- `ovphysx/ovruntime/source/omni.physx/tests/test.unit/physics/TestOvstageChange.cpp`:
  `Ovstage update skips consumed ordinals and delivers later population`.
- `ovphysx/tests/c_unittests/test_usd_loading.cpp`:
  `AttachAndUpdateReturnCollectableTasks` (accepted update identity and
  collect-once completion for AC-3).
- `ovphysx/tests/c_unittests/test_task_completion.cpp`:
  `LifecycleInvalidArgumentsAndMissingAttachmentRejectWithoutTaskIdentity`
  (invalid range, missing attachment and invalid instance reject without a task
  for AC-3).
