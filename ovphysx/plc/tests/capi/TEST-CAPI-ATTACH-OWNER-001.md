<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ATTACH-OWNER-001
maps_to: REQ-CAPI-ATTACH-OWNER-001
type: integration
---

## Scenario (AC-1, AC-2, AC-4)

Two ovphysx instances attach separate ovstage Stages loaded from the same USD
file. Each keeps its own runtime attachment. Stepping and detaching one does
not displace the other, and an existing GPU pose read on the survivor stays
finite.

## Given

- Instance A attaches an ovstage Stage and reads a nonempty GPU rigid-body-pose
  binding.
- Instance B is a separate instance with its own ovstage Stage from the same
  USD scene.

## When

- Instance B calls `ovphysx_attach_ovstage()` while A's attach is still live,
  warms up, and reads its own poses.
- Instance A reads through the same tensor binding again.
- Instance B calls `ovphysx_detach_ovstage()`.
- Instance A reads through the same tensor binding once more and steps.
- On the C path, both instances step, A detaches, and B steps
  again. A's all-objects-destroyed subscription fires; B's does not.
- On the C path, both instances also alternate steps, then A calls
  `ovphysx_detach_ovstage()` and is destroyed. B's attachment and a later step
  survive.
- Separately on the C path, attaching A's already-attached ovstage instance to
  B is rejected while A remains attached, and A still steps afterward.
- On the GPU path, stepping B does not change A's pose read, and stepping A
  does not change B's pose read.

## Then

- B's attach task completes successfully. `ovphysx_is_stage_attached` confirms each instance
  owns its stage, and A's retained query remains valid (REQ AC-1).
- Both instances can step while both are attached (REQ AC-1).
- A's pose read after B attaches matches the read taken before B attached
  (REQ AC-4).
- B's detach leaves B unattached and A attached. A's retained query and pose
  read remain valid, and A can still step (REQ AC-2, AC-4).
- Detaching A notifies only A's object-change subscription and leaves B
  steppable.
- `test_multi_instance.cpp`: `TwoInstancesRemainAttachedAndStepIndependently`,
  `DetachAndDestroyOneLiveInstanceLeavesPeerSteppable`,
  `SecondAttachOfSameOvstageIsRejected`.
- `lifecycle_tests/test_multi_instance.py`:
  `test_second_attach_leaves_first_gpu_binding_readable`.

## Scenario (AC-3)

Destroying an attached instance with an uncollected task failure must release
that instance's runtime attachment, so a later instance can attach -- across
both a same-process next instance and a full `ovphysx_shutdown()` /
`ovphysx_initialize()` cycle.

### Given

- Instance A attaches an ovstage Stage.
- A clone submitted to A with a missing source is accepted and fails inline,
  leaving an uncollected failed task in A's task tracking (never collected via
  `ovphysx_wait_task()` or `ovphysx_wait_all()`).

### When

- `ovphysx_destroy_instance()` is called on A. Its internal execution drain
  preserves the task failure without treating it as a prerequisite for teardown.

### Then

- Instance A is fully destroyed and destruction succeeds.
- A newly created instance B can call `ovphysx_attach_ovstage()` successfully
  and step (REQ AC-3).
- After destroying B, `ovphysx_shutdown()` and a fresh `ovphysx_initialize()`
  also leave the runtime in a clean state: a new instance C can attach and
  step normally.
- `test_global_lifecycle.cpp`:
  `DestroyAfterFailedPendingOpReleasesLiveAttachOwnerAcrossInstancesAndShutdown`.

This scenario verifies release with uncollected failures. The defensive
attachment release after a separate detach error remains a source-inspected
part of AC-3; this test no longer injects a detach error. That fallback resets
the attachment's tensor stage before `detachStage`, then clears the handle.
