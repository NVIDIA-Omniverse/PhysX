<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ATTACH-OWNER-001
title: Process-Wide Ovstage Attach Ownership
status: implemented
owner: ovphysx
---

## Description

`IPhysxSimulation` is a process-wide singleton, and its
`beginSimulationAttach()` unconditionally tears down whatever attach is
currently live before installing a new one. `ovphysx_attach_ovstage()`'s
own "already attached" guard only inspected the calling instance's local
`attachHandle`, so a second `ovphysx` instance could call
`ovphysx_attach_ovstage()` while a first instance's attach was still live:
the call would reach `beginSimulationAttach()`, silently detach the first
instance's runtime attach, and install the second instance's attach in its
place. The first instance's own `attachHandle` bookkeeping was never
touched by the second instance's call, so the first instance's step and
detach paths kept trusting a handle that no longer corresponded to a live
runtime attach — a silent cross-instance control hazard.

`ovphysx.cpp` now tracks the process-wide live-attach owner explicitly and
rejects a second instance's attach attempt outright while another
instance's attach is live, rather than letting it displace the first.

## Acceptance Criteria

- AC-1: `ovphysx_attach_ovstage()` returns `OVPHYSX_API_ERROR` and leaves no
  partial attachment when another instance already owns the one live
  process-wide PhysX attach. The owning instance's own attach handle and
  runtime attach are unaffected by the rejected attempt.
- AC-2: `ovphysx_detach_ovstage()` only calls
  `IPhysxSimulation::detachStage()` when the calling instance is still the
  recorded process-wide attach owner, so a call on an instance that is not
  (or is no longer) the owner cannot tear down a different instance's live
  attach.
- AC-3: `omni_sdk_physx_destroy()` releases the process-wide attach owner
  latch for its own handle even when the normal `ovphysx_detach_ovstage()`
  call it makes fails (e.g. because `wait_for_all_pending_ops()` finds a
  permanently-failed pending op). Destruction must never leave the latch
  attributed to a handle that no longer exists, which would otherwise reject
  every later instance's attach forever.
- AC-4: A rejected attach leaves the non-owning instance unattached and
  reusable. After the current owner releases the live attach, the rejected
  instance can attach its own stage normally, create a nonempty rigid-body-pose
  tensor binding, and read finite poses from it.

## Test References

- TEST-CAPI-ATTACH-OWNER-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_attach_ovstage`, `ovphysx_detach_ovstage`, `omni_sdk_physx_destroy`, `g_liveAttachOwner`)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_attach_ovstage` documented failure mode)
- ovphysx/tests/c_unittests/test_multi_instance.cpp (C API regression)
- ovphysx/tests/c_unittests/test_global_lifecycle.cpp (destroy-under-failed-op latch release, across shutdown/reinitialize)
- ovphysx/tests/python_tests/lifecycle_tests/test_multi_instance.py (DirectGPU binding lifetime and rejected-instance reuse)

## Dependencies

- REQ-CAPI-DETACH-001 (teardown-call ordering within a single instance's detach)
