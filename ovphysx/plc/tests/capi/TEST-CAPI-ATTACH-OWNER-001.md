<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ATTACH-OWNER-001
maps_to: REQ-CAPI-ATTACH-OWNER-001
type: integration
---

## Scenario (AC-1, AC-2, AC-4)

A second `ovphysx` instance's `ovphysx_attach_ovstage()` call is rejected,
not silently allowed to displace another instance's live attach, and
neither the rejected attach nor a no-op detach on the second instance can
disturb the first instance's live attach or an existing tensor binding.

## Given

- Instance A attaches an ovstage Stage and holds the one live process-wide
  PhysX attach.
- Instance A creates a nonempty GPU rigid-body-pose tensor binding and reads
  it into a CPU buffer.
- Instance B is created separately and builds its own ovstage Stage from
  the same USD scene.

## When

- Instance B calls `ovphysx_attach_ovstage()` directly while A's attach is
  still live.
- Instance A reads through the same tensor binding again.
- Instance B then calls `ovphysx_detach_ovstage()` (never having validly
  attached).
- Instance A reads through the same tensor binding once more.
- After A destroys its binding and releases the live attach, B retries the
  attach and reads its own nonempty GPU rigid-body-pose binding.

## Then

- B's `ovphysx_attach_ovstage()` call returns `OVPHYSX_API_ERROR` and B's
  own attach handle remains 0 (REQ AC-1).
- A's attach handle (read via `ovphysx_get_attach_handle()`) is unchanged
  before and after B's rejected attempt, and `ovphysx_step_sync()` on A
  still succeeds afterward, proving A's runtime attach was never displaced.
- The same A tensor binding rewrites a poisoned CPU buffer with the original
  finite poses after B's rejected attach, proving the binding still reaches
  A's live GPU scene.
- B's `ovphysx_detach_ovstage()` call succeeds as a harmless no-op (REQ
  AC-2) and does not disturb A's attach handle, tensor binding, or ability
  to step.
- Once A releases the live attach, B can attach normally and read its own
  binding, proving the earlier rejection left B reusable (REQ AC-4).
- `test_multi_instance.cpp`: `SecondInstanceAttachRejectedWhileFirstOwnsLiveAttach`.
- `lifecycle_tests/test_multi_instance.py`:
  `test_rejected_second_attach_keeps_first_gpu_binding_readable`.

## Scenario (AC-3)

Destroying an attached instance whose pending-op wait is permanently failing
must still release the process-wide attach owner latch, so a later instance
can attach -- across both a same-process next instance and a full
`ovphysx_shutdown()` / `ovphysx_initialize()` cycle.

### Given

- Instance A attaches an ovstage Stage and holds the one live process-wide
  PhysX attach.
- A user task added to A returns `OVPHYSX_API_ERROR`, leaving a permanently
  unconsumed failed op in A's pending-op tracking (never observed via
  `ovphysx_wait_op()`).

### When

- `ovphysx_destroy_instance()` is called on A. Its internal
  `wait_for_all_pending_ops()` and the `ovphysx_detach_ovstage()` call it
  makes both fail because of the still-pending failed op.

### Then

- Instance A is fully destroyed despite the detach failure.
- A newly created instance B can call `ovphysx_attach_ovstage()`
  successfully and step -- the latch was not left attributed to A's
  now-invalid handle (REQ AC-3).
- After destroying B, `ovphysx_shutdown()` and a fresh
  `ovphysx_initialize()` also leave the runtime in a clean state: a new
  instance C can attach and step normally.
- `test_global_lifecycle.cpp`:
  `DestroyAfterFailedPendingOpReleasesLiveAttachOwnerAcrossInstancesAndShutdown`.
