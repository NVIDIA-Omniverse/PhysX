<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-DETACH-001
maps_to: REQ-CAPI-DETACH-001
type: unit
---

## Scenario

`ovphysx_detach_ovstage`'s two teardown calls run in the order
`SimulationBackend.cpp` requires: `resetStage` before `detachStage`, not
after.

## Given

The pre-fix source read `physxSim->detachStage()` first (destroying the
`AttachedStage`) and then `TensorApi::resetStage(attachHandle)` — the exact
inverse of `SimulationBackend.cpp`'s header comment on `resetStage` ("this
must run before the attach is torn down") and of the order
`TestOvstageStageless.cpp` exercises.

## When

`ovphysx_detach_ovstage`'s body is reordered: the `resetStage` block now
precedes the `physxSim->detachStage()` call, unconditionally for any attach
with `ovstage_attached == true` (not gated on `stageId != 0`).

## Then

- Static/code-review confirmation only: the reordered call sequence in
  `ovphysx.cpp` now matches `SimulationBackend.cpp`'s documented requirement
  and `TestOvstageStageless.cpp`'s exercised order.

**Known gap, not closed by this fix:** no test — at either the `ovphysx` C
ABI layer or `ovruntime`'s own suite — pins the *call order* of
`ovphysx_detach_ovstage`'s two teardown steps directly (e.g. via an
instrumented `IPhysxSimulation`/`TensorApi` pair recording invocation order
across a real detach call). The bug was latent, not crash-reproducing
(`BaseSimulationView::invalidate()` only nulls its own pointer fields rather
than dereferencing the dangling attach), so no existing case failed before
the fix and none newly passes after it — this AC is verified by inspection
of the reordered source against the documented contract, not by a red→green
test. A regression test would need to instrument call order directly, which
is out of scope for `ovphysx`'s existing GTest/pytest harnesses and not
attempted here without sign-off on the added instrumentation.

## Attachment state and task outcomes (AC-3, AC-4)

`PhysXTestFixture.AttachmentStateHasNoRuntimeIdentityAndRejectsInvalidArguments`
checks null/invalid outputs, attached/detached state, repeated detach, and a
failed task whose diagnostic remains collectible after teardown.
`test_lifecycle.py::test_native_same_stage_reattach_invalidates_python_cache`
performs native detach/reattach of the same Stage and checks Python state and
cache rejection. Task-completion tests verify accepted detach receipts,
including repeated detach, and earlier failures remain independently collectible.
Python lifecycle tests retain the Stage across pending tasks and after failure
when native state is attached or cannot be observed. Borrowed-read lifecycle and multi-instance
suites cover retained storage and owner isolation. Teardown's unchanged handling
of accumulated simulation time is verified by code review; no new time oracle
is claimed. The direct teardown-call-order coverage gap above remains.

## Admission before teardown (AC-5)

Lifecycle tests reject invalid instance and companion-transition submissions
without issued task identity. Companion rejection preserves an existing write
session. Destruction and failed-attach rollback create no nested detach receipt;
source review checks that they use the private teardown path.

## Private query validity (AC-6)

### Given

- An instance owns a retained ordinary output query on an attached Stage; another
  instance owns a separate Stage and query.
- The first instance has uncollected successful and failed task outcomes.

### When

- The private native helper checks a live query, a null output, an invalid
  instance, an unknown query, a foreign query and a released query.
- Native calls detach and reattach the same Stage, and the helper checks the old
  retained query again.
- Task outcomes are collected after the validity checks.

### Then

- The live original query returns success with true. The old retained query
  returns success with false after detach and remains false after reattachment
  (REQ AC-6).
- Invalid instance/output arguments return `OVPHYSX_API_INVALID_ARGUMENT`.
  Unknown, foreign and released queries return `OVPHYSX_API_NOT_FOUND`; every
  nonnull output is false on failure (REQ AC-6).
- Successful and failed task outcomes remain separately collectible after the
  helper drains execution (REQ AC-6).
- Public C headers omit `ovphysx_query_shared_dictionary`; Python methods/stubs
  omit `PhysX.query_shared_dictionary()`. Neither surface gains a public query
  validity accessor (REQ AC-6).

Native coverage is in `tests/c_unittests/test_multi_instance.cpp`:
`QueryValidityRejectsInvalidArgumentsAndUnknownQueries`,
`QueryValidityRetainsOriginalAttachmentAndTaskOutcomes`, and
`ForeignQueryRejectedOnPeerInstance`. Callback-reentrancy rejection is preserved
by the existing drain path and verified by source inspection.
`tests/python_tests/test_session_operations.py::test_query_dictionary_accessor_is_not_public`
checks removal of the Python method and native export; public header/stub removal
and absence of a public replacement are verified by source inspection.
