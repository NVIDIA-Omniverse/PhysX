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
