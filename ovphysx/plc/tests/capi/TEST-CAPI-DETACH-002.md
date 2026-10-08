<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-DETACH-002
maps_to: REQ-CAPI-DETACH-002
type: unit
---

## Scenario

A stage that declares articulations is attached and detached without ever being
stepped; a different stage is then attached and stepped. The step must complete
normally rather than faulting on record indices left behind by the dead attach.

## Given

`PhysXUsdPhysicsInterface::mArticulations` is populated while a stage is parsed
at attach time and drained only by `finishSetup()` on the first step after an
attach. `AttachedStage::releasePhysicsObjects()` →
`PhysXUsdPhysicsInterface::releaseAllObjects()` is the only teardown that clears
it on a detach that never stepped.

Pre-fix, that call was compiled out under `OVRUNTIME_PHYSX_NO_USD`, so
`mArticulations` still held two `ObjectId`s from `two_articulations.usda` after
the detach. `finishSetup()` on the next attach's first step indexed the new
attach's `InternalPhysXDatabase::getRecords()` with them and dereferenced
`Record::mInternalPtr` as an `InternalArticulation*`, giving SIGSEGV (exit 139)
in `addArticulationToScene`.

## When

`PhysXTestFixture.DetachWithoutStepDropsPendingArticulations`
(`ovphysx/tests/c_unittests/test_usd_loading.cpp`) runs on the installed SDK
(one USD-free build since ADR-0027; the `OVPHYSX_NO_USD=ON` wording in the
history below names the build that existed when the bug was found):

1. attach `tests/data/two_articulations.usda` via ovstage,
2. `ovphysx_detach_ovstage` — no `ovphysx_step` in between,
3. attach `tests/data/minimal_scene.usda` (no articulations),
4. `ovphysx_step` + `ovphysx_wait_op`.

## Then

- Steps 1-3 each return `OVPHYSX_API_SUCCESS`.
- The step in step 4 enqueues with `OVPHYSX_API_SUCCESS` and its wait reports no
  operation errors — the process does not fault (REQ-CAPI-DETACH-002 AC-4).
- Green observed directly on the fixed NO_USD build (in isolation and inside the
  full `cpu` pass). The red half was observed on the pre-fix NO_USD binary as the
  same four steps split across two cases —
  `ScopedSealAllowsUnrelatedUnsealedData` (attach `two_articulations.usda`,
  detach, never step) followed by `StepWaitReportsPriorUserTaskFailure` (attach
  `minimal_scene.usda`, step), exit 139 in `addArticulationToScene` — not by
  running this case itself against a re-fenced build.
- The same ordering is also covered incidentally by the full `cpu` filter pass
  of `scripts/test_cpp.cmake`
  (`-*GpuTest*:GlobalLifecycle.*:ActiveCudaGpusAttachTest.*:OmniPvdColdCreation.*`),
  where `ScopedSealAllowsUnrelatedUnsealedData` /
  `UnsealedAttachFailsAndSealedRetrySucceeds` precede
  `StepWaitReportsPriorUserTaskFailure`. That pass is what the `cpp-unit` CTest
  label runs, and it is green end to end with the fix.

**Coverage note:** AC-2 and AC-3 — that the later attach's first step resolves
no record minted by the destroyed attach — are verified indirectly, through
AC-4's crash-free outcome and by inspection of `releaseAllObjects()` clearing
`mArticulations` / `mParticleSystems`. `ovphysx`'s C ABI exposes no accessor for
those private runtime lists, so no test asserts on them directly.

**AC-1 note:** there is one configuration (ADR-0027), so the original
per-configuration differential claim is moot; the case runs on the one shipped
build and the fence class that produced the bug cannot recur (no
`OVRUNTIME_PHYSX_NO_USD` macro exists to fence on — REQ-BUILD-UNIBUILD-001 AC-1).
