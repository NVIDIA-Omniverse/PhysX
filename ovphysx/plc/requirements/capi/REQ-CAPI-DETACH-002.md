<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-DETACH-002
title: Detach Releases Runtime Engine Objects So a Later Attach Cannot Index Dead Records
status: implemented
owner: ovphysx
---

## Description

`ovphysx_detach_ovstage` destroys the runtime's `AttachedStage` through
`IPhysxSimulation::detachStage`, which routes to
`UsdLoad::detach` → `AttachedStage::releasePhysicsObjects` →
`PhysXUsdPhysicsInterface::releaseAllObjects()`. That last call is the only
thing that drains the physics interface's *pending* per-attach lists.

The pending list that matters here is `PhysXUsdPhysicsInterface::mArticulations`:
it holds `InternalDatabase` record indices for articulations created while the
stage was parsed at attach time, and it is drained only by
`PhysXUsdPhysicsInterface::finishSetup()`, which runs on the first simulation
step after an attach. A caller that attaches a stage declaring articulations and
detaches it *without ever stepping* therefore leaves the list populated. The
record vector those indices address belongs to the attach that just died, so on
the next attach + `ovphysx_step` `finishSetup()` indexes
`InternalPhysXDatabase::getRecords()` with indices from the dead attach and hands
whatever those slots now hold to `addArticulationToScene`, which dereferences
`Record::mInternalPtr` as an `InternalArticulation*`.

Bootstrapped from a bug: under `OVPHYSX_NO_USD=ON` the
`mPhysicsInterface->releaseAllObjects()` call was fenced out of
`AttachedStage::releasePhysicsObjects()` behind
`#if !defined(OVRUNTIME_PHYSX_NO_USD)`, so no detach released engine objects at
all in that configuration. The installed-SDK `c_unittests` `cpu` pass then
segfaulted (exit 139) in `addArticulationToScene` at
`PhysXTestFixture.StepWaitReportsPriorUserTaskFailure` — the first case to attach
and step after `PhysXTestFixture.ScopedSealAllowsUnrelatedUnsealedData` attaches
`two_articulations.usda` and detaches it without stepping — which blocked the
`cpp-unit` CTest label end to end. The default (USD) configuration was never
affected, because there the call was never fenced.

## Acceptance Criteria

The acceptance criteria below are stated on the **ovphysx C-API surface** —
what a caller of `ovphysx_detach_ovstage` can observe. The ovruntime call chain
named above is the mechanism they depend on, not their subject; the ovruntime
side of the same contract is
[REQ-CAPI-DETACH-001](REQ-CAPI-DETACH-001.md)'s ordering rule plus the
`releaseAllObjects()` call site listed under Code References.

- **AC-1: Detach tears the engine down unconditionally.** There is one ovphysx
  build (ADR-0027) and the detach path has no compile-time variant:
  `AttachedStage::releasePhysicsObjects()` calls
  `mPhysicsInterface->releaseAllObjects()` unconditionally, with no fence around
  it. *(Amended 2026-09-01: the original "identically in the default wheel and in
  the `OVPHYSX_NO_USD=ON` wheel" differential is moot — there is no second
  wheel.)*
- **AC-2: No pending articulation survives a detach.** After
  `ovphysx_detach_ovstage` returns, a subsequent `ovphysx_attach_ovstage` +
  `ovphysx_step` never resolves an articulation record minted by the destroyed
  attach — whether or not that attach was ever stepped.
- **AC-3: No pending particle system survives a detach.** The same holds for
  particle systems: a later attach's first step resolves none of the destroyed
  attach's particle-system records.
- **AC-4: Attach → detach → attach → step is crash-free.** Attaching a stage that
  declares articulations, detaching it without ever calling `ovphysx_step`,
  attaching a different stage, and stepping that one completes with
  `OVPHYSX_API_SUCCESS` and no fault.

## Test References

- TEST-CAPI-DETACH-002

## Code References

- ovphysx/ovruntime/source/omni.physx/plugins/usdLoad/AttachedStage.cpp
  (`AttachedStage::releasePhysicsObjects` — the unconditional
  `releaseAllObjects()` call AC-1 constrains; in `ovruntime`'s tree because that
  is where the single enforcing call site lives, referenced here because
  `ovphysx_detach_ovstage` is the C ABI entry point that depends on it)
- ovphysx/ovruntime/source/omni.physx/plugins/usdInterface/UsdInterface.cpp
  (`PhysXUsdPhysicsInterface::releaseAllObjects` clears `mArticulations` and
  `mParticleSystems`; `finishSetup`/`finalizeArticulations` and
  `addArticulationToScene` are the consumers AC-2 and AC-3 protect)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_detach_ovstage`)
- ovphysx/tests/c_unittests/test_usd_loading.cpp
  (`PhysXTestFixture.DetachWithoutStepDropsPendingArticulations` — AC-4)

## Dependencies

- [REQ-CAPI-DETACH-001](REQ-CAPI-DETACH-001.md) — the teardown ordering this
  requirement's detach path also has to honor
- [ADR-0018](../../../ovruntime/plc/adr/ADR-0018-usd-free-runtime-build.md) —
  the USD-free build variant whose fencing introduced the regression AC-1 closes
  (origin; superseded)
- [ADR-0027](../../../ovruntime/plc/adr/ADR-0027-usd-parsing-loadable-library.md) —
  one USD-free build; the fence class that caused this bug no longer exists
