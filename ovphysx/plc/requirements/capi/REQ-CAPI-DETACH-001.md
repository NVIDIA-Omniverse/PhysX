<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-DETACH-001
title: ovphysx_detach_ovstage Tears Down Tensor Data Before the Attach
status: implemented
owner: ovphysx
---

## Description

`ovphysx_detach_ovstage` tears down a stage attach by calling into two
independent subsystems: `IPhysxSimulation::detachStage`, which destroys the
runtime's `AttachedStage`, and `TensorApi::resetStage`, which releases the
tensor `SimulationBackend`'s per-attach data for the same attach handle.

`SimulationBackend.cpp` documents the ordering these two calls must observe:
a tensor view holds both a `shared_ptr` to `SimulationBackend`-owned data and
a borrowed, non-owning `AttachedStage*`, so `resetStage` must run *before*
`detachStage` destroys the attach it borrows a pointer into — releasing the
data after the attach it points into is already gone means the release
itself runs against a dangling attach. `ovphysx_detach_ovstage` called them
in the wrong order (`detachStage` before `resetStage`), the inverse of what
`SimulationBackend.cpp` requires and what
`ovruntime`'s own `TestOvstageStageless.cpp` exercises. The bug was latent
rather than crashing in practice, because `BaseSimulationView::invalidate()`
(the consumer of the now-dangling `AttachedStage*`) only nulls its own
pointer fields and never dereferences the dead attach.

## Acceptance Criteria

- AC-1: `ovphysx_detach_ovstage` calls `TensorApi::resetStage(attachHandle)`
  before `IPhysxSimulation::detachStage()`, for every stage that was attached
  (`instanceShared->ovstage_attached`), matching the order
  `SimulationBackend.cpp`'s header comment on `resetStage` requires and
  `TestOvstageStageless.cpp` exercises.
- AC-2: `resetStage` is called unconditionally on every detach with an
  ovstage attach, not gated on a nonzero USD stage id — a stageless attach
  (no backing USD stage) needs the tensor-data release the same as a
  USD-backed one.

## Test References

- TEST-CAPI-DETACH-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_detach_ovstage`)
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/SimulationBackend.cpp (`resetStage`'s
  ordering contract; owned by `ovphysx/ovruntime/plc/`'s
  [REQ-TENSOR-VIEW-001](../../../ovruntime/plc/requirements/tensors/REQ-TENSOR-VIEW-001.md) AC-9,
  referenced here for the boundary this sidecar must honor)
- ovphysx/ovruntime/source/omni.physx/tests/test.unit/physics/TestOvstageStageless.cpp (the
  correct order, exercised at the `ovruntime` layer)

## Dependencies

- [REQ-TENSOR-VIEW-001](../../../ovruntime/plc/requirements/tensors/REQ-TENSOR-VIEW-001.md) AC-9
  — the ordering contract this requirement's C ABI caller must honor
