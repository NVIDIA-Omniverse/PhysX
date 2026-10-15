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

- AC-3: `ovphysx_is_stage_attached(instance, stage, out_attached)` and Python
  `PhysX.is_stage_attached()` report native state without exposing a runtime
  attachment identity or collecting task outcomes. The C call rejects a null
  output or invalid instance with `OVPHYSX_API_INVALID_ARGUMENT`, setting a
  nonnull output false first. A null candidate stage means any attachment; a nonnull candidate must match
  the caller-owned stage pointer, without dereferencing it. Native transitions
  are visible to Python. This boolean is an immediate ownership snapshot, not
  a readiness or resource-validity token. It neither creates a task nor drains
  pending work. Callers wait for a lifecycle task before querying its settled
  state; false during a pending attach does not permit destroying the Stage.
- AC-4: Repeated detach is a successful no-op. Teardown drains pending work
  without consuming its separately collectible failures or resetting accumulated
  simulation time, and never tears down a peer's attachment. Python releases
  stage-bound read handles before detach and retains borrowed numeric storage.
  Detach returns a task receipt, including for an already-detached instance;
  reset is removed. Python retains its Stage keepalive through task completion
  and releases it only after native state confirms teardown. A failed teardown
  can retain ownership and retire some resources; it does not restore poses.
- AC-5: Invalid-instance and companion-transition rejection occur before
  retiring writes, SDF views, tensors or the attachment. Once admitted, teardown
  failures are retained in the task outcome. Destruction and failed-attach
  rollback use the private teardown path without creating nested detach tasks.
- AC-6: The binding-only `ovphysx_internal_is_query_valid(instance, query,
  out_valid)` reports whether a retained ordinary output query's original attachment remains
  live. A retired attachment returns success with false, including after native
  detach/reattach of the same Stage; an unknown, released or foreign query returns
  `OVPHYSX_API_NOT_FOUND`. A null output or invalid instance returns
  `OVPHYSX_API_INVALID_ARGUMENT`; every nonnull output is initialized false before
  validation. The call drains pending execution without collecting task outcomes
  and preserves the existing callback-reentrancy rejection. The export has no
  public header declaration. Public `ovphysx_query_shared_dictionary` and
  `PhysX.query_shared_dictionary()` are removed without adding a public query
  validity accessor.

## Test References

- TEST-CAPI-DETACH-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_detach_ovstage`, `ovphysx_is_stage_attached`, private teardown)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_internal_is_query_valid`)
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/SimulationBackend.cpp (`resetStage`'s
  ordering contract; owned by `ovphysx/ovruntime/plc/`'s
  [REQ-TENSOR-VIEW-001](../../../ovruntime/plc/requirements/tensors/REQ-TENSOR-VIEW-001.md) AC-9,
  referenced here for the boundary this sidecar must honor)
- ovphysx/ovruntime/source/omni.physx/tests/test.unit/physics/TestOvstageStageless.cpp (the
  correct order, exercised at the `ovruntime` layer)

- ovphysx/python/ovphysx/api.py (`is_stage_attached`, `_is_query_valid`, lifecycle keepalive handling)
- ovphysx/python/ovphysx/_bindings.py and ovphysx/python/ovphysx/api.pyi (binding and typing)
- ovphysx/tests/c_unittests/test_multi_instance.cpp (private query validity and task outcomes)
- ovphysx/tests/python_tests/test_session_operations.py (removed public accessor and native symbol)

## Dependencies

- [REQ-TENSOR-VIEW-001](../../../ovruntime/plc/requirements/tensors/REQ-TENSOR-VIEW-001.md) AC-9
  — the ordering contract this requirement's C ABI caller must honor
