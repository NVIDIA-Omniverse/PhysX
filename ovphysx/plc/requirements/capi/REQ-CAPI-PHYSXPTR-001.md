<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PHYSXPTR-001
title: Pathless PxPhysics Pointer Lookup
status: implemented
owner: ovphysx
---

## Description

`OVPHYSX_PHYSX_TYPE_PHYSICS` identifies the process-wide
`physx::PxPhysics` object, which has no physics-object path.
`ovphysx_get_physx_ptr` therefore uses a zero-length `prim_path` as its
distinguished selector. Both C representations, `{ NULL, 0 }` and
`{ "", 0 }`, have identical meaning.

This exception applies only to `OVPHYSX_PHYSX_TYPE_PHYSICS`. Path-bound
types retain their non-empty path validation, and a non-empty selector
for `PHYSICS` is invalid rather than a failed path lookup.

The returned pointer is borrowed process-global state. Callers must not
release it, must use the matching PhysX SDK headers shipped for the
ovphysx build, and must apply the existing interop lifetime and
between-step access boundaries. Objects that callers explicitly create
through `PxPhysics` follow the PhysX SDK's ownership rules; the
no-release rule applies to the borrowed pointer returned by ovphysx.

Disabling or re-enabling rigid-body simulation through a borrowed actor
pointer is not supported. Callers must use the ovstage `disableSimulation`
attribute (`ovphysx_write()` / `OVPHYSX_ATTR_DISABLE_SIMULATION`). Toggling
`PxActorFlag::eDISABLE_SIMULATION` directly on a
pointer from this API is undefined for DirectGPU read and write: cached GPU
index maps may go stale and subsequent I/O may address the wrong body without
error.

## Acceptance Criteria

- AC-1: With a valid instance and attached, initialized stage, both
  `{ NULL, 0 }` and `{ "", 0 }` with
  `OVPHYSX_PHYSX_TYPE_PHYSICS` return `OVPHYSX_API_SUCCESS` and write a
  non-null pointer.
- AC-2: Both zero-length representations and repeated lookups, including
  across a completed simulation step, return the same `PxPhysics`
  pointer.
- AC-3: A non-zero-length selector with `PHYSICS`, a zero-length selector
  with any non-`PHYSICS` type, or `{ NULL, length > 0 }` returns
  `OVPHYSX_API_INVALID_ARGUMENT` and leaves `out_ptr` null. An embedded-NUL
  selector for `PHYSICS` also returns `OVPHYSX_API_INVALID_ARGUMENT` and
  leaves `out_ptr` null.
- AC-4: The existing attached-stage boundary remains: before attachment,
  after reset, or after direct detachment, the lookup fails and does not
  return a cached pointer.
- AC-5: The experimental C++ `PhysXTypeFor<physx::PxPhysics>` trait maps
  to `OVPHYSX_PHYSX_TYPE_PHYSICS`, and the typed empty-string lookup
  reaches the same public behavior.
- AC-6: Documentation on `ovphysx_get_physx_ptr` and in
  `docs/developer_guide.md` states that toggling `eDISABLE_SIMULATION`
  through a returned actor pointer is unsupported and undefined for DirectGPU
  I/O, and names the official disable path (ovstage `disableSimulation` via
  `ovphysx_write()`).

## Test References

- TEST-CAPI-PHYSXPTR-001
- AC-6 is a documentation obligation, verified by inspecting `ovphysx_get_physx_ptr`'s doc comment
  and `docs/developer_guide.md` — not by an automated test. The behavioral corollary (the official
  disable paths work) is covered cross-tree by `test_directgpu_disabled_body` against
  REQ-READ-CORE-001 AC-8 / REQ-INPUT-CORE-001 AC-10.

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_physx_ptr`)
- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_PHYSX_TYPE_PHYSICS`)
- ovphysx/include/ovphysx/experimental/ovphysx.hpp (`PhysXTypeFor<physx::PxPhysics>`)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`validateInteropArgs`, `ovphysx_get_physx_ptr`)
- ovphysx/src/include/internal/sidecar/ovphysxInternalInterop.h (`ovphysx_internal_get_physx_ptr`)
- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp (`ovphysx_internal_get_physx_ptr`)
- ovphysx/tests/c_unittests/test_physx_ptr.cpp
- ovphysx/tests/c_unittests/test_cpp_wrapper_comprehensive.cpp
- ovphysx/tests/c_unittests/test_directgpu_disabled_body.cpp
- ovphysx/docs/developer_guide.md (`PhysX Pointer Interop`)

## Dependencies

- [ADR-0027](../../../ovruntime/plc/adr/ADR-0027-rigid-body-disable-notification.md) — official disable paths vs raw-pointer toggle
