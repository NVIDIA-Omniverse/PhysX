<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PHYSXPTR-001
maps_to: REQ-CAPI-PHYSXPTR-001
type: integration
---

## Scenario

The public PhysX pointer API retrieves the pathless process-wide
`PxPhysics` pointer without weakening validation for path-bound object
types. Regression for NVBug 6530143.

## Given

- A valid ovphysx instance with a rigid-body stage attached and stepped.
- The same valid instance before a stage is attached.
- Zero-length selectors `{ NULL, 0 }` and `{ "", 0 }`.
- A non-empty selector, an embedded-NUL selector, a malformed
  `{ NULL, 1 }` selector, and a path-bound `SCENE` lookup.
- The experimental C++ typed `physx::PxPhysics` accessor.

## When

- `ovphysx_get_physx_ptr` queries `PHYSICS` with each zero-length
  representation.
- The lookup is repeated after a completed simulation step.
- `PHYSICS` is queried with a non-empty selector.
- A non-`PHYSICS` type is queried with a zero-length selector.
- `PHYSICS` is queried with the embedded-NUL selector.
- The malformed selector is submitted.
- The pathless lookup is attempted before attachment, after reset, and
  after direct detachment.
- The C++ wrapper queries `physx::PxPhysics` with an empty string.

## Then

- Both zero-length `PHYSICS` calls succeed and return the same non-null
  pointer (AC-1, AC-2).
- The pointer remains identical after the completed step (AC-2).
- Every invalid selector/type combination returns
  `OVPHYSX_API_INVALID_ARGUMENT` with null output (AC-3).
- The pre-attachment, post-reset, and post-detachment lookups fail with
  null output, and a previously returned pointer is never dereferenced or
  released after a lifecycle boundary (AC-4).
- The C++ trait maps to `OVPHYSX_PHYSX_TYPE_PHYSICS`, and the typed lookup
  succeeds with a non-null pointer (AC-5).
- `ovphysx_get_physx_ptr` and `docs/developer_guide.md` document that toggling
  `eDISABLE_SIMULATION` on a returned actor pointer is unsupported and undefined
  for DirectGPU I/O, and name the official disable paths (AC-6). DirectGPU
  disable/read/write coverage through the official ovstage path lives in
  `test_directgpu_disabled_body.cpp`.
