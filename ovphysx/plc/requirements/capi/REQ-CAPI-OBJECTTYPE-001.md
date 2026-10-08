<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OBJECTTYPE-001
title: TensorAPI Object Type Classification Includes Standalone and Custom Joints
status: implemented
owner: ovphysx
---

## Description

`ovphysx_get_object_type` classifies authored prim paths by the TensorAPI object
taxonomy mirrored in `ovphysx_object_type_t` and Python `ObjectType`. Maximal-coordinate
(standalone) joints -- UsdPhysics joint prims between plain rigid bodies with no
`ArticulationRootAPI` on the subgraph -- simulate as `physx::PxJoint` objects and
are reachable through `ovphysx_get_physx_ptr(..., OVPHYSX_PHYSX_TYPE_JOINT)`, but
previously mapped to `OVPHYSX_OBJECT_TYPE_INVALID`, identical to a genuinely absent
path. Plugin-registered custom joints resolve through
`OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT` and suffered the same classification gap.

The public enum therefore exposes `OVPHYSX_OBJECT_TYPE_JOINT` (value 6) and
`OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT` (value 7), mirroring the existing split in
`ovphysx_physx_type_t`. Articulation joints remain
`OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT`.

## Acceptance Criteria

- AC-1: `ovphysx_object_type_t` and Python `ObjectType` include `JOINT` (6) and
  `CUSTOM_JOINT` (7), appended after `ARTICULATION_JOINT` (5).
- AC-2: On a correctly authored standalone joint prim whose path resolves to
  `physx::PxJoint`, `ovphysx_get_object_type` returns `OVPHYSX_OBJECT_TYPE_JOINT`
  with `OVPHYSX_API_SUCCESS`. This holds for maximal-coordinate joint schemas
  generally (for example prismatic and revolute), not a single schema variant.
- AC-3: Genuinely absent paths still return `OVPHYSX_OBJECT_TYPE_INVALID` with
  `OVPHYSX_API_SUCCESS`. Articulation joints continue to classify as
  `OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT`, distinct from `JOINT`.
- AC-4: When a prim path resolves to a live custom joint through
  `OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT`, `ovphysx_get_object_type` returns
  `OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT` with `OVPHYSX_API_SUCCESS`, distinct from
  `JOINT` and `INVALID`.

## Test References

- TEST-CAPI-OBJECTTYPE-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_object_type_t`)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_object_type`)
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (`ovphysx_get_object_type`)
- ovphysx/python/ovphysx/types.py (`ObjectType`)
- ovphysx/python/ovphysx/api.py (`PhysX.get_object_type`)
- ovphysx/tests/c_unittests/test_tensor_binding.cpp (`CpuGetObjectType`, `CpuGetObjectTypeStandalonePrismaticJoint`, `CpuGetObjectTypeStandaloneRevoluteJoint`)
- ovphysx/tests/python_tests/cpu_tests/test_object_type_joint.py
- ovphysx/ovruntime/source/omni.physx/tests/test.unit/physics/TestCustomRegistration.cpp (AC-4 runtime coverage)

## Dependencies

- REQ-TENSOR-OBJECTTYPE-001 (runtime `getObjectType` classification)
