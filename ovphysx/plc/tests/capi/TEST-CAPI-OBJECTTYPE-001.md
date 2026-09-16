<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OBJECTTYPE-001
maps_to: REQ-CAPI-OBJECTTYPE-001
type: integration
---

## Scenario

Standalone maximal-coordinate joints classify as `JOINT`, plugin-registered custom
joints classify as `CUSTOM_JOINT`, absent paths stay `INVALID`, and articulation
joints stay `ARTICULATION_JOINT`. Regression for NVBugs 6560084.

## Given

- A valid ovphysx instance with CPU dynamics enabled.
- `standalone_prismatic_joint.usda` loaded: standalone prismatic joint at
  `/World/Anchor_Slide` between two plain rigid bodies.
- `revolute_joint_scene.usda` loaded: standalone revolute joint at
  `/World/revoluteJoint`.
- `links_chain_sample.usda` loaded: articulation joint and articulation root controls.
- Custom-joint end-to-end classification is covered in ovruntime
  `TestCustomRegistration.cpp` (requires `IPhysxCustomJoint` registration).

## When

- `ovphysx_get_object_type` is called on each joint path without an explicit sim
  step beforehand.
- The same paths are queried through Python `PhysX.get_object_type`.
- A nonexistent prim path is queried on the prismatic scene.

## Then

- Standalone prismatic and revolute joint paths return `OVPHYSX_OBJECT_TYPE_JOINT` /
  `ObjectType.JOINT` (AC-2).
- The nonexistent path returns `INVALID` (AC-3).
- An articulation joint path returns `ARTICULATION_JOINT`, not `JOINT` (AC-3).
- Python enum exposes `ObjectType.JOINT == 6` and `ObjectType.CUSTOM_JOINT == 7`
  (AC-1). Runtime custom-joint classification is asserted in
  `TestCustomRegistration.cpp` (AC-4).
