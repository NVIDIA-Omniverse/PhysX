<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PHYSXPTR-001
title: Native PhysX Pointers Through Reads
status: implemented
owner: ovphysx
requirement: REQ-CAPI-PHYSXPTR-001
---

## Environment

- A stepped ovphysx instance with an attached USD scene.
- Fixtures: `boxes_falling_on_groundplane.usda`, `kinematic_support.usda`,
  `two_articulations.usda`, `revolute_joint_scene.usda`,
  `convex_decomposition_shape.usda`.
- GPU particle coverage uses `particles_simple.usda` when GPU tests are enabled.

## When

- **Physics row (AC-1).** `test_physx_ptr.cpp` reads `OVPHYSX_QUERY_TARGET_PHYSICS`
  and checks one non-null address whose path is `/`, then checks the
  same address after a step.
- **Path-bound kinds (AC-2, AC-3, AC-5).** The same file reads scene, rigid
  actor, shape, material, articulation, link, articulation joint, standalone
  joint, and compound shape. A missing actor path and an articulation joint
  queried as a standalone joint are absent. Distinct cubes and distinct
  articulations have distinct addresses. A scene address is unchanged across
  ten steps.
- **No column (AC-3).** A rigid-body read of `physxPointer` returns no rows.
- **Lifetime (AC-4).** Before attach the physics row is absent. Detach makes it
  absent again, including after a second attach/detach cycle.
- **Documentation (AC-6).** Inspect `docs/developer_guide.md`.

## Then

- Live objects yield a non-null host `uint64`.
- Missing objects and unsupported query types yield no pointer row.
- Detach stops the address from being reacquired.
