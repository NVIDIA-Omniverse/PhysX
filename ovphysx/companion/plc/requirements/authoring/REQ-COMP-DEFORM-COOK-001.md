<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-DEFORM-COOK-001
title: "Deformable topology and embedding results"
status: implemented
owner: ovphysx
---

## Description

Deformable cooking returns simulation/collision topology, rest data and correspondence independently of USD authoring.

## Implementation scope

Native CPU triangle/tetrahedron mapping and interpolation return copied arrays, topology versions, declared frames/units, signed triangle offsets and explicit unmatched/projection diagnostics. Copied-input asynchronous Python jobs retain their results after close. Conforming and voxel tetrahedralization now produce independent collision/simulation meshes; `cook_volume` builds visual correspondence and identifies incomplete/outside mappings. The native-worker/customer-USD sample constructs these mesh roles with undo/redo. Automatic surface-deformable remeshing and GPU mapping are not advertised or qualified.

## Acceptance Criteria

- AC-1: **Outputs.** Supported deformable cook operations return simulation and collision mesh identities, topology/rest data and required visual embedding or correspondence with explicit topology versions.
- AC-2: **Space conventions.** Each mesh and mapping declares coordinate frame, units, index domain and array layout; simulation, collision and visual vertex counts are not assumed equal.
- AC-3: **Validation.** Invalid topology or unembeddable visual points produce diagnostics identifying affected elements; partial results are labelled and never silently treated as a complete mapping.
- AC-4: **Backend.** Cook and interpolation availability are separately reported for CPU/GPU execution; produced arrays honor job completion and buffer ownership contracts.

## Test References

- [TEST-COMP-DEFORM-COOK-001](../../tests/authoring/TEST-COMP-DEFORM-COOK-001.md) - executed CPU topology generation, mapping/interpolation and customer-USD construction.

## Code References

- [Native table](../../../include/ovphysx_companion/mesh.h)
- [Bridge](../../../src/Mesh.cpp)
- [Python mapping records](../../../python/ovphysx_companion/mesh.py)
- [Jobs](../../../python/ovphysx_companion/jobs.py)
- [Runnable sample](../../../samples/mesh_mapping.py)
- [Native topology cooking](../../../src/Cooking.cpp)
- [Volume recipe and mapping](../../../python/ovphysx_companion/deformable_cooking.py)
- [Native-worker authoring sample](../../../samples/deformable_cooking.py)

Existing integration anchors for topology generation:

- [ovphysx/ovruntime/include/omni/physx/IPhysxCooking.h](../../../../ovruntime/include/omni/physx/IPhysxCooking.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h](../../../../ovruntime/include/omni/physx/IPhysxCookingService.h)
- [ovphysx/python/ovphysx/utils/deformable.py](../../../../python/ovphysx/utils/deformable.py)

## Dependencies

- [REQ-COMP-COOK-001](REQ-COMP-COOK-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0005](../../adr/ADR-0005-attachment-and-deformable-split.md)

## Embedding work limit

The stopped volume recipe rejects visual-point count times simulation-tetrahedron
count above 10000000 before embedding. Report the workload and advise reducing
voxel resolution or visual mesh complexity; do not silently chunk or weaken the
native bound.
