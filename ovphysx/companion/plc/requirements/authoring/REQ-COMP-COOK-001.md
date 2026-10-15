<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-COOK-001
title: "Cooking jobs and compatible cache results"
status: implemented
owner: ovphysx
---

## Description

Provide raw-data cooking and collision representations for collision overlays, validation, immediate graph geometry and deformable construction, without exposing private cache/context management.

## Acceptance Criteria

- AC-1: **Inputs.** Cooking accepts validated raw geometry and explicit approximation parameters without a live attach, and advertises triangle, convex, SDF, decomposition and tetrahedral operation availability separately.
- AC-2: **Results.** Jobs return owned cooked blobs or versioned collision-representation arrays with diagnostics and progress; a native cooked-object pointer is not a persistent format.
- AC-3: **Cache identity.** Cache keys include geometry, topology, parameters and relevant SDK/cooking format version; incompatible blobs are rejected or recooked, never silently reused.
- AC-4: **Invalidation.** Cache eviction respects active result leases and job cancellation; invalidation is scoped unless explicitly requested as global and never exposes CUDA context destruction to clients.

## Implementation scope

The separate cooking table exposes CPU triangle, convex, decomposition, SDF,
sphere-fill, conforming-tetrahedron and voxel-tetrahedron operations. It retains
existing runtime services independently of its bootstrap core. Dispatch holds
a shared authoring admission for the synchronous kernel. Only recording transport
transitions exclude that admission; kernels do not hold configuration or live-owner
execution locks. Independent playback, preview and attachment remain usable. Python jobs
own copied input, cancellation suppresses delivery, and service close drains work.
Result arrays/blobs have independent ownership. Per-service LRU caches include
geometry, topology, normalized parameters, frame/units and SDK fingerprint;
invalidation prevents in-flight cache insertion. Checksummed archives reject
incompatible SDK/format/recipe data. GPU cooking is not advertised. Linux x86_64
CPU execution is qualified locally; other platform qualification remains a CI task.

## Test References

- [TEST-COMP-COOK-001](../../tests/authoring/TEST-COMP-COOK-001.md) - executable native CPU cooking, lifetime, cache and cancellation coverage.

## Code References

- [Dedicated cooking table](../../../include/ovphysx_companion/cooking.h)
- [Native bridge](../../../src/Cooking.cpp)
- [Python jobs, arrays and cache](../../../python/ovphysx_companion/cooking.py)
- [Collider overlay sample](../../../samples/cooking.py)

Existing native kernels:

- [ovphysx/ovruntime/include/omni/physx/IPhysxCooking.h](../../../../ovruntime/include/omni/physx/IPhysxCooking.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h](../../../../ovruntime/include/omni/physx/IPhysxCookingService.h)

## Dependencies

- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)

## Recipe and representation limits

Validate only fields consumed by each recipe. Python rejects an irrelevant option
and hashes only that recipe's normalized parameters. Runtime dispatch owns native
authoring admission; the companion bridge owns ABI validation and result handles.
Simplified triangle cooks export blobs without the pre-simplification surface;
convex/decomposition/SDF blobs likewise make no cooked triangle-overlay promise.
An unavailable surface must be reported explicitly rather than drawing input data.

