<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CLONE-001
title: Reuse Native Clone and Articulation Metadata
status: implemented
owner: ovphysx
---

## Description

Replicated-scene initialization must avoid redundant stage queries, destination
path resolution, and articulation metadata construction without changing object
selection, placement, simulation settings, or resource lifetime.

## Acceptance Criteria

- AC-1: Batched existence checks read the requested path handles directly, not
  through a stage-wide path predicate. Missing and deleted prims remain absent;
  repeated requests preserve their individual results.
- AC-2: Resolve each cloned object's destination key before copying metadata.
  Parallel workers consume those keys without rebuilding paths or taking a
  shared path-resolution lock. Rigid and articulated clones survive reset,
  reload, and a second clone operation.
- AC-3: Articulation bindings reuse the scene-owned metadata already used by
  read sessions. Each receiving view owns its metatype and subspace references.
  Existing scene-cache invalidation remains authoritative; GPU and CPU binding
  values, ordering, metadata, and teardown behavior stay unchanged.
- AC-4: Attributes with identical ordered path patterns and the same native view
  family share valid nonempty selections within one attach. Handles and scratch
  buffers remain independent; destroying one handle leaves the others usable.
  Clone/reset invalidation and detach/reattach prevent stale-view reuse, and
  attribute-specific validation still applies to shared selections.

## Test References

- TEST-CAPI-CLONE-001

## Code References

- `ovruntime/source/omni.physics.ovstage/OvstageSource.cpp`
- `ovruntime/source/omni.physx/plugins/PhysXReplicator.cpp`
- `ovruntime/source/omni.physx/plugins/tensors/base/BaseSimulationView.cpp`
- `src/ovphysx/ovphysxTensorBinding.cpp`
