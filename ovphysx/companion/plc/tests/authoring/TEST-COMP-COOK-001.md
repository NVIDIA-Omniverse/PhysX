<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-COOK-001
maps_to: REQ-COMP-COOK-001
type: integration
---

## Coverage status

Automated CPU coverage passes for [REQ-COMP-COOK-001](../../requirements/authoring/REQ-COMP-COOK-001.md) AC-1 through AC-4. GPU operations are not advertised; other platform execution remains a CI qualification task.

## Scenario

Cook, cache and visualize a collider while editing its topology and retaining an older representation.

## Given

- A convex-capable mesh, a leased first cook result, and cache instrumentation.
- A cooked blob tagged with an incompatible format/SDK version.

## When

- Cook twice, change topology/parameters, request the incompatible blob, then invalidate the cache while the first result remains leased.

## Then

- Supported jobs complete without attach; unsupported approximation modes fail by capability (REQ AC-1).
- Representation arrays reconstruct the expected collider and progress reaches one terminal result (REQ AC-2).
- The identical input may hit cache, changed input misses, and the incompatible blob is rejected or recooked (REQ AC-3).
- The old leased result stays valid through invalidation and is freed after release; no client context-release operation is required (REQ AC-4).

## Executable coverage

[test_cooking.py](../../../tests_native/test_cooking.py), 24 passing cases:

- `test_raw_cooking_families_without_attach`: real convex, triangle, decomposition, SDF and sphere-fill results, triangle arrays and original-polygon map.
- `test_conforming_and_voxel_tetrahedra` and `test_volume_recipe_owns_distinct_meshes_and_visual_embedding`: nondegenerate volumes and reconstructed visual geometry.
- `test_scoped_cache_identity_invalidation_and_result_lifetime`, `test_cache_archive_compatibility_and_corruption`: recipe keys, scoped invalidation, immutable owned arrays and SDK/key/checksum rejection.
- `test_queued_cancellation_copied_inputs_and_cache_epoch`: copied inputs, cancellation, terminal status and in-flight invalidation.
- `test_result_buffer_canaries_and_service_independent_lease`, `test_cooking_table_canaries`: atomic output, owned native result and version/size failure.
- `test_cooking_during_pending_preview_preserves_simulation`: cooking succeeds while Inspector simulation is pending, and the owner fetch completes.
- `test_cooking_outlives_last_core_and_eviction_keeps_results`: independent process lease, eviction and retained data.
- Invalid geometry/options cases reject without publishing a result.

## Review regressions

Simplified triangle results cannot draw input triangles as a cooked overlay;
non-applicable Python recipe options are rejected. Per-kind invalid native options
preserve failure state. The volume recipe reports the 10000000-pair work limit
before mapping dispatch. All polygon and tetrahedral families run through native
runtime admission.


`test_simplified_triangle_archive_roundtrip` cooks a native simplified triangle
blob, saves/loads it with its mode-specific empty topology, and saves/loads again.
