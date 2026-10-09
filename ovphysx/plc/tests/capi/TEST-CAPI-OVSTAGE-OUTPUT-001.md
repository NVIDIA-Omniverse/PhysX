<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-OUTPUT-001
maps_to: REQ-CAPI-OVSTAGE-OUTPUT-001
type: integration
---

## Scenario

An application publishes completed native physics poses to its attached stage,
preserving signed world scale while owning the cache and output ordinals.

## Given

- A populated and attached ovstage, with current sealed world matrices.
- CPU rigid bodies in two physics scenes, including a reflected and rotated
  body, a sheared body, and a body below a scaled parent; hierarchy computation
  is completed and sealed for the nested body before publication.
- The same public helper is exercised with articulation links, a mixed fixed
  body and point-instancer scene, and an empty scene.
- A separate GPU fixture enables `suppressReadback`, verifies native CUDA pose
  publication, and also exercises CPU poses with a GPU-authored source matrix.

## When

- The application steps, publishes to a fresh output ordinal, seals it, and
  reads the resulting world matrices.
- Calls alternate between an explicit cache, no cache, and a refreshed cache;
  the application changes a world matrix's scale between CPU frames.
- The application attempts publication with invalid arguments, a stale cache
  after reattachment, a missing source matrix, or an unsealed prior output.
- An unrelated stage containing the same body paths and source matrices is
  passed with no cache and with a fresh cache.
- The GPU caller temporarily has no current CUDA context, then a real foreign
  context, and publishes cached, uncached, and refreshed output; test assertions
  read GPU rigid-body and articulation-link results back to CPU.
- With CPU physics poses, the application authors a world matrix from CUDA
  storage and calls the helper without an intervening stage read, under null
  and foreign caller contexts.

## Then

- Every fixed body in both CPU scenes and every articulation link receives a
  matrix; instancer groups are counted while the fixed body still publishes
  (REQ AC-1).
- Each CPU matrix matches the native physics pose combined with signed scale
  from `omni::physx::decomposeMatrix`; nested world scale is `(1, 6, 1)`, and
  matrix values have float64, 16-lane MATRIX storage (REQ AC-2).
- Publication leaves the write floor unchanged and does not advance the
  simulation; a subsequent unsealed source read fails explicitly (REQ AC-3).
- The explicit cache retains scale across authored edits, the default path
  reads current scale, refresh recaptures it, and refresh cannot permit stale
  attachment reuse (REQ AC-4).
- DirectGPU rigid bodies and articulation links preserve reflected nonuniform
  scale and physics poses through repeated writes; the calling thread regains
  its original null or foreign context after each helper call (REQ AC-2,
  REQ AC-5).
- CPU poses combined with GPU-authored source matrices preserve signed scale
  and restore the null or foreign caller context (REQ AC-2, REQ AC-5).
- Missing input and stale attachment failures report zero completed matrices
  and nonempty error text; empty output succeeds with zero counts (REQ AC-6).
- An unrelated stage is rejected without changing its matrices or binding the
  cache; that cache remains usable with the attached stage (REQ AC-4, REQ AC-6).
- The complete C++17 sample compiles against the installed SDK without CUDA
  or PhysX SDK headers and performs five caller-stepped, caller-sealed frames
  (REQ AC-1, REQ AC-3, REQ AC-4).

## Test locations

- `ovphysx/tests/c_unittests/test_ovstage_output.cpp`:
  `OvStageOutputTest.*`,
  `OvStageOutputGpuTest.DirectGpuCachedAndUncachedMatricesPreserveScaleAndCallerContext`,
  `OvStageOutputGpuTest.DirectGpuArticulationLinksMatchPoseAndSignedScale`,
  `OvStageOutputGpuTest.DirectGpuPublicationRestoresForeignCallerContext`, and
  `OvStageOutputGpuTest.CpuPosesCaptureGpuAuthoredScaleAndRestoreCallerContext`.
- `ovphysx/tests/c_samples/ovstage_output_cpp/main.cpp`:
  `publishFrames` and installed-SDK executable `ovstage_output_cpp`.
- Fixtures: `ovphysx/tests/data/ovstage_output_bodies.usda`,
  `ovstage_output_instancer.usda`, `ovstage_output_empty.usda`, and existing
  `two_articulations.usda`, `boxes_falling_on_groundplane_gpu.usda`, and
  `simple_physics_scene_cpu.usda`.

## Coverage limits

The GPU cases exercise primary-context execution on the selected device;
private-producer portability is outside the supported contract. Multiple-device
preflight and the documented partial-publication failure contract remain
source-inspection checks. Null and foreign calling-thread contexts are covered.
