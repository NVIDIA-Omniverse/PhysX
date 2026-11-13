<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-OUTPUT-001
maps_to: REQ-CAPI-OVSTAGE-OUTPUT-001
type: integration
---

## Scenario

An application publishes completed native physics poses to its attached stage,
composed with the signed world scale ovphysx stored per body, while owning the
cache and output ordinals.

## Given

- A populated and attached ovstage; its sealed world matrices serve only as the
  test's reference for the parser's scale decomposition.
- OVStage returns float64 scalar-lane matrices with component axes `[N, 4, 4]`;
  the test reader normalizes descriptor shape and row stride.
- CPU rigid bodies in two physics scenes, including a reflected and rotated
  body, a sheared body, and a body below a scaled parent.
- The same public helper is exercised with articulation links, one of them
  scaled `(2, 2, 2)`, a mixed fixed body and point-instancer scene, and an
  empty scene.
- A separate GPU fixture enables `suppressReadback`, verifies native CUDA
  publication of a reflected and a nested-scaled body and of scaled
  articulation links, and also exercises CPU poses over a GPU-authored stage
  matrix.
- A CPU scene with two stationary character controllers.
- For OMPE-120176, a kinematic body under a parent Xform, under a translated,
  rotated and scaled ancestor, in a zero-gravity CPU scene.

## When

- The application steps, publishes to a fresh output ordinal, seals it, and
  reads the resulting world matrices.
- Calls alternate between an explicit cache, no cache, and a refreshed cache;
  the application writes a different scale into a body's stage world matrix
  before each CPU frame.
- The application attempts publication with invalid arguments, a stale cache
  after reattachment, or an output ordinal at or below the sealed write floor,
  and publishes a body whose stage world matrix was deleted.
- An unrelated stage containing the same body paths and source matrices is
  passed with no cache and with a fresh cache.
- The GPU caller temporarily has no current CUDA context, then a real foreign
  context, and publishes cached, uncached, and refreshed output; test assertions
  read GPU rigid-body and articulation-link results back to CPU.
- With CPU physics poses, the application authors a world matrix from CUDA
  storage and calls the helper under null and foreign caller contexts.
  Repeated CPU publication uses a populated cache while overwriting
  GPU-backed destination matrices.
- The controller scene is published at two increasing output ordinals, first
  without a cache and then with one.
- After publishing and sealing the kinematic body's pose, the application
  writes a translation to the parent's `omni:xform` at the next ordinal, seals
  it, drains it into physics and steps.

## Then

- Every fixed body in both CPU scenes and every articulation link receives a
  matrix; instancer groups are counted while the fixed body still publishes
  (REQ AC-1).
- The read `worldScale` of every body and link equals
  `omni::physx::decomposeMatrix` of its initial stage world matrix: the
  reflected body reads `(-2, -3, -4)`, the nested body `(1, 6, 1)`, the scaled
  link `(2, 2, 2)`. Each published matrix matches the native physics pose
  combined with that scale; publication uses float64, 16-lane MATRIX input and
  readback exposes the same matrix components through the normalized read
  layout (REQ AC-2).
- Stage world-matrix scale edits never reach the output, and a body whose
  stage world matrix was deleted is published with its stored scale
  (REQ AC-2).
- Each stationary controller keeps all 16 components of its initial world
  matrix after both publications (REQ AC-2).
- Publication leaves the write floor unchanged and does not advance the
  simulation; an output ordinal at or below the sealed write floor fails
  explicitly with zero completed matrices (REQ AC-3).
- Cached, uncached and refreshed calls publish identical matrices, and refresh
  cannot permit stale attachment reuse (REQ AC-4).
- On DirectGPU, `worldScale` is read as a `kDLCUDA` column; rigid bodies and
  articulation links keep reflected nonuniform and link scale with physics
  poses through repeated writes, and the calling thread regains its original
  null or foreign context after each helper call (REQ AC-2, REQ AC-5).
- CPU poses published over GPU-authored stage matrices keep the stored scale
  and restore the null or foreign caller context, including cached CPU writes
  into GPU-backed destination columns that can invoke CUDA inline (REQ AC-2,
  REQ AC-5).
- Stale attachment failures report zero completed matrices and nonempty error
  text; empty output succeeds with zero counts (REQ AC-6).
- An unrelated stage is rejected without changing its matrices or binding the
  cache; that cache remains usable with the attached stage (REQ AC-4, REQ AC-6).
- After the parent edit, the parent's world translation is `(42, 6, 0)`. The
  published body follows it to world `(42, 6, 8)` and keeps its authored local
  translation `(0, 0, 4)`; its physics pose after the drain and step is also
  `(42, 6, 8)` (REQ AC-7).
- The complete C++17 sample compiles against the installed SDK without CUDA
  or PhysX SDK headers and performs five caller-stepped, caller-sealed frames
  (REQ AC-1, REQ AC-3, REQ AC-4).

## Test locations

- `ovphysx/tests/c_unittests/test_ovstage_output.cpp`:
  `OvStageOutputTest.*` (including
  `OvStageOutputTest.CharacterControllersPublishPrimOrientation`),
  `OvStageOutputGpuTest.DirectGpuCachedAndUncachedMatricesUseDeviceScaleAndCallerContext`,
  `OvStageOutputGpuTest.DirectGpuArticulationLinksMatchPoseAndStoredScale`,
  `OvStageOutputGpuTest.DirectGpuPublicationRestoresForeignCallerContext`, and
  `OvStageOutputGpuTest.CpuPublicationOverGpuAuthoredMatricesKeepsStoredScaleAndCallerContext`.
- `ovphysx/tests/c_samples/ovstage_output_cpp/main.cpp`:
  `publishFrames` and installed-SDK executable `ovstage_output_cpp`.
- Fixtures: `ovphysx/tests/data/character_controller.usda`, `ovphysx/tests/data/ovstage_output_bodies.usda`,
  `ovstage_output_bodies_gpu.usda`, `ovstage_output_instancer.usda`,
  `ovstage_output_empty.usda`, `ovstage_output_parent_edit.usda`,
  `two_articulations_scaled.usda`, and existing
  `boxes_falling_on_groundplane.usda`, `boxes_falling_on_groundplane_gpu.usda`,
  and `simple_physics_scene_cpu.usda`.

## Coverage limits

The GPU cases exercise primary-context execution on the selected device;
private-producer portability is outside the supported contract. Multiple-device
preflight and the documented partial-publication failure contract remain
source-inspection checks. Null and foreign calling-thread contexts are covered.
The REQ AC-6 failure before any write for a missing or malformed pose or
`worldScale` column, or for nonfinite values in one, is also a
source-inspection check: a public-API test cannot readily make physics output
missing, malformed or nonfinite.
