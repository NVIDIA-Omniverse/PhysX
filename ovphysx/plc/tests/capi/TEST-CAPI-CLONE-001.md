<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CLONE-001
maps_to: REQ-CAPI-CLONE-001
type: integration
---

## Scenario

- `test_tensor_bindings.py::test_tensor_binding_explicit_paths` mixes existing,
  missing, and repeated paths and checks the resulting count and order (AC-1).
- `test_clone_large_batch_gpu.py::test_large_clone_batch_survives_reset_cycle`
  clones rigid bodies and articulations, resets, reloads, and repeats. Both
  cycles check the exact original-plus-clone count after stepping (AC-2, AC-3).
- `test_tensor_bindings_api_gpu.py` and `cpu_tests/test_tensor_bindings_api.py`
  verify articulation metadata, DOF and body properties, tensor reads/writes,
  and inverse dynamics through independently constructed bindings (AC-3).
- `test_clone_prebinding_gpu.py` and the lifecycle tests for cache staleness,
  destruction during reads, and binding-handle isolation cover invalidation and
  teardown (AC-3). CPU-mode tests run in a separate process from GPU tests.
- `TensorBindingCpuTest.MultipleSamePatternBindings` counts native allocations
  for three attributes of one articulation: one view, three distinct handles.
  `DuplicateBindingSameType` reads through a surviving binding after destroying
  its peer. `CpuArticulationCentroidalMomentumFixedBaseRejected` retains a position
  binding while rejecting unsupported centroidal momentum on its shared view
  (AC-4). The existing stale-metadata and pre-clone-binding tests cover reuse
  across attach changes and clone invalidation.

## Performance Verification

Use a matched 4096-environment Isaac Lab Kuka Allegro lift workload with 16
object variants, GPU native cloning, no renderer, the same timestep and solver
settings, and warm compilation/asset caches. Measure environment creation
separately from runtime; discard 20 warmup steps before timing 100 steps.
Report the native build and Isaac Lab commit with both measurements. Profiling
should attribute the startup reduction to candidate-path reads, destination-key
reuse, and fewer articulation-entry constructions, not reduced scene content.
Attribute binding time should fall when native selections are shared (AC-4).
