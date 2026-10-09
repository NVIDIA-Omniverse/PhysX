<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BINDING-SELECTION-001
maps_to: REQ-CAPI-BINDING-SELECTION-001
type: integration
---

## Scenario

Repeated tensor-binding creation shares compatible native selections without retaining
outdated membership or coupling public handle lifetimes.

## Given

The C++ cases use `ovphysx/tests/c_unittests/test_tensor_binding.cpp`:

- `TensorBindingCpuTest.MultipleSamePatternBindings` selects one articulation for
  position, velocity and position targets while a scoped probe counts native
  simulation-view creation.
- `TensorBindingCpuTest.DuplicateBindingSameType` creates two equivalent bindings.
- `TensorBindingCpuTest.NewBindingIncludesAddedBody` creates a pose binding for one
  realized body, then adds another body through ovstage and applies the changes.
- `TensorBindingGpuTest.NewBindingIncludesReenabledBody` creates a pose binding while
  one matching GPU body is disabled, then re-enables it through a session write.
- `TensorBindingGpuTest.HostPropertiesSurviveSiblingDisableAndRowRefresh` creates
  mass, pose and disable bindings with identical patterns, then disables one body
  and attempts a pose read through the older GPU mapping.
- `TensorBindingCpuTest.CpuArticulationCentroidalMomentumFixedBaseRejected` keeps a
  position binding alive before requesting unsupported centroidal momentum.

Python coverage uses `test_tensor_binding_explicit_paths` in
`ovphysx/tests/python_tests/test_tensor_bindings.py` and
`test_preclone_binding_velocity_reaches_all_envs` in
`ovphysx/tests/python_tests/test_clone_prebinding_gpu.py`.

## When

The tests create sibling bindings, destroy one sibling, add or re-enable matching
objects before creating another binding, request an unsupported tensor attribute,
or reorder an explicit path list containing missing and repeated entries.
The clone case keeps a pre-clone binding alive while creating a post-clone binding.

## Then

- Three compatible attributes allocate one native simulation view and return three
  distinct handles with matching object and DOF dimensions (AC-1, AC-2).
- Destroying one duplicate leaves a successful tensor read through its sibling
  (AC-2).
- Host mass reads still succeed with unchanged values after both a sibling disable
  write and the subsequent GPU pose-mapping invalidation. The pose binding remains
  valid until its first GPU row refresh (AC-2).
- The newly created binding includes two bodies after addition or 11 bodies after
  GPU re-enable; the older bindings retain their original counts of one and 10
  respectively and still return finite poses through successful reads (AC-3).
- The post-clone binding includes every cloned environment (AC-3).
- Centroidal momentum on a fixed-base articulation is rejected while the existing
  position binding remains usable (AC-4).
- Each explicit path list yields its own requested order, skips the absent path and
  includes a repeated body only once (AC-4).

The cross-instance/attachment/family rejection and final native-resource release
structure are verified by source inspection. Detach/reattach rejection is additionally
covered by TEST-CAPI-BINDING-STALE-001. These tests do not establish a performance gain.
