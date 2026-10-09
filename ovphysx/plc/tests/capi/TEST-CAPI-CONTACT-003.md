<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONTACT-003
maps_to: REQ-CAPI-CONTACT-003
type: integration
---

## Scenario

Public contact bindings expose separate force components and preserve the
results of deprecated aliases.

## Given

Cube1 resting on BigBase in `boxes_falling_on_groundplane.usda`, with contact
bindings of detailed capacity zero. Python also creates a ground-plane filter
that excludes BigBase and an unfiltered binding. GPU Python cases use CUDA
output tensors and filtered bindings with detailed capacities 0, 1, and 64.
These bindings share one scene and settling sequence. Validation checks share
one binding per device, exercise all four methods, and then verify that all
four reject the destroyed binding. Failures identify the method or capacity.

## When

- C reads normal, friction, and both force matrices through the current functions.
- Python gives Cube1 tangential velocity, steps, and reads both components,
  including net friction from the binding whose filter excludes the support.
- Invalid output descriptors and destroyed bindings are used.

## Then

- Normal support is positive and friction is separate (AC-1).
- Python friction opposes sliding and agrees across unfiltered and excluded
  filter bindings. Both excluded pair matrices are zero, while the support
  pair's friction matrix equals net friction with detailed capacity zero (AC-1).
- C rejects null, wrong dtype, wrong shape, and wrong-device outputs for net
  friction and the friction matrix. C and Python reject destroyed bindings,
  and Python rejects wrong shapes, including a nonempty matrix without filters.
  Both Python matrix reads accept an empty matrix when no filters are configured
  (AC-3).
- All four Python force methods reject incorrect output dtype and shape and
  destroyed bindings on CPU and GPU. GPU bindings also reject host output.
  A valid call succeeds after rejected output descriptors (AC-3).
- On GPU, both net components agree across unfiltered, support-filtered, and
  ground-filtered bindings. The support matrix equals the corresponding net
  force, and the excluded ground matrix is zero at every tested capacity.
  Normal force has no tangential component and friction has no normal component
  for the horizontal support (AC-1).

## Implementation

- `ovphysx/tests/c_unittests/test_contact_binding.cpp`:
  `ContactBindingTest.ForceComponents`.
- `ovphysx/tests/python_tests/cpu_tests/test_contact_binding_advanced.py`:
  `test_force_components`, `test_contact_component_validation`.
- `ovphysx/tests/python_tests/test_tensor_bindings_api_gpu.py`:
  `TestContactBindingGpu.test_force_components`,
  `TestContactBindingGpu.test_component_validation`.

## Detailed Force Components

- `FilteredReadsOverflowReturnsRequiredCountAndValidPrefix` in the C test file
  uses current APIs to verify required counts and valid truncated layouts.
- `test_detailed_contact_data_force_components` in the Python test file uses
  capacities 1 and 64. Complete normal data reconstructs the normal force matrix
  by summing scalar force times contact normal. Friction opposes sliding; the
  friction matrix equals net friction at both capacities for the sensor whose
  filter selects its only partner. Complete anchor forces sum to that matrix
  entry (AC-1).

## Deprecated Alias Compatibility

- C `DeprecatedAggregateContactAliases` and Python
  `test_deprecated_aggregate_contact_aliases` compare the old net-normal and
  matrix reads with their current replacements (AC-2).
- C `DeprecatedDetailedContactAliases` and Python
  `test_deprecated_detailed_contact_aliases` compare all normal/friction data
  buffers, layouts, and required counts at capacities 1 and 64. C status codes
  agree for complete and truncated reads. Python also verifies the replacement
  name in each deprecation warning (AC-2).
- The C compatibility cases pass invalid dtypes to current and deprecated
  aggregate and detailed reads. Each error names the requested C operation,
  including the original name for a deprecated alias (AC-2, AC-3).
- These compatibility cases retire with the aliases. Component behavior tests
  use the explicit component methods within the deprecated contact-binding family.


## Contact-Binding Deprecation

- `test_contact_binding_factory_deprecation` creates a binding, checks exactly
  one caller-attributed deprecation warning, reads zero forces before the first
  step, and destroys the binding (AC-4).
- `test_contact_component_deprecation_annotations` checks the shipped type stub
  for `ContactBinding`, its factory, and all six component reads. It also checks
  their runtime docstrings for the deprecation notice (AC-4).
- Manual compiler validation references every C contact-binding function with
  deprecated-declaration warnings enabled and confirms a diagnostic for each.
  Including the C header without calling these functions remains valid (AC-4).
