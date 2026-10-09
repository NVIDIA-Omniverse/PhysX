<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CONTACT-003
title: Separate Contact Force Components and Compatible Aliases
status: implemented
owner: ovphysx
---

## Description

The C and Python contact bindings expose explicit normal-force reads, an
unfiltered net-friction read, and normal/friction contact data, retaining the
old entry points as deprecated aliases.

## Acceptance Criteria

- AC-1: **Components.** `ovphysx_read_contact_net_normal_forces`,
  `ovphysx_read_contact_net_friction_forces`,
  `ovphysx_read_contact_normal_force_matrix`, and
  `ovphysx_read_contact_friction_force_matrix` are exposed in Python as
  `read_net_normal_forces`, `read_net_friction_forces`,
  `read_normal_force_matrix`, and `read_friction_force_matrix`. Net reads have
  shape `[S, 3]`; matrices have shape `[S, F, 3]` and include only configured
  sensor/filter pairs, independently of detailed-contact capacity.
  Net friction includes reported contacts excluded by
  filters and requires neither filters nor detailed-contact capacity.
- AC-2: **Compatibility.** `ovphysx_read_contact_net_forces`,
  `ovphysx_read_contact_force_matrix`, `ovphysx_read_contact_data`, and
  `ovphysx_read_friction_data`, and their Python counterparts `read_net_forces`,
  `read_force_matrix`, `read_contact_data`, and `read_friction_data`, remain
  aliases of the corresponding explicit component reads.
  C declarations and Python type stubs mark these aliases deprecated; Python
  calls emit `DeprecationWarning` pointing to the replacement name.
  C validation errors retain the operation name of the entry point called,
  including deprecated aliases.
  `ovphysx_read_normal_contact_data` / `read_normal_contact_data` retain the
  detailed normal forces, normals, positions, separations, counts, start indices,
  required capacity, and truncation behavior of the old detailed read.
  `ovphysx_read_friction_contact_data` / `read_friction_contact_data` likewise
  preserve friction forces, anchor points, layouts, required capacity, and
  truncation behavior.
- AC-3: **Read contract.** Component reads use the last successful simulation
  step's dt, validate float32 output shape/device, and reject destroyed or
  invalidated bindings. Their normal and friction outputs for the same step
  can be added to obtain total contact force, per sensor for net reads or per
  sensor/filter pair for matrices; the friction vector is not torque.

- AC-4: **Deprecation status.** All C functions that create or operate on a
  contact-binding handle carry compiler deprecation attributes. This includes
  all six explicit normal/friction read functions. Python type stubs mark
  `ContactBinding`, `PhysX.create_contact_binding`, and those six read methods
  deprecated, and their API documentation states the same status. The Python
  factory emits one caller-attributed `DeprecationWarning` and returns a
  usable binding. Deprecation preserves read results and alias warnings.
  The independent contact-report API is outside this deprecated family.

## Test References

- [TEST-CAPI-CONTACT-003](../../tests/capi/TEST-CAPI-CONTACT-003.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx.h`
- `ovphysx/src/ovphysx/ovphysxContactBinding.cpp`
- `ovphysx/python/ovphysx/_bindings.py`
- `ovphysx/python/ovphysx/api.py`
- `ovphysx/python/ovphysx/api.pyi`
- `ovphysx/tests/c_unittests/test_contact_binding.cpp`
- `ovphysx/tests/python_tests/cpu_tests/test_contact_binding_advanced.py`
- `ovphysx/tests/python_tests/test_tensor_bindings_api_gpu.py`

## Dependencies

None.
