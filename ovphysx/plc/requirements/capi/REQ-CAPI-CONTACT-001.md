<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CONTACT-001
title: Sensor and Other Actor Identities in read_raw_contact_data
status: implemented
owner: ovphysx
---

## Description

`ovphysx_read_raw_contact_data` returns per-contact force/point/normal/separation plus a
per-sensor `count`/`start_indices` pair and a raw `other_actor_ids` buffer. Building a logical
contact record (which sensor reported which other actor, for a given contact) requires the
caller to also know the sensor's own actor id — previously unavailable from this call — so
callers had to track sensor identity separately from the output buffers (see OMPE-104131).

This requirement reshapes `ovphysx_read_raw_contact_data` so both actor identities are
returned in one call, aligned 1:1 per contact, and so the quantities that are only
meaningful in pairs travel as columns of one tensor rather than as separate buffers the
caller must keep in step: `sensor_layout_tensor` `[S, 2]` (count, start index) and
`actor_ids_tensor` `[C, 2]` (sensor actor, other actor). The call takes six tensors where
it previously took seven. No new C API function is introduced; the reshaped signature is a
direct replacement, and the break is acceptable because the C ABI is pre-release
(`ovphysx/AGENTS.md`).

## Acceptance Criteria

- AC-1: `ovphysx_read_raw_contact_data` returns force, point, normal, separation, the
  per-sensor `sensor_layout_tensor` (`[S, 2]`: count, start index) and the per-contact
  `actor_ids_tensor` (`[C, 2]`: sensor actor, other actor) in one call, aligned 1:1 per
  contact — six tensors, with `sensor_layout_tensor` immediately before `actor_ids_tensor`.
- AC-2: Both columns of `actor_ids_tensor` use the same token namespace
  — the runtime's opaque actor-identity handle, identical across build configurations and
  across the CPU and GPU pipelines — and are therefore directly comparable with each other;
  both are resolvable via `ovphysx_contact_binding_get_other_actor_paths_from_ids`. See
  REQ-TENSOR-CONTACT-001 AC-2 for the runtime-side definition of the encoding.
- AC-3: `ovphysx_read_raw_contact_data` rejects wrong dtype/shape for every tensor argument
  (`OVPHYSX_API_INVALID_ARGUMENT`) and rejects device mismatches (`OVPHYSX_API_DEVICE_MISMATCH`),
  including the two paired tensors. It also requires `max_contact_data_count > 0` at
  binding creation.
- AC-4: `ovphysx_contact_binding_get_other_actor_paths_from_ids` resolves tokens from either
  the sensor-id or other-actor-id buffer, since both use the same namespace.
- AC-5: The Python `ContactBinding.read_raw_contact_data(...)` method exposes all six tensors
  (forces, points, normals, separations, sensor_layout, actor_ids) following the same
  ctypes/DLPack pattern as the existing API.

## Test References

- TEST-CAPI-CONTACT-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_read_raw_contact_data` extended signature)
- ovphysx/src/ovphysx/ovphysxContactBinding.cpp
- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/api.py (`ContactBinding.read_raw_contact_data`)
- ovphysx/python/ovphysx/api.pyi
- ovphysx/tests/c_unittests/test_contact_binding.cpp
- ovphysx/tests/python_tests/cpu_tests/test_tensor_bindings_api.py

## Dependencies

- REQ-TENSOR-CONTACT-001 (runtime `getRawContactData` extended with `sensorActorIdsTensor`)
