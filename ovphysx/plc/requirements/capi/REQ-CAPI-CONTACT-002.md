<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CONTACT-002
title: Filtered Contact and Friction Reads Report Overflow
status: implemented
owner: ovphysx
---

## Description

`ovphysx_read_normal_contact_data` and `ovphysx_read_friction_contact_data` write flat buffers
indexed by per-`(sensor, filter)` count and start-index tensors. The call must
make buffer overflow explicit: it reports the total entries produced for the
step separately from the valid prefix written into the fixed-capacity output
tensors, so callers can distinguish truncation from pairs that genuinely
reported no contacts and can size a replacement binding for a complete read.

`ovphysx_contact_binding_get_sensor_paths` and
`ovphysx_contact_binding_get_filter_paths` use the same public fill contract:
the count out-parameter is total demand, a short caller buffer still receives
a valid prefix, and truncation returns `OVPHYSX_API_BUFFER_TOO_SMALL`.

The extra out-count argument is a pre-release C ABI break, matching
`ovphysx_read_raw_contact_data`.

## Acceptance Criteria

- AC-1: `ovphysx_read_normal_contact_data` sets `out_required_contact_count` to the
  total contacts produced for the step before truncation. If that total exceeds
  the binding's `max_contact_data_count`, it returns
  `OVPHYSX_API_BUFFER_TOO_SMALL` while preserving a valid prefix: per-pair
  counts describe only contacts written and every `[start, start + count)`
  range remains within capacity.
- AC-2: `ovphysx_read_friction_contact_data` sets `out_required_friction_count` to the
  total friction anchors produced for the step before truncation, with the same
  `OVPHYSX_API_BUFFER_TOO_SMALL` and in-range prefix contract as AC-1.
- AC-3: Python `ContactBinding.read_normal_contact_data` and `read_friction_contact_data`
  return the same totals without discarding the valid prefix, treating
  `BUFFER_TOO_SMALL` as a successful truncated read rather than an exception.
- AC-4: `ovphysx_contact_binding_get_sensor_paths` and
  `ovphysx_contact_binding_get_filter_paths` set `out_count` to the complete
  path demand (`sensor_count` and `sensor_count * filter_count`). If demand
  exceeds `max_paths`, they write a valid prefix of `max_paths` strings and
  return `OVPHYSX_API_BUFFER_TOO_SMALL`. Callers index
  `min(out_count, max_paths)`. Python `sensor_paths` / `filter_paths` size
  the native buffer from the spec and treat `BUFFER_TOO_SMALL` as a truncated
  fill, never indexing past that buffer.

## Test References

- TEST-CAPI-CONTACT-002

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_read_normal_contact_data`,
  `ovphysx_read_friction_contact_data`, `ovphysx_contact_binding_get_sensor_paths`,
  `ovphysx_contact_binding_get_filter_paths`)
- ovphysx/src/ovphysx/ovphysxContactBinding.cpp
- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/api.py
- ovphysx/python/ovphysx/api.pyi
- ovphysx/tests/c_unittests/test_contact_binding.cpp
- ovphysx/tests/python_tests/cpu_tests/test_tensor_bindings_api.py

## Dependencies

- REQ-TENSOR-CONTACT-002
- REQ-CAPI-CONTACT-001 (raw-path overflow contract that this filtered path matches)
