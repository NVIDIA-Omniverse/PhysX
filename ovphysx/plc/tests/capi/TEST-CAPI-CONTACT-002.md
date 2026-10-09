<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONTACT-002
maps_to: REQ-CAPI-CONTACT-002
type: integration
---

## Scenario

Filtered contact and friction reads report complete demand on overflow while
leaving a usable in-range prefix in the caller's buffers. Sensor and filter
path getters use the same demand + `BUFFER_TOO_SMALL` fill contract.

## Given

- Two C-ABI filtered contact bindings over settled
  `boxes_falling_on_groundplane.usda`: a reference binding with capacity 64 and
  an undersized binding with capacity 1 (`/World/Cube*` sensors,
  `/World/BigBase` filter).
- A C-ABI contact binding with two sensors and two filters
  (`GetContactBindingSpecExactCounts` fixture: Cube1/Cube2 sensors, Cube2/Cube3
  filters).
- The Python `TestContactBinding._make_cube_pair_contact_binding` fixture with
  `max_contact_data_count=1`.

## When

- The reference and undersized C-ABI bindings both read contact data and then
  friction data against the same settled step.
- Python `read_contact_data` and `read_friction_data` are called on the
  capacity-1 cube-pair fixture.
- C-ABI `ovphysx_contact_binding_get_sensor_paths` and
  `get_filter_paths` are called with `max_paths` smaller than demand.

## Then

- The reference C contact read succeeds and reports a required count equal to
  its summed per-pair counts. The undersized C contact read returns
  `OVPHYSX_API_BUFFER_TOO_SMALL`, reports the same complete required count,
  writes exactly one contact, and keeps every pair range in bounds (REQ AC-1).
- The friction reads do the same for friction anchors (REQ AC-2).
- Python returns a required count greater than capacity while exposing a valid
  prefix whose counts sum to capacity and whose `[start, start + count)` ranges
  stay in bounds (REQ AC-3).
- Sensor-path get with `max_paths=1` returns `OVPHYSX_API_BUFFER_TOO_SMALL`,
  `out_count=2`, and the first sensor path. Filter-path get with `max_paths=1`
  returns `BUFFER_TOO_SMALL`, `out_count=4`, and the first filter path
  (REQ AC-4).

## Implementation

- `ovphysx/tests/c_unittests/test_contact_binding.cpp`
  (`FilteredReadsOverflowReturnsRequiredCountAndValidPrefix`,
  `PathGettersReportDemandAndBufferTooSmall`)
- `ovphysx/tests/python_tests/cpu_tests/test_tensor_bindings_api.py`
  (`test_read_normal_contact_data_overflow_returns_required_count_and_prefix`,
  `test_read_friction_contact_data_overflow_returns_required_count_and_prefix`)
