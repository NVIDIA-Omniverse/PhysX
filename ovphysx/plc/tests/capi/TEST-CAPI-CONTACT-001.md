<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONTACT-001
maps_to: REQ-CAPI-CONTACT-001
type: integration
---

## Scenario

The reshaped `ovphysx_read_raw_contact_data` (six tensors, with the paired `[S, 2]` and
`[C, 2]` layouts) wires through
correctly end to end — from the C ABI down to the runtime, and up through the Python
`ContactBinding.read_raw_contact_data` method.

## Given

- `boxes_falling_on_groundplane.usda` attached, a contact binding over `/World/Cube*` with
  `max_contact_data_count = 64`, settled by 60 simulation steps.
- The Python `TestContactBinding._make_cube_pair_contact_binding` fixture (Cube1 sensor,
  Cube2 filter, settled by one step after positioning Cube2 to overlap Cube1).

## When

- (C ABI) `ovphysx_read_raw_contact_data` is called with all six output tensors (forces,
  points, normals, separations, sensor_layout, actor_ids),
  then `ovphysx_contact_binding_get_other_actor_paths_from_ids` is called on the resulting
  sensor-id tensor and on the other-actor-id tensor.
- (C ABI) `ovphysx_read_raw_contact_data` is called with a sensor-layout tensor of the wrong
  column count, `[S, 4]` (regression case: `RawContactDataRejectsWrongShape`).
- (Python) `ContactBinding.read_raw_contact_data(...)` is called with six tensors, then
  `ContactBinding.get_other_actor_paths_from_ids(...)` is called once on a slice of
  `sensor_actor_ids` and once on a slice of `other_actor_ids`.

## Then

- `ovphysx_read_raw_contact_data` returns `OVPHYSX_API_SUCCESS` and both id columns are
  non-zero for every contact inside a sensor's `[start, start + count)` range, read from that
  sensor's `sensor_layout` row (REQ AC-1, AC-2).
- `ovphysx_contact_binding_get_other_actor_paths_from_ids`, called once on each id tensor,
  writes one NUL-terminated entry per id and a non-empty path for every non-zero id; the
  untouched tail of the buffer holds zero ids, which resolve to empty. Both buffers going
  through the same resolver is the C-ABI evidence for the shared namespace (REQ AC-2, AC-4).
- The wrong-shape call returns `OVPHYSX_API_INVALID_ARGUMENT` (REQ AC-3).
- The Python `read_raw_contact_data` call reports at least one contact between Cube1 and Cube2;
  resolving Cube1's sensor-actor ids via `get_other_actor_paths_from_ids` returns paths
  containing `"Cube1"`, and resolving the other-actor ids returns paths containing `"Cube2"`
  (REQ AC-1, AC-2, AC-4, AC-5).
