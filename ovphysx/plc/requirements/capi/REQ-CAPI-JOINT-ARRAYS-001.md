<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-JOINT-ARRAYS-001
title: Joint Groups Regrouped Into One Array per Joint
status: implemented
owner: ovphysx
---

## Description

An articulation-joint read or write group has one row per DOF, and its prim list names each
row's joint, so a joint's path repeats once per axis (ADR-0059). ovstage stores a joint's values
as one array per joint prim. The joint-arrays functions of `ovphysx_utils.h` (C) and
`ovphysx.utils.split_joint_group` (Python) turn a joint group into that form for a caller that hands
joint values to ovstage or works per joint, without copying anything.

## Acceptance Criteria

- AC-1: **Utility surface.** `ovphysx/ovphysx_utils.h` is a C header beside `ovphysx.h`, which
  does not include it; it compiles as C11 on its own and after `ovphysx.h`.
  `ovphysx_create_joint_arrays_from_read_group` and `ovphysx_create_joint_arrays_from_map_group`
  take the attached stage whose path dictionary holds the group's prim list and a fetched joint
  `ovstage_read_group_t` or `ovstage_map_group_t`, and return an opaque `ovphysx_joint_arrays_t`
  that `ovphysx_destroy_joint_arrays` destroys. `ovphysx_get_joint_arrays_prim_list` and
  `ovphysx_get_joint_arrays_write_data` read it; they and the destroy accept NULL. The Python
  utility takes the attached `ovstage.Stage` and a joint `ReadGroup` or `WriteGroup` and returns
  `(prim_list, tensors)`. Neither needs CUDA or PhysX SDK headers in the consuming application.
- AC-2: **One view per joint, nothing copied.** The result holds one tensor per joint, in the
  group's order: a view of that joint's consecutive rows in the group's own memory, with the
  group's element type, lanes and device, shaped `[its DOF count]`. Writing through a view of a
  write group fills that group's column. The views are valid only while the group is: until a
  read group is released or a write group is committed.
- AC-3: **A list naming each joint once.** The result has a path list naming each joint once, in
  view order, which a caller passes to `ovstage_query_from_path_list` (or
  `Stage.query_from_path_list`). In C the object owns it and destroying the object releases it; in
  Python it is an `ovstage.PathList` the caller releases, empty for an empty group.
  `ovphysx_get_joint_arrays_write_data` is an ovstage write payload of the views as an array
  attribute, carrying a read group's semantic and CUDA sync, so a joint read group reaches the
  stage as one array per joint prim.
- AC-4: **Errors.** A deleted read group, or a group not shaped as a joint group -- more than one
  tensor, an index map, or a row count different from its prim count -- is refused. C also refuses
  a NULL argument, a read group with other than one column, a mask and a tensor that is not
  one-dimensional and contiguous; the Python groups carry no mask, and their arrays are contiguous
  with any lanes as a trailing dimension. A C refusal returns `OVPHYSX_API_INVALID_ARGUMENT`, sets
  no object, and leaves the reason for `ovphysx_get_last_error`; Python raises `ValueError`. A C
  path-dictionary or allocation failure returns `OVPHYSX_API_ERROR` the same way, with the
  dictionary's message.

## Test References

- [TEST-CAPI-JOINT-ARRAYS-001](../../tests/capi/TEST-CAPI-JOINT-ARRAYS-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_utils.h (`ovphysx_joint_arrays_t` and its functions; AC-1
  through AC-4)
- ovphysx/src/ovphysx/ovphysxUtils.cpp (regrouping, the owned path list and the write payload;
  AC-1 through AC-4)
- ovphysx/python/ovphysx/utils/simulation.py (`split_joint_group`; AC-1 through AC-4)

## Dependencies

- [ADR-0059](../../../ovruntime/plc/adr/ADR-0059-joint-groups-per-dof.md) (joint groups have one
  row per DOF)
- [REQ-READ-CORE-001](../../../ovruntime/plc/requirements/output/REQ-READ-CORE-001.md) AC-11 (the
  joint group layout)
