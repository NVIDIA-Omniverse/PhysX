<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-JOINT-ARRAYS-001
maps_to: REQ-CAPI-JOINT-ARRAYS-001
type: integration
---

## Scenario

An application regroups a joint group, one row per DOF, into one array per joint: it reads the
views, writes through them, and publishes them to its attached stage.

## Given

- The humanoid fixture attached through ovstage on a CPU scene and stepped once. Its joints have
  one, two and three axes, so the joint group's prim list repeats paths.

## When

- `ovphysx_utils.h` is compiled as C11 on its own and after `ovphysx.h`, each function named with
  its signature.
- A `jointPosition` read group is regrouped (C and Python), its views compared with the group's
  rows, and, in C, its write payload written to the stage under `sim:jointPosition`, sealed and
  read back joint by joint.
- A `jointPosition` write group is regrouped, each joint's view filled with its own value, the
  group committed and the column read back.
- A synthetic group carrying two tensors is regrouped (C and Python), and a deleted read group; in C
  also NULL arguments, with the getters and the destroy called on NULL; in Python a group whose row
  count differs from its prim count.

## Then

- Both header translation units compile (REQ AC-1).
- There is one view per run of equal paths, in row order, and the path list names those joints
  once each; each view starts at its joint's first row of the group's own column, holds its axis
  count of rows, and has the column's element type, lanes and device (REQ AC-2, REQ AC-3).
- The write payload is an array write of those views, and the stage reads back each joint's own
  rows (REQ AC-3).
- After the commit, every row of a joint reads that joint's value (REQ AC-2).
- The malformed groups are refused: in C with `OVPHYSX_API_INVALID_ARGUMENT`, no object and a
  message from `ovphysx_get_last_error`, the getters returning nothing for NULL; in Python with
  `ValueError` (REQ AC-1, REQ AC-4).

## Test locations

- `ovphysx/tests/c_unittests/test_joint_arrays.cpp`: `JointArraysTest.*`.
- `ovphysx/tests/c_unittests/test_utils_header_include_only.c`,
  `ovphysx/tests/c_unittests/test_utils_after_primary.c` (the `c_api_compatibility_test` target).
- `ovphysx/tests/python_tests/test_step_and_write_to_ovstage.py`:
  `test_split_joint_group_views_a_joint_read_group_per_joint`,
  `test_split_joint_group_views_fill_a_joint_write_group`,
  `test_split_joint_group_refuses_a_group_not_shaped_as_a_joint_group`.
- Fixture: `ovphysx/tests/benchmarks/data/humanoid.usda`.

## Coverage limits

The fixtures are CPU scenes. A DirectGPU group gives device views the same way, through the same
offsets, but no case here regroups one.
