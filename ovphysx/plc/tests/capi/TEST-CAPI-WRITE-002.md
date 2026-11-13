<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-WRITE-002
maps_to: REQ-CAPI-WRITE-001
type: integration
---

## Scenario

`ovphysx_commit_group_masked` writes only the rows a mask selects, through the public C
surface and Python's `commit(mask=...)`, and refuses a malformed mask without spending the
group (AC-16).

## Given

- An instance attached to `boxes_falling_on_groundplane.usda` on a CPU scene, stepped once.
- A rigid-body query and a `position` write over it.

## When

- C: every row of every group is filled with 777 and committed under a host uint8 mask
  selecting every other row.
- C: a fetched group is committed with a null mask, then with a float32 mask, then with a
  uint8 mask one entry too long, and finally unmasked.
- Python: every row is filled with 300 and committed with `mask=` a NumPy bool array
  selecting every other row.
- Python: a fetched group is committed with a NumPy bool mask one entry too long, then with
  a mask selecting every row.

## Then

- Each position afterwards is its previous value or the written one, and exactly as many
  rows changed as the masks selected (REQ AC-16).
- The null, float32 and overlong masks are all rejected at admission with
  `OVPHYSX_API_INVALID_ARGUMENT`, and the unmasked commit after them succeeds: the group
  stayed live (REQ AC-16).
- In Python the selected rows read 300 and every other row is unchanged (REQ AC-16).
- In Python the overlong mask raises at `commit`, naming the rejected mask, and the commit
  that follows writes every row: the group was not spent (REQ AC-16).

## Code References

- `ovphysx/tests/c_unittests/test_write_api.cpp`: `WriteApiTest.MaskedCommitWritesOnlySelectedRows`,
  `WriteApiTest.MaskedCommitRejectsMalformedMasks`.
- `ovphysx/tests/python_tests/cpu_tests/test_write_api_cpu.py`:
  `test_cpu_masked_commit_writes_only_selected_rows`,
  `test_cpu_malformed_mask_leaves_group_committable`.

The device paths -- the mask read on the device, the stream-ordered commit and its capture --
are covered in the runtime by
[TEST-INPUT-CORE-002](../../../ovruntime/plc/tests/input/TEST-INPUT-CORE-002.md) and
[TEST-INPUT-DEVICE-006](../../../ovruntime/plc/tests/input/TEST-INPUT-DEVICE-006.md).
