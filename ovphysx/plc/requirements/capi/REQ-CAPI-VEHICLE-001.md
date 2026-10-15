<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-VEHICLE-001
title: Vehicle data through the public session API
status: implemented
owner: ovphysx
---

## Description

C and Python expose vehicle controls, transmission state and wheel telemetry through the existing
query/read/write sessions. The public surface forwards the selected rows and typed columns without
adding vehicle-specific functions or requested-control readback caches.

## Acceptance Criteria

- AC-1: **Vehicle selection.** `OVPHYSX_QUERY_TARGET_VEHICLE` and `SimObjectType.VEHICLE` have value 13
  and select vehicle prims. `RIGID_BODY` selects chassis state on that same prim; `VEHICLE_WHEEL`
  independently selects wheel attachments. Filtered and empty selections work before any read.
  Vehicle discovery lists the readable union supported by the selected drive types. A mixed read
  emits only applicable rows and marks unsupported configurations in its optional summary.
- AC-2: **Typed telemetry.** The public names and types match the
  [readable vehicle and wheel tables](../../../docs/read_write/readable.md#whole-vehicles).
  Every group is fixed, host-resident and keyed by its own prim list. Scalars use one DLTensor lane,
  vectors use three or four; Python exposes matching NumPy-compatible Warp shapes and dtypes.
  Ground identities are uint64 prim handles resolvable in the attached Stage's dictionary, with
  zero for absent/unrepresentable identities. Invalid contact samples have zero ground plane,
  hit position and identities.
- AC-3: **Access classification.** `ovphysx_writability` classifies whole-vehicle controls
  (accelerator/brake/steer and the write-only requested gear/tank thrust/reset commands) as
  `CONDITIONAL`, existing raw-wheel controls as `WRITE_ONLY`, and transmission/wheel telemetry
  as `READ_ONLY`. Read discovery excludes write-only names;
  the public write path rejects read-only and drive-inapplicable requests.
- AC-4: **Typed controls and reset.** C/Python sessions accept the control dtypes and ranges in the
  [writable vehicle tables](../../../docs/read_write/writable.md#whole-vehicles), including a
  filtered first-operation write. Controls persist across steps; invalid values fail without
  mutating the addressed group. `resetToRestState` is uint8: zero makes no mutation, one executes
  once on commit, other values fail. Every addressed chassis must have simulation enabled,
  including zero rows. Reset preserves pose and controls, clears chassis motion, and is
  neither replayed on the next step nor applied on release without commit. Chassis writes issued
  after reset remain effective. Existing task, group and session ownership rules apply.

## Test References

- [TEST-CAPI-VEHICLE-001](../../tests/capi/TEST-CAPI-VEHICLE-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (selector: AC-1 through AC-4)
- ovphysx/include/ovphysx/ovphysx_attributes.h (attribute constants: AC-1 through AC-4)
- ovphysx/python/ovphysx/types.py (`SimObjectType.VEHICLE`: AC-1)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (public type forwarding and writability: AC-1, AC-3)
- ovphysx/tests/c_unittests/test_vehicle_read_write.cpp (C sessions: AC-1, AC-2)
- ovphysx/tests/python_tests/cpu_tests/test_vehicle_read_write.py (Python session behavior: all ACs)
- ovphysx/tests/c_unittests/test_writability.cpp and ovphysx/tests/python_tests/test_writability.py (AC-3)
- ovphysx/tests/c_samples/vehicle_read_write_cpp/main.cpp (complete C API workflow: AC-1, AC-2, AC-4)
- ovphysx/tests/python_samples/vehicle_read_write.py (complete Python workflow: AC-1, AC-2, AC-4)
- ovphysx/tests/data/vehicle_read_write.usda (Basic, Standard, tank and raw-wheel fixture)

## Dependencies

- REQ-CAPI-SESSION-001 (task completion, summaries and ownership)
- REQ-CAPI-SELECT-001 (prim selection)
- REQ-CAPI-WRITE-001 (group commit and release)
- REQ-PYTHON-READ-001 (Warp columns)
