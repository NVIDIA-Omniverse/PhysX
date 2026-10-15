<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-CAMERA-001
maps_to: REQ-COMP-CAMERA-001
type: integration
---

## Coverage status

Eleven automated cases pass for [REQ-COMP-CAMERA-001](../../requirements/controllers/REQ-COMP-CAMERA-001.md) AC-1 through AC-4, including native completed reads and an isolated native-worker/customer-USD sample. GPU transport remains dependent on explicit host copies and hardware qualification.

## Scenario

Follow a moving target, then delete and recreate it while disabling the camera.

## Given

- A known body trajectory, saved camera opinions, a selected follow mode and an explicit camera layer.

## When

- Update from completed frames, delete/recreate the target, then disable the camera and close the stage adapter.

## Then

- Camera poses match the fixture trajectory/frame transform using only supported reads (REQ AC-1).
- Only the selected camera layer is authored by Python/host code (REQ AC-2).
- The declared restore mode restores saved opinions and no updates arrive after disable (REQ AC-3).
- A stale target causes the selected documented outcome, never following the recreated identity accidentally (REQ AC-4).

## Executable coverage

- [test_camera.py](../../../tests_native/test_camera.py): `test_follow_completed_core_body_output`, units/velocity/orientation, stale incarnation, missing/invalid input and drone local input/disable.
- [test_camera_usd.py](../../../tests/test_camera_usd.py): explicit layer and parent/unit conversion, restore on missing/recreated target, disable and session close, no update after close, conflict rejection, and `test_native_follow_camera_sample`.

[test_output.py](../../../tests_native/test_output.py) rejects PointInstancer array
groups in the scalar body-output helper with an explicit instance-aware selection
message; selecting scalar rigid-body paths still returns completed poses (REQ AC-1).
