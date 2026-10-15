<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-CAMERA-002
maps_to: REQ-UX-CAMERA-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Follow completed state and restore temporary camera ownership. Verifies REQ-UX-CAMERA-001 AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- An advertised camera service, live body and saved active camera; pending incomplete body output.

## When

- Enable follow, complete one output, Pause/resume then Stop; parameterize target deletion/disable/detach as teardown triggers.

## Then

- Camera follows only completed output, freezes while paused, and Stop restores initial transient state; teardown restores prior active-camera/input ownership without extra physics steps. (REQ-UX-CAMERA-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-CAMERA-001](../../requirements/features/REQ-UX-CAMERA-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
