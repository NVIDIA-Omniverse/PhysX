<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-SCENE-WIDGET-002
maps_to: REQ-UX-SCENE-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Group deliberate GPU dependencies in one undo action. Verifies REQ-UX-SCENE-WIDGET-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A writable scene with GPU dynamics disabled and known broadphase/PCM opinions; active session in the deferred variant.

## When

- Deliberately enable GPU dynamics in the widget, inspect dependent edits, undo/redo.

## Then

- Required GPU broadphase/PCM changes are explained and committed with the initiating edit in one undo group; one undo restores all original opinions. (REQ-UX-SCENE-WIDGET-001 AC-2).
- The running instance remains unchanged until the supported boundary and the UI reports pending/restart state. (REQ-UX-SCENE-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-SCENE-WIDGET-001](../../requirements/features/REQ-UX-SCENE-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

