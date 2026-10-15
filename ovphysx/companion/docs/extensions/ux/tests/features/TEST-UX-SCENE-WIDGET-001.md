<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-SCENE-WIDGET-001
maps_to: REQ-UX-SCENE-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Opening a scene widget does not repair or author GPU dependencies. Verifies REQ-UX-SCENE-WIDGET-001 AC-1, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Authored GPU dynamics enabled with deliberately different broadphase/PCM values, recorded layer text and inactive-engine variant.

## When

- Open/rebuild the scene widget twice and switch selection/engine.

## Then

- Layer bytes/opinions remain unchanged and no backend is created. (REQ-UX-SCENE-WIDGET-001 AC-1).
- The UI distinguishes authored state from support/effective/pending state rather than claiming a live device change. (REQ-UX-SCENE-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-SCENE-WIDGET-001](../../requirements/features/REQ-UX-SCENE-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

