<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEBUG-DATA-002
maps_to: REQ-UX-DEBUG-DATA-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Scope logging toggles to PhysX. Verifies REQ-UX-DEBUG-DATA-001 AC-4.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- PhysX and another engine expose distinct sentinel logging channels.

## When

- Toggle Robotics and Scene Multi-GPU logging on/off in debug.

## Then

- Only the intended PhysX channels change; the other engine channels and unrelated logging remain exact. (REQ-UX-DEBUG-DATA-001 AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEBUG-DATA-001](../../requirements/features/REQ-UX-DEBUG-DATA-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

