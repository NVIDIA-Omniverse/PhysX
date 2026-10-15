<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEBUG-CONTROL-001
maps_to: REQ-UX-DEBUG-CONTROL-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Run pause and stop through the generic hooks. Verifies REQ-UX-DEBUG-CONTROL-001 AC-1, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A remembered stopped stage with one moving rigid body, known initial pose/velocities and generic-hook/backend spies.

## When

- Open debug, Run, issue a normal timeline update, Pause, resume, Stop; attempt Step/force load/reset/release while stopped and paused.

## Then

- Play loads once, Pause retains the session, resume does not reload, Stop restores the known pose/velocities and destroys the session. Unsupported controls are disabled with reasons and cause no calls. (REQ-UX-DEBUG-CONTROL-001 AC-1).
- Opening the window and inspecting an unavailable mass/diagnostic view while stopped creates no instance or legacy test-interface call. (REQ-UX-DEBUG-CONTROL-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEBUG-CONTROL-001](../../requirements/features/REQ-UX-DEBUG-CONTROL-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

