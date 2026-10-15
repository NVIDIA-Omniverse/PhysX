<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MOUSE-001
maps_to: REQ-UX-MOUSE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Keep a drag with its selected engine and completed hit identity. Verifies REQ-UX-MOUSE-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A running single-scene provider, another selectable engine, a perspective viewport and two instancer bodies with distinguishable hit identities.

## When

- Press/hold/release over a known instance while changing the hover target and delaying hit feedback.

## Then

- The emitted ray equals the viewport world-ray oracle; all intents go to the initial owner with no hidden fallback. (REQ-UX-MOUSE-001 AC-1).
- Markers identify the completed hit instance and generation and remain absent before valid feedback; stale feedback is ignored. (REQ-UX-MOUSE-001 AC-2).
- Release ends the interaction once and removes owned markers/transients. (REQ-UX-MOUSE-001 AC-3).

- **Force semantics.** Preserve legacy push/grab coefficient, acceleration, damping, gravity and ignore-invisible selection policy under runtime ADR-0009. Use companion managed force grabbing by default on exact instances; native code owns its force law and restoration. Push retains its separate impulse workflow. D6 drag is a separately labelled explicit mode and is not accepted as proof of force-law parity. (REQ AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MOUSE-001](../../requirements/features/REQ-UX-MOUSE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
