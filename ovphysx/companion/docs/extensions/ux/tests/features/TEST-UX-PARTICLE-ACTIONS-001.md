<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PARTICLE-ACTIONS-001
maps_to: REQ-UX-PARTICLE-ACTIONS-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve particle authoring and distinguish postprocess capabilities, including failure, source replacement and exact undo/teardown.

## Given

A watertight box and open triangle mesh, existing particle system/set and PBD material, unrelated points and both authored/absent offset values. Separate fixtures declare diffuse, smoothing, anisotropy and isosurface provider availability.

## When

Add/remove each particle schema and command; sample volume and surface modes and change topology/options while sampling is held. Disable sampling. Execute each declared postprocess against a completed frame with a known reference result.

## Then

- Schema targets, fluid/solid and mass/density values match the selected command; undo restores authored absence and preserves unrelated points. (REQ AC-1).
- Samples satisfy the fixture volume/surface and separation bounds in stage units; stale jobs publish nothing. Disabling sampling cancels future jobs without deleting preexisting points. (REQ AC-2).
- Each postprocess matches its separate reference fixture and generation. Missing providers are recorded as incomplete positive cases; Poisson sampling and old authored points cannot supply their oracle. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-PARTICLE-ACTIONS-001](../../requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md).
