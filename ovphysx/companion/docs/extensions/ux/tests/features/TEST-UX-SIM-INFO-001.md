<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-SIM-INFO-001
maps_to: REQ-UX-SIM-INFO-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Retain simulation information using explicit owner diagnostics, including failure, source replacement and exact undo/teardown.

## Given

Playback owner A with a completed CPU frame, independent preview B with different counters, a recording view, and a provider reporting one unsupported metric. A second fixture uses a verified GPU pipeline.

## When

Open the information view while stopped and during playback; select A then B, inspect destinations, stop A and close the panel. Observe owner creation and step counters.

## Then

- Labels show the selected exact owner, stage and role; CPU/GPU text matches the actual pipeline fixture rather than a stored preference. (REQ AC-1).
- Displayed supported counts equal captured completed records, while the unsupported metric says unavailable. Opening/switching the view creates and steps no owner. (REQ AC-2).
- Destination inspection installs no USD/Fabric writer. Stopping A clears its statistics while B and the recording view retain their identities and data. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-SIM-INFO-001](../../requirements/features/REQ-UX-SIM-INFO-001.md).
