<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-READINESS-001
maps_to: REQ-UX-READINESS-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve capability checks and safe physics variant selection, including failure, source replacement and exact undo/teardown.

## Given

A stage with rigid-only and deformable variants, an unrecognized schema and providers advertising distinct CPU/GPU/authoring capabilities; one provider is absent. Capture existing variant opinions and runtime owner counts.

## When

Open readiness, inspect reasons, explicitly select a compatible variant and undo. Change variants externally, replace the stage and register/unregister a provider while a check is outstanding.

## Then

- Rigid/deformable and unknown cases receive supported/unsupported/unqualified reasons based on declared operation capabilities. Opening the panel initializes no engine. (REQ AC-1).
- Proposals leave authored selections unchanged; only the approved valid choice creates one undo item. No runtime owner or legacy extension activation changes. (REQ AC-2).
- Current-stage schema/provider notices cause one refreshed result per logical update; stale results are discarded. Closing the feature restores callback counts to baseline. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-READINESS-001](../../requirements/features/REQ-UX-READINESS-001.md).
