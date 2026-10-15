<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEMOS-001
maps_to: REQ-UX-DEMOS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Browse and launch through normal unsaved-stage and Play behavior. Verifies REQ-UX-DEMOS-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A dirty stage, provider demo metadata and supported single-scene/unsupported-capability demo entries; no initial backend.

## When

- Browse, select unavailable entry, launch a valid demo parameterized over save/discard/cancel at the dirty-stage prompt, then Play the launched stage.

## Then

- Browse loads metadata only and explains missing capability without enabling test extensions or creating physics. (REQ-UX-DEMOS-001 AC-1).
- Cancel preserves the dirty stage; save/discard follows Kit semantics; successful launch uses normal stage switching and one-scene Play with no hidden instance. (REQ-UX-DEMOS-001 AC-2).

- **Inventory.** Each legacy demo and retained internal/example scene shall have a recorded runnable, fixture-only, separate-backend or unsupported disposition with its service/backend/assets requirements; no unclassified scene may silently disappear from migration coverage. (REQ AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEMOS-001](../../requirements/features/REQ-UX-DEMOS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

