<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-STAGE-POLICY-003
maps_to: REQ-UX-STAGE-POLICY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Translate only validated geometry settings without cross-stage leakage. Verifies REQ-UX-STAGE-POLICY-001 AC-3, AC-5.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Stages A/B: A has explicit custom-geometry requests and legacy approximation flags; B has no overrides. Provider defaults are recorded and a second process-config consumer can report a conflict.

## When

- With validated geometry support, Play A then Stop/switch/Play B; repeat with unvalidated mode or conflict. Compare produced collision representation, not merely getter values.

## Then

- The provider uses named typed custom-geometry keys before attach, never blindly copies/inverts old approximation flags; no live recook or authored geometry replacement occurs. Unvalidated support stays unavailable. (REQ-UX-STAGE-POLICY-001 AC-3).
- B resolves explicit defaults rather than A values; conflict/rejection is visible before attach with no silent process-global overwrite. Geometry evidence demonstrates the advertised behavior when enabled. (REQ-UX-STAGE-POLICY-001 AC-5).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-STAGE-POLICY-001](../../requirements/features/REQ-UX-STAGE-POLICY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

