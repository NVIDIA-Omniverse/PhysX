<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEBUG-CONTROL-002
maps_to: REQ-UX-DEBUG-CONTROL-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reject unsupported live overrides without lying about effective state. Verifies REQ-UX-DEBUG-CONTROL-001 AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A running provider with explicit configuration boundaries and known CPU/GPU/solver/synchronous-CUDA effective values.

## When

- Request every debug override, parameterizing accepted deferred changes and rejected unsupported changes.

## Then

- The UI states the required restart/reload/safe boundary and retains prior effective values on rejection; only acknowledged application changes effective status. (REQ-UX-DEBUG-CONTROL-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEBUG-CONTROL-001](../../requirements/features/REQ-UX-DEBUG-CONTROL-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

