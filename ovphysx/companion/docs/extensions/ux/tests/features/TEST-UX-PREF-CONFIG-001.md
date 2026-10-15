<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PREF-CONFIG-001
maps_to: REQ-UX-PREF-CONFIG-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Select a CUDA device without replaying legacy context or multi-GPU policy. Verifies REQ-UX-PREF-CONFIG-001 AC-1, AC-5.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Stored use-active-context=true and multi-GPU=all, a supported device list and a session with known bootstrap ordinal.

## When

- Open preferences, request empty/-1/a valid single ordinal and invalid/out-of-range input; change ordinal after GPU bootstrap, then Stop/Play.

## Then

- Only explicit device requests map to active_cuda_gpus; empty stays no override, -1 stays automatic, invalid values are rejected. Legacy booleans/multi-GPU values cause no config calls; multi-GPU control is absent. Post-bootstrap ordinal change remains Requires Restart even after Stop. (REQ-UX-PREF-CONFIG-001 AC-1).
- UI sends requests through the runtime owner, makes no instance of its own and does not report a stored request as effective. (REQ-UX-PREF-CONFIG-001 AC-5).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PREF-CONFIG-001](../../requirements/features/REQ-UX-PREF-CONFIG-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

