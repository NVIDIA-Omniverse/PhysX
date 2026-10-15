<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PLAY-002
maps_to: REQ-RT-PLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Play rejects missing stage identity without fallback loading. Verifies REQ-RT-PLAY-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize Detached with no identity and Stopped whose remembered stage can no longer be resolved.

## When

- Call start/onResume in each fixture.

## Then

- A stage-context error is observable; no filename load, alternative stage, backend instance or dynamics step is created. (REQ-RT-PLAY-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

Mapped requirement: [REQ-RT-PLAY-001](../../requirements/simulation/REQ-RT-PLAY-001.md).
