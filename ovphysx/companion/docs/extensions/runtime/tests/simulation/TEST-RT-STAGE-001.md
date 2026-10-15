<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STAGE-001
maps_to: REQ-RT-STAGE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Open and close an unplayed stage without allocating physics. Verifies REQ-RT-STAGE-001 AC-1, AC-2, AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- A live host stage with one physics scene and one rigid sphere; backend call/allocation counters initially zero.

## When

- Open it, repeat its attach hook, send stopped updates and ray input, then close it twice.

## Then

- Open and stopped activity allocate no instance, physics objects, snapshot/cache, input consumer or output ordinal. (REQ-RT-STAGE-001 AC-1).
- Duplicate attach leaves the same host-stage generation and identity. (REQ-RT-STAGE-001 AC-2).
- Close clears the reference; both closes make zero backend detach/create calls. (REQ-RT-STAGE-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-STAGE-001](../../requirements/simulation/REQ-RT-STAGE-001.md).
