<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-SNAPSHOT-004
maps_to: REQ-RT-SNAPSHOT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Object identity changes do not corrupt the play baseline. Verifies REQ-RT-SNAPSHOT-001 AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running session with a captured object A, supported addition of object B with known initial state, and an existing snapshot generation.

## When

- Add B before its first output, remove A, pause/resume, and attempt a structural edit for which no safe identity mapping exists, then Stop and inspect the removed prim.

## Then

- B is captured once before its first output, A is marked absent and never resurrected, pause/resume preserves the baseline, and unmappable structure requires Stop/new Play instead of remapping stale entries. (REQ-RT-SNAPSHOT-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-SNAPSHOT-001](../../requirements/simulation/REQ-RT-SNAPSHOT-001.md).
