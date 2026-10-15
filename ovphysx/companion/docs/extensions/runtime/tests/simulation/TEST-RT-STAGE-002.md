<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STAGE-002
maps_to: REQ-RT-STAGE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Stage replacement drains the old session before remembering the new stage. Verifies REQ-RT-STAGE-001 AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running stage A with held output completion; replacement stage B also contains exactly one scene. Stages are used sequentially.

## When

- Repeat A attach, then request replacement by B; release A completion and allow detach/attach hooks to finish.

## Then

- Duplicate A attach does not reload. A cleanup finishes before B becomes remembered; B is Stopped with no instance and requires explicit Play. (REQ-RT-STAGE-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-STAGE-001](../../requirements/simulation/REQ-RT-STAGE-001.md).
