<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-CACHE-002
maps_to: REQ-RT-CACHE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Cache refresh uses current authored scale without rewriting the Stop snapshot. Verifies REQ-RT-CACHE-001 AC-2, AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running cached-output session with initial scale (2,3,4), instancer baseline and copied initial snapshot; all borrowed read views are released after capture.

## When

- Make a supported authored scale/pose change, refresh/recreate cache, publish, then close cache.

## Then

- Next output uses the new scale/instancer baseline, with no persistent borrowed view; a cache from another instance or expired attachment is rejected. (REQ-RT-CACHE-001 AC-2).
- The separate Stop snapshot remains byte-for-byte equal to its original copied values and presence metadata after refresh and close. (REQ-RT-CACHE-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-CACHE-001](../../requirements/simulation/REQ-RT-CACHE-001.md).
