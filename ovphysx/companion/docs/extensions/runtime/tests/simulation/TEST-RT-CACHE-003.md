<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-CACHE-003
maps_to: REQ-RT-CACHE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Failed publication releases queries without freeing in-use device buffers. Verifies REQ-RT-CACHE-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One completed step with output queries and buffer/event probes; inject a write failure and hold one dependent completion event.

## When

- Run publication failure cleanup, inspect resource ownership, then release the outstanding event and finish cleanup.

## Then

- Queries are released on error, the failed output ordinal is not announced as completed, and in-use buffers remain alive until dependent work finishes. Afterwards all releasable output resources close without a second step. (REQ-RT-CACHE-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-CACHE-001](../../requirements/simulation/REQ-RT-CACHE-001.md).
