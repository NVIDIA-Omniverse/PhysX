<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-DETACH-001
maps_to: REQ-RT-DETACH-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Close waits for output and cleans up before releasing host ownership. Verifies REQ-RT-DETACH-001 AC-1, AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running one-scene session with live output views/cache, active interaction and a deliberately held output completion event.

## When

- Request stage close; observe blocked cleanup, signal completion, and allow teardown to finish.

## Then

- Recorded order is finish step/output, interaction cleanup, close/release views/cache/callbacks, backend detach, instance destroy. (REQ-RT-DETACH-001 AC-1).
- The host stage remains valid until detach completes; no borrowed actor or host framework is destroyed and the adapter reference is released last. (REQ-RT-DETACH-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-DETACH-001](../../requirements/simulation/REQ-RT-DETACH-001.md).
