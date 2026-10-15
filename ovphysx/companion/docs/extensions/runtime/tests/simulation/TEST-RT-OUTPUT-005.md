<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-OUTPUT-005
maps_to: REQ-RT-OUTPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Presentation waits for the output ordinal and device completion. Verifies REQ-RT-OUTPUT-001 AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Instrumented host ordinal allocator and fences; delay one output write/device event after a completed backend step on a supported CPU/CUDA path.

## When

- Observe backend post-step notification, attempt presentation consumption, then complete writes/device event and advance the output floor.

## Then

- A fresh output ordinal is reserved and excluded from control drains; no presentation-ready event occurs at backend post-step or seal alone. Only completed writes/floor/device fences release consumers. (REQ-RT-OUTPUT-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-OUTPUT-001](../../requirements/simulation/REQ-RT-OUTPUT-001.md).
