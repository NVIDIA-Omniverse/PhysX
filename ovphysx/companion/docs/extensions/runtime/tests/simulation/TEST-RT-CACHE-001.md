<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-CACHE-001
maps_to: REQ-RT-CACHE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Output cache lifetime follows the play session rather than the open stage. Verifies REQ-RT-CACHE-001 AC-1.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Host stage open with cache/buffer/view/event creation and release counters.

## When

- Play and publish, Pause/resume, Stop, then Play and publish again before stage close.

## Then

- No output resources exist before Play. Pause can retain valid resources; Stop closes them before detach. New Play uses fresh attachment-bound resources and close returns session resource counters to zero. (REQ-RT-CACHE-001 AC-1).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).

Mapped requirement: [REQ-RT-CACHE-001](../../requirements/simulation/REQ-RT-CACHE-001.md).
