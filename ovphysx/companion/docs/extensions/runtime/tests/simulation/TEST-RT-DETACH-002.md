<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-DETACH-002
maps_to: REQ-RT-DETACH-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Old generations cannot mutate a newly played stage. Verifies REQ-RT-DETACH-001 AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- A Running stage with saved hit/view/queued completion tokens and a known play generation.

## When

- Stop/Play the same host stage, deliver old tokens, then close/open a replacement single-scene stage and deliver them again.

## Then

- Old commands/completions are discarded before backend/stage access; old hits/views are rejected. Stop changes play generation; close/reopen also changes host generation. New stage values and step count remain unchanged by old work. (REQ-RT-DETACH-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

Mapped requirement: [REQ-RT-DETACH-001](../../requirements/simulation/REQ-RT-DETACH-001.md).
