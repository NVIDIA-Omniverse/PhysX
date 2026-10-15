<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STEP-003
maps_to: REQ-RT-STEP-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Invalid or non-simulating updates do not maintain an editor backend. Verifies REQ-RT-STEP-001 AC-3, AC-6.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- An instrumented adapter in Stopped, Paused and Running variants, with known remainder h/2; a held interaction only in the Running variant.

## When

- Send disabled updates, zero dt, negative dt, NaN/infinite dt and nonfinite currentTime; also send positive enabled updates while Stopped/Paused.
- Inspect calls and then deliver a valid h/2 update while Running.

## Then

- All disallowed updates have zero step/output calls and unchanged remainder. Disabled updates also have zero input-drain/cooking/query-service calls, apart from permitted interaction cancellation. (REQ-RT-STEP-001 AC-3).
- Only admitted elapsed time contributes; the final valid h/2 update makes the original remainder due once, with no debt from ignored time. (REQ-RT-STEP-001 AC-6).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_boundary_conditions.py](../../../../../../tests/python_tests/cpu_tests/test_step_boundary_conditions.py) — NaN dt rejection and synchronous/asynchronous step boundary cases (current API reference).

Mapped requirement: [REQ-RT-STEP-001](../../requirements/simulation/REQ-RT-STEP-001.md).
