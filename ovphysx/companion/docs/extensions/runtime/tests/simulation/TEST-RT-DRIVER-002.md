<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-DRIVER-002
maps_to: REQ-RT-DRIVER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reentrant pause and presenter gating cannot create a second producer. Verifies REQ-RT-DRIVER-001 AC-3, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running single-scene session, h=1/60, pre-step callback that requests Pause, and an instrumented operation log.

## When

- Request Pause from the callback, complete the current substep/publication, then send gated disabled updates for one second.
- Reopen the gate without Play, then explicitly resume and deliver one h update.

## Then

- Pause is processed only at the safe boundary; no overlapping backend call or callback into a destroyed session occurs. (REQ-RT-DRIVER-001 AC-3).
- Disabled time adds no debt; gate reopening alone does not resume the Paused session. The explicit resumed update advances once. (REQ-RT-DRIVER-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-DRIVER-001](../../requirements/simulation/REQ-RT-DRIVER-001.md).
