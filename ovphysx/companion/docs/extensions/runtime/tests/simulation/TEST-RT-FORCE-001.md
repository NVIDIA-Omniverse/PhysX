<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FORCE-001
maps_to: REQ-RT-FORCE-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Push applies a point impulse with the substep duration. Verifies REQ-RT-FORCE-001 AC-1, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Force-calculation fixture with m=2, u=(1,0,0), A=3, h=0.01, COM=(0,0,0), hit=(0,1,0), no gravity/contact/damping, and a native impulse sink spy.

## When

- Apply one due push substep and record linear impulse and its moment.

## Then

- Impulse is (0.06,0,0) and moment is (0,0,-0.06) within 1e-6; it is applied at the hit point, not the COM. (REQ-RT-FORCE-001 AC-1).
- One due substep gives one application between step completions, using matching borrowed actor types; a zero-due frame gives none. (REQ-RT-FORCE-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-FORCE-001](../../requirements/simulation/REQ-RT-FORCE-001.md).
