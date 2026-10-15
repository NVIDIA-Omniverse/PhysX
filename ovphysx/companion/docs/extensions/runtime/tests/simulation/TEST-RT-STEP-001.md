<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STEP-001
maps_to: REQ-RT-STEP-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Resolve and freeze the single-scene timestep. Verifies REQ-RT-STEP-001 AC-1.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize one recognized scene: PhysX=120/Newton=90; Newton-only=90; neither authored=60; invalid/zero effective rate. Minimum frame rate initially 30.

## When

- Play; for valid rows, author new rate/minimum-frame-rate values during Play, update, then Stop/Play again.

## Then

- Effective rates are 120,90,60 respectively; invalid/zero fails before Running. Changes do not affect the current epoch and are read by the next Play. (REQ-RT-STEP-001 AC-1).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-STEP-001](../../requirements/simulation/REQ-RT-STEP-001.md).
