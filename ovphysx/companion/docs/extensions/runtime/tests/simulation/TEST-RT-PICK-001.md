<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PICK-001
maps_to: REQ-RT-PICK-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Ray selection resolves eligible objects and copies transient identity. Verifies REQ-RT-PICK-001 AC-1, AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with a hidden nearest shape, visible dynamic targets including a point-instancer body, and static/kinematic/disabled negative-control targets; query buffers are overwritten by a later query.

## When

- Submit finite rays with non-unit directions, then invalid rays; vary eligibility/visibility so each target class is exercised. Perform a second query after selecting the instancer body.

## Then

- Valid directions are normalized, max distance is 1000000 stage units, and the nearest eligible target is selected even when filtering removes the closest raw hit. Invalid rays and non-dynamic targets cause no impulse. (REQ-RT-PICK-001 AC-1).
- The selected body/instance retains copied path/hit identity after the query buffer changes; opaque IDs are never interpreted as pointers or paths. (REQ-RT-PICK-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

Mapped requirement: [REQ-RT-PICK-001](../../requirements/simulation/REQ-RT-PICK-001.md).
