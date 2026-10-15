<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STOP-003
maps_to: REQ-RT-STOP-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Stop restores admitted non-rigid restart inputs after destroying physics. Verifies REQ-RT-STOP-001 AC-2, AC-3, AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize an admitted articulation, deformable and particle fixture, each with nontrivial initial root/joint or points/velocities and visible output mappings.

## When

- Simulate, Stop, compare copied host state, then Play again.

## Then

- Each admitted type restores its exact initial restart inputs through host writes after backend teardown; articulation links are not individually restored through read-only live pose setters. (REQ-RT-STOP-001 AC-2).
- Visible geometry/descendants and sim:* presence match the initial presentation before Stop succeeds. (REQ-RT-STOP-001 AC-3).
- The next Play creates a fresh generation from restored inputs. (REQ-RT-STOP-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-STOP-001](../../requirements/simulation/REQ-RT-STOP-001.md).
