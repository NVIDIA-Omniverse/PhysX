<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PICK-003
maps_to: REQ-RT-PICK-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Unavailable interaction modes fail explicitly without fallback. Verifies REQ-RT-PICK-001 AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize force/joint/deformable modes on a declared-supported execution path and a path lacking one required operation (for example DirectGPU damping/impulse parity); include a Stopped stage.

## When

- Query availability and attempt a press for each row.

## Then

- Unsupported complete modes report unavailable before drag state is established; no silent force/joint/rigid substitution or stopped-stage loading occurs. A claimed-supported mode must execute its full mapped FORCE tests, not skip the missing component. (REQ-RT-PICK-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-PICK-001](../../requirements/simulation/REQ-RT-PICK-001.md).
