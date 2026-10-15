<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-OUTPUT-001
title: Publish exactly the completed simulation output to ovstage
status: proposed
owner: physics-runtime
---

## Description

Publish exactly the completed simulation output to ovstage. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Step/output count.** For n admitted substeps, runtime shall execute exactly n solver steps and n output transactions using step_and_write_to_ovstage(dt=h) or an equivalent composition. It shall not call another step before the stepping helper; stopped/paused/zero-substep updates shall produce none.
- AC-2: **Rigid mapping.** Fixed rigid, articulation-link and wheel poses shall select position and orientation together, combine with valid shear-free signed scale, and write omni:fabric:worldMatrix without rewriting source omni:xform/reset-stack or authored local transform operations.
- AC-3: **Instancer mapping.** Instancer poses shall write native local positions/orientations with correct quaternion dtype/order and preserve unsimulated holes, trailing rows, scales and prototype indices.
- AC-4: **Shadow mapping.** Other selected values shall write sim:<name> with documented units/frames; admitted deformable/particle output shall have an explicit visible consumer/mapping for sim:points/velocities, and affected render descendants shall be updated before presentation completion.
- AC-5: **Publication fence.** Allocate fresh host-coordinated output ordinals excluded from input draining; presentation readiness shall follow completed reads/writes, write-floor advancement and CPU/CUDA fences. Backend post-step events alone shall not release presentation consumers.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-OUTPUT-001](../../tests/simulation/TEST-RT-OUTPUT-001.md) — Two admitted substeps produce two steps and two publications.
- [TEST-RT-OUTPUT-002](../../tests/simulation/TEST-RT-OUTPUT-002.md) — Fixed poses update world matrices while preserving source authoring.
- [TEST-RT-OUTPUT-003](../../tests/simulation/TEST-RT-OUTPUT-003.md) — Native instancer output preserves holes, tail and quaternion conventions.
- [TEST-RT-OUTPUT-004](../../tests/simulation/TEST-RT-OUTPUT-004.md) — Shadow values and render descendants are ready at publication completion.
- [TEST-RT-OUTPUT-005](../../tests/simulation/TEST-RT-OUTPUT-005.md) — Presentation waits for the output ordinal and device completion.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

## Dependencies

- [ADR-0010](../../adr/ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
- [REQ-RT-INPUT-001](REQ-RT-INPUT-001.md).
- [REQ-RT-SNAPSHOT-001](REQ-RT-SNAPSHOT-001.md).
