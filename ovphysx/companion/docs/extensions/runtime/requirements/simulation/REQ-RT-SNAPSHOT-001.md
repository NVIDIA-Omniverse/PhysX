<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-SNAPSHOT-001
title: Capture independent initial state for each play epoch
status: proposed
owner: physics-runtime
---

## Description

Capture independent initial state for each play epoch. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Capture boundary.** Before backend loading or output mutation, the adapter shall copy the initial state from the same evaluated/sealed host input used for Play; the snapshot shall not borrow backend/stage views or alias OvStageOutputCache.
- AC-2: **Rigid authoring.** The snapshot shall retain initial pose, linear/angular velocity, units/frames, source transform representation/order/reset-stack/scale and property presence, including initially absent velocity and output attributes.
- AC-3: **Instancer state.** The snapshot shall identify simulated point-instancer slots and retain their initial positions, orientations, velocities, original array extent and publication state while distinguishing untouched slots and scale/prototype data.
- AC-4: **Type admission.** Every admitted articulation/deformable/particle type shall have complete restart-input and presentation mappings: root/joint state or initial points/velocities as appropriate. A type without complete capture/restore and visible-output mappings shall fail admission before output mutation.
- AC-5: **Epoch identity.** Snapshot entries shall use host-stage and play-session generations plus prim/instance identity, survive pause/resume unchanged, capture supported new objects before first output, and never resurrect removed prims. Unmappable structural changes require Stop and a new Play.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-SNAPSHOT-001](../../tests/simulation/TEST-RT-SNAPSHOT-001.md) — Capture exact rigid authoring before simulation touches it.
- [TEST-RT-SNAPSHOT-002](../../tests/simulation/TEST-RT-SNAPSHOT-002.md) — Instancer capture distinguishes simulated slots from untouched data.
- [TEST-RT-SNAPSHOT-003](../../tests/simulation/TEST-RT-SNAPSHOT-003.md) — Admission requires restart and visible-output mappings for every selected type.
- [TEST-RT-SNAPSHOT-004](../../tests/simulation/TEST-RT-SNAPSHOT-004.md) — Object identity changes do not corrupt the play baseline.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).

## Dependencies

- [ADR-0004](../../adr/ADR-0004-start-resume-state-capture.md).
- [REQ-RT-STAGE-001](REQ-RT-STAGE-001.md).
