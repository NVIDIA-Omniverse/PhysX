<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-STOP-001
title: Destroy simulation and restore initial host state on Stop
status: proposed
owner: physics-runtime
---

## Description

Destroy simulation and restore initial host state on Stop. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Destroy before stopped.** onReset from Running/Paused shall drain work, cancel interaction, close output caches/views/subscriptions, detach the backend and destroy the instance; successful Stopped shall retain no physics objects, play-session buffers, snapshot or output resources.
- AC-2: **Restore copied state.** After backend teardown, Stop shall restore copied source transforms and initial linear/angular velocities, including authored presence and transform representation, plus admitted instancer/articulation/deformable/particle inputs. Restoration shall not require live actor/tensor setters.
- AC-3: **Restore presentation.** Stop shall restore/remove simulation-owned world matrices, instancer slots and sim:* shadows to their original state/presence, complete derived descendant updates, and preserve unrelated author edits, unsimulated slots and removed prims.
- AC-4: **No reload.** Stop shall complete host restoration writes and enter Stopped retaining only the host-stage identity/reference; it shall not step, reattach, reparse, recreate the instance or allocate output resources. A reset-on-stop preference shall not bypass restoration.
- AC-5: **Repeated epochs.** Stop before the first step shall still destroy the loaded session; Stop with no epoch shall be a no-op and not undo subsequent stopped edits. The next Play shall capture/load the then-current host state in a fresh simulation generation.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-STOP-001](../../tests/simulation/TEST-RT-STOP-001.md) — Stop restores rigid state and destroys the entire session.
- [TEST-RT-STOP-002](../../tests/simulation/TEST-RT-STOP-002.md) — Stop restores dynamic instancer slots without overwriting authoring holes.
- [TEST-RT-STOP-003](../../tests/simulation/TEST-RT-STOP-003.md) — Stop restores admitted non-rigid restart inputs after destroying physics.
- [TEST-RT-STOP-004](../../tests/simulation/TEST-RT-STOP-004.md) — Play then Stop without a step leaves no latent objects for the next epoch.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

## Dependencies

- [ADR-0007](../../adr/ADR-0007-stop-and-state-restoration.md).
- [REQ-RT-SNAPSHOT-001](REQ-RT-SNAPSHOT-001.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-DETACH-001](REQ-RT-DETACH-001.md).
