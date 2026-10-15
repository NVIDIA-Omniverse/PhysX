<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-AUTHOR-STAGE-001
title: Associate authoring providers through attach-detach-only Umbrella hooks
status: proposed
owner: physics-ux
---

## Description

Associate authoring providers through attach-detach-only Umbrella hooks. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Association.** Host stage open/late provider registration shall create one generation-scoped Python EditSession association, idempotently. Association and UI refresh shall allocate no native simulation attachment; explicit stopped computations may initialize their supported data services.
- AC-2: **Teardown.** Replacement shall cancel stale delivery, drain old jobs, close preview/output leases and then close the old EditSession before associating the next generation. Failed association shall roll back only its acquired resources; generic USD authoring remains usable.
- AC-3: **Separation.** Authoring association shall not receive playback update/resume/reset dispatch. Play/Pause/Stop operates on playback A; explicit Inspector/validation activation may create independent native owner B through the companion. Neither owner may collect, step or publish the other owner's work.
- AC-4: **Publication.** Jobs shall capture stage/selection/topology revision and apply copied results through DeferredEdit/EditSession only when current. OutputNotices shall suppress output feedback; ordinary authored edits shall still reach the sealed playback input path. A disabled stage-update callback is not an authoring pump.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-AUTHOR-STAGE-001](../../tests/features/TEST-UX-AUTHOR-STAGE-001.md) — Attach an authoring provider independently of simulator activation.
- [TEST-UX-AUTHOR-STAGE-002](../../tests/features/TEST-UX-AUTHOR-STAGE-002.md) — Drain authoring jobs before replacement and isolate attach failure.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.
- [jobs](../../../../../python/ovphysx_companion/jobs.py) — existing companion substrate; host integration remains proposed.
- [exchange](../../../../../python/ovphysx_companion/exchange.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0026](../../adr/ADR-0026-authoring-stage-lifecycle.md).
- [REQ-RT-STAGE-001](../../../runtime/requirements/simulation/REQ-RT-STAGE-001.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
