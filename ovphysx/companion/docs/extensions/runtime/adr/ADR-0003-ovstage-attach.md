<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0003 — Remember the opened stage without loading simulation
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

The generic stage attach hook runs when a stage opens, before the user presses Play. Calling `ovphysx_attach_ovstage` at that point would parse and create simulation objects while the application is only editing. Simulation must exist only within a play epoch, including its paused intervals.

## Decision

Treat the Kit attach hook as **stage bookkeeping only**. Remember the host's live stage identity and its lifetime/lookup reference, then enter Stopped. No ovphysx instance, backend attachment, simulation objects, play snapshot, input consumer or output cache exists in this state.

1. Record the opened stage and a new host-stage generation. Keep enough host-owned identity to resolve its current live ovstage at Play; a USD stage-cache ID is not an ovstage pointer.
2. Do not call `ovphysx_attach_ovstage`, parse/cook physics, reserve or consume a simulation input ordinal, capture initial state or create simulation resources on stage open. Do not freeze the stage contents at open time.
3. Changes made while stopped remain host authoring changes. The next Play resolves the latest stage, validates its readiness and loads its then-current sealed contents under ADR-0004.
4. A duplicate attach for the same host-stage generation changes nothing, including during Running or Paused. A different stage first completes the old stage's detach path, then remembers the new stage in Stopped. Never infer Play from an attach hook alone.
5. Closing a stage that was never played simply clears its remembered identity/reference. No backend detach is necessary when no simulation session was created.

Keep host-stage identity separate from the simulation generation. Each Play after Stop creates a new simulation generation for the same remembered stage; Stop destroys that generation while leaving the stage remembered for later Play.

## Alternatives considered

- Load ovphysx on stage open: allocates and parses simulation before it is requested.
- Cache a file or stage-content snapshot at open: ignores edits before Play and loses the live stage identity.
- Add a force-load transition: reintroduces an explicitly unsupported legacy operation.

## Consequences

- **Positive:** opening or editing a stage has no simulation allocation or loading cost.
- **Cost:** resolve and validate the live ovstage when Play is requested, rather than assuming it was loaded at open.
- **Lifecycle:** Stopped retains only stage bookkeeping; Play creates simulation, Stop destroys it, and stage detach forgets the stage.

## References

- [Simulation boundary](ADR-0001-ovphysx-stageupdate-lifecycle.md).
- [Play-time loading and state capture](ADR-0004-start-resume-state-capture.md).
- [Stop teardown](ADR-0007-stop-and-state-restoration.md).
- [Stage detach](ADR-0008-ovstage-detach.md).
- [REQ-RT-STAGE-001](../requirements/simulation/REQ-RT-STAGE-001.md).
