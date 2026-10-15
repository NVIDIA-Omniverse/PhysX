<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0008 — Tear down any play session and forget the detached stage
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

The Kit detach hook runs when the stage closes or is replaced. It may follow Stop, occur before any Play, or interrupt a Running/Paused session. Backend detach is necessary only when Play created a simulation session; stage detach must always clear the remembered host-stage identity.

## Decision

Separate **simulation-session teardown** from **forgetting the host stage**. Stop uses the first operation and retains the stage; Kit detach or provider shutdown performs both.

For an active or partially initialized play session:

1. Reject new session commands and invalidate queued input/completions against the simulation generation.
2. Complete submitted simulation and output work while their resources and the host stage still exist.
3. Cancel interaction and restore temporary properties while actors remain valid; release adapter-created constraints/targets and remove feedback.
4. Close `OvStageOutputCache`, release borrowed views/bindings and output work, and remove all play-session observers/subscriptions.
5. Call `ovphysx_detach_ovstage` if backend attach succeeded, then destroy the adapter-owned ovphysx instance. Never release borrowed PhysX actors or the host-owned ovstage itself. No instance is retained for the next Play.
6. Clear ray, substep and other simulation-session state. Stop retains the copied play snapshot until host-state restoration finishes; stage close may discard it when restoration is not required.

After session teardown, Kit detach drops the remembered host-stage reference/identity and stage lifecycle subscriptions, invalidates the host-stage generation and enters Detached. If the stage was already stopped or never played, there is no backend session: only this bookkeeping cleanup runs. Do not create an instance merely to call detach.

Repeated detach is harmless. Every subsequent Play creates a fresh simulation generation, views and cache even if the same stage remained open through Stop. A new stage open establishes a fresh host-stage generation. No ray hit, pointer, view or old completion crosses either invalidation boundary.

If cleanup fails, retain only resources needed to complete it and report Faulted rather than freeing a stage underneath pending work. Unregister provider callbacks and unload owned libraries only after cleanup completes; shared host framework/stage infrastructure is not destroyed by a per-session teardown.

Stage close does not promise restoration of a stage being discarded. A caller requiring a restored live stage must complete Stop first. Stop keeps its host-stage reference throughout restoration and ends in Stopped with no backend; stage detach ends in Detached with no remembered stage.

## Alternatives considered

- Unconditionally forward every Kit detach to ovphysx: assumes stage open created a backend attachment and misses the never-played/already-stopped cases.
- Release the stage before draining work: permits use-after-free and output into another stage generation.
- Keep the instance until provider shutdown: fails to destroy all play-session resources on Stop.

## Consequences

- **Positive:** both stage-close paths and Stop share ordered teardown without conflating their stage-lifetime behavior.
- **Cost:** track whether instance creation and backend attach succeeded so partial initialization can be cleaned up safely.
- **Lifecycle:** Stop forgets simulation; stage detach additionally forgets the host stage.

## References

- [Stage-open bookkeeping](ADR-0003-ovstage-attach.md).
- [Play-time creation](ADR-0004-start-resume-state-capture.md).
- [Stop restores and retains the host stage](ADR-0007-stop-and-state-restoration.md).
- [Interaction resources](ADR-0009-raycast-and-mouse-forces.md).
- [Output resources](ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-DETACH-001](../requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
