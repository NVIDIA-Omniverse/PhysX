<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0006 — Pause without restoring or rebuilding simulation state
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

Pause must preserve a simulation for continuation. Treating it as Stop would discard velocity, solver state and the original play snapshot; continuing hidden updates would make a paused scene move.

## Decision

On the pause hook, stop admitting substeps, finish the current step and its output publication, cancel the active ray interaction, restore temporary interaction properties, then enter Paused. With synchronous updates, the hook runs after that serialized update finishes. A pause received reentrantly is deferred to the next safe completed-substep boundary.

Retain the current play session, including its ovphysx instance, backend ovstage attachment, physics objects, initial play snapshot, completed simulation time, accumulated remainder and valid output resources. Do not restore poses, zero velocities, detach, reset or accumulate paused wall time. Input release/cancellation must not require another physics step.

A repeated pause is a no-op. Pause from Stopped creates no instance or play epoch; pause from Detached has no backend work. Updates in Paused do not step, drain editor changes or publish speculative results. On resume, consume supported pending controls at the simulation boundary and continue the original epoch; the first new elapsed interval contains only resumed time. Held mouse buttons require a new press after cancellation.

When timeline/presenter blocking suppresses a forwarded pause, `enableUpdate=false` still prevents stepping and triggers interaction cancellation. It does not grant an editor-maintenance path or permission to replay missed frames.

## Alternatives considered

- Use Stop as Pause: destroys the continuation state.
- Keep applying a queued grab while paused: changes motion or leaves temporary damping active without user input.

## Consequences

- **Positive:** resume continues the paused simulation with the original Stop baseline.
- **Cost:** cancellation and deferred hook handling must work without another step.
- **Lifecycle:** a failure while completing current work enters Faulted rather than falsely reporting Paused.

## References

- [Resume semantics](ADR-0004-start-resume-state-capture.md).
- [Substep boundaries](ADR-0005-simulation-update-and-substepping.md).
- [Interaction cancellation](ADR-0009-raycast-and-mouse-forces.md).
- [REQ-RT-PAUSE-001](../requirements/simulation/REQ-RT-PAUSE-001.md).
