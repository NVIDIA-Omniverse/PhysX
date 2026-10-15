<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0007 — Restore initial state and destroy simulation on Stop
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

Kit Stop is forwarded as `onReset`, whereas `resetSimulation` is a separate legacy operation. Stop must restore initial transformations and velocities and leave no loaded simulation behind. ovphysx does not own the Kit initial-state snapshot; the adapter must restore the host stage while tearing down its play session.

## Decision

Always restore the adapter's start-state snapshot on Stop, including nonzero initial linear/angular velocities, and destroy all simulation-session resources. A reset-on-stop preference cannot disable restoration. Do not register or call the legacy `resetSimulation` or `releasePhysicsObjects` operations.

Use **drain, destroy, restore**. Keep the host stage open and remembered; do not recreate an ovphysx instance, reattach, reparse or create output resources at the end of Stop.

1. Stop admitting work, complete submitted simulation/output operations and cancel interaction, including temporary damping, constraints and feedback. Keep the host ovstage alive throughout cleanup and restoration.
2. Close output caches, release borrowed views and bindings, remove session subscriptions, and call `ovphysx_detach_ovstage` if the session attached successfully. Destroy the adapter-owned ovphysx instance. Invalidate its simulation generation and all runtime identities, callbacks, ray commands, buffers and substep state. Retain the adapter's copied snapshot only until restoration completes.
3. Restore snapshot-owned source transforms and velocities through host ovstage writes. Restore actual initial values, property presence and transform representation. Do not replace absent velocities with authored zeros or write world matrices into local transform slots.
4. Restore point-instancer simulated slots without changing unsimulated slots; restore initial articulation root/joint and admitted deformable/particle state from their captured inputs. Restoration writes to the host stage and requires no live actors or tensor setters.
5. Restore or remove simulation-owned presentation attributes (`omni:fabric:worldMatrix`, changed instancer poses and `sim:*` shadows) according to their original presence. Recompute affected derived transforms/descendants before declaring the stopped display ready. Preserve unrelated author edits and deleted prims; no whole-stage rollback.
6. Seal and complete restoration, release restoration buffers and the play snapshot, and enter Stopped. Only the host-stage identity/reference and stage lifecycle bookkeeping remain. No simulation instance, backend attachment, physics objects, output cache or play-session subscriptions remain.

Stop from Running or Paused performs the same teardown. Stop after Play but before the first step still destroys the created session without a warm-up step. Stop without an active epoch is a no-op and repeated Stop never undoes later edits. A subsequent Play captures a new baseline and loads the current stage, including stopped-state edits.

Restoration ordinals belong to the host and retain distinct authoritative-input versus presentation-only provenance. No backend consumes them during Stop. The next Play selects a new sealed initial input containing the restored state and subsequent author edits; it does not replay old simulation-output ordinals.

On teardown or restoration failure, enter Faulted, retain only resources needed to finish cleanup/restoration, report the failed phase and block Play. Do not recreate simulation as a recovery shortcut or report successful Stop while resources remain live or state is only partly restored.

## Alternatives considered

- Reattach after restoring: recreates simulation while stopped and repeats load work that belongs to the next Play.
- Keep an empty ovphysx instance or cached backend objects for reuse: retains play-session resources after Stop.
- Call ovphysx reset or only write live rigid-body state: does not restore the complete host-stage and presentation contract.

## Consequences

- **Positive:** stopped stages have their initial pose/motion and no live simulation resources.
- **Cost:** subsequent Play pays the instance creation and stage-loading cost; restoration needs copied state independent of destroyed backend objects.
- **Lifecycle:** Stop retains host-stage identity but ends the simulation generation. Shared application infrastructure is not owned or shut down by this per-play teardown.

## References

- [Initial snapshot and Play-time loading](ADR-0004-start-resume-state-capture.md).
- [Session teardown and stage detach](ADR-0008-ovstage-detach.md).
- [Remembered stage](ADR-0003-ovstage-attach.md).
- [Output restoration](ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-STOP-001](../requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
