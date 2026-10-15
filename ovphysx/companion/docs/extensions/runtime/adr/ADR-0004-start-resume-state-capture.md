<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0004 — Load simulation and capture initial state on Play; preserve both across resume
status: accepted
date: 2026-09-18
last_updated: 2026-10-05
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

Legacy omni.physx saved initial transformations and linear/angular velocities, including authored-property presence, for later Stop restoration. ovphysx attach and step do not implement that Kit policy. Capturing output after the first step loses the original state.

## Decision

Make the runtime adapter own an immutable **play snapshot**, separate from `OvStageOutputCache`. Both `onResume` and the supported `startSimulation` callback enter this same transition.

From Stopped, perform the following serialized Play transition:

1. Resolve the remembered host-stage identity to its current live ovstage and retain it for the session. If unavailable or replaced, fail Play without loading a filename or another stage. Required physics schemas must have been registered by the host before stage population.
2. Finish host authoring/evaluation needed for the initial transforms and select a nonzero **sealed** input ordinal containing the latest stage contents, including edits since open or the previous Stop. Capture the initial state described below from that consistent input before loading simulation or publishing output. Validate the single physics scene's timestep and admitted object mappings.
3. Create a fresh adapter-owned ovphysx instance and call `ovphysx_attach_ovstage(handle, stage, ordinal)` once. This is when the stage is loaded and physics objects are created. Retain that instance's exact attachment. Distinct instances may attach different sources concurrently; attaching the same ovstage twice fails without displacing its owner. The initial ordinal is consumed by attach and is not drained again.
4. Establish a fresh simulation generation, zeroed substep state and session-owned interaction/output resources. Enter Running only after loading and initialization succeed. Do not advance dynamics as part of Play; subsequent enabled updates perform the scheduled steps.

If initialization fails, drain and destroy all partially created simulation resources and restore any changed presentation state. Return to Stopped only after cleanup succeeds; otherwise enter Faulted and retain the resources needed to finish cleanup. A failure never leaves a hidden simulation running. The C API borrows the ovstage pointer for this play epoch; retain the actual `ovstage.Stage` when using Python output helpers.

| State to capture | Required representation |
| --- | --- |
| Rigid poses and transforms | Initial position/orientation and the exact affected source transform representation: local transform operations/order/reset-stack semantics, scale and derived world matrices where present. Restoring a world pose alone must not change parent-relative authoring. |
| Rigid motion | Initial linear and angular velocity, frames/units, authored value and whether the property existed. Missing properties restore to absence, not a newly authored zero. |
| Point-instancer motion | Initial positions, orientations, linear/angular velocities, index identity and original array extent; retain unchanged scale and unsimulated slots. |
| Other simulated state | For each admitted articulation, deformable or particle output, retain its restart inputs: root/joint state or initial points/velocities, plus property presence. Do not pretend rigid-body state covers these types. |
| Published output | Original value/presence of attributes the adapter will overwrite, including world matrices and `sim:*` shadows. Track exactly which attributes and instance slots are simulation-owned. |

Copy values into adapter-owned storage; do not retain borrowed tensor/stage views. Key entries by host-stage and play-session generations, prim identity and instance index. Capture both authoritative restart inputs and original presentation state so Stop can restore the stage and display after destroying simulation, without stepping or reloading it. A simulated type lacking a complete capture/restore mapping fails admission before any output overwrites its state.

Start establishes zero accumulated remainder and a new simulation epoch. Resume from Paused retains the existing instance and ovstage attachment, original snapshot, solver state, completed-step time and remainder. It neither reloads the stage nor captures the paused pose as a new initial pose. Resume/start from Detached fails because no stage is remembered. A duplicate start while Running changes nothing.

If supported live changes introduce an object during Play, capture its initial state before its first output overwrite. Removing an object drops its runtime entry; Stop does not resurrect removed prims. Structural changes that cannot preserve identity require Stop followed by a new Play/load rather than a best-effort remapping.

## Alternatives considered

- Rely on ovphysx or the output cache: neither is the immutable Kit initial-state snapshot.
- Snapshot on every resume: makes Stop return to the last pause instead of the start of Play.
- Zero velocities on Stop: loses nonzero initial motion and authored-property semantics.

## Consequences

- **Positive:** Stop can reproduce the original pose and motion after any number of pauses.
- **Cost:** each Play after Stop creates and loads a fresh simulation session; implement typed snapshot capture and explicit admission of restorable object kinds.
- **Lifecycle:** retain the snapshot until successful Stop or detach; a capture failure leaves the stage unadvanced.

## References

- [Remembered stage](ADR-0003-ovstage-attach.md).
- [Scheduling validation](ADR-0005-simulation-update-and-substepping.md).
- [Stop restoration](ADR-0007-stop-and-state-restoration.md).
- [Published state](ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-PLAY-001](../requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-SNAPSHOT-001](../requirements/simulation/REQ-RT-SNAPSHOT-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
