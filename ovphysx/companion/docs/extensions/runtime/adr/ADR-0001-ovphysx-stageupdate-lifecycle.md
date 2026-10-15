<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0001 — Drive runtime simulation through the generic stage-update hooks
status: accepted
date: 2026-09-18
last_updated: 2026-10-01
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

The existing generic stage-update extension forwards Kit stage and timeline events through Umbrella to registered `StageUpdateFns`. The PhysX provider must replace legacy simulation forwarding with ovphysx while preserving the simulation lifecycle. Authoring services are a separate concern.

## Decision

Use `omni.physics.stageupdate` as the sole Kit driver, Umbrella as dispatcher, and a simulation adapter in the runtime extension group as the ovphysx owner. Replace the provider callbacks; do not register another `omni.physx.stageupdate` node, late Fabric node, or background stepping loop. Retain generic presenter/timeline gating.

Assume exactly one physics scene in the stage. Serialize hooks, ovphysx operations and publication through one adapter owner. While stopped, retain only the remembered host-stage identity/reference. On Play, create one ovphysx simulation session with a live ovstage attachment, an immutable start-state snapshot, one substep accumulator, ray interaction state and output resources. Preserve that session across Pause; destroy it completely on Stop. ovphysx owns simulation; the adapter owns lifecycle policy, state restoration, scheduling, mouse-force calculation and output to ovstage.

| Operation | Decision | Detail |
| --- | --- | --- |
| Attach / stage open | Remember which live stage to load. Create no simulation instance, backend attachment or physics objects. | ADR-0003 |
| Resume / start | From Stopped, capture current initial state, create the simulation instance and load the live ovstage through `ovphysx_attach_ovstage`. From Paused, resume the existing session without reloading or recapturing. | ADR-0004 |
| Update, `enableUpdate=true` | Simulate only while Running; respect scene timestep, accumulated time and substeps. | ADR-0005 |
| Update, `enableUpdate=false` | Unsupported as an editor-maintenance operation. Ignore the forwarded callback without stepping, input draining, cooking/query servicing, output or time accumulation; cancel an existing interaction safely if gating changed. | ADR-0005 |
| Pause | Finish current work, preserve state and suspend stepping. | ADR-0006 |
| Stop / `onReset` | Restore initial transformations and velocities, including published state, and destroy all simulation-session resources. Keep only the stage identity/reference for later Play. | ADR-0007 |
| Detach / stage close | Tear down any active simulation session, then forget the stage. If already stopped, only clear stage bookkeeping. | ADR-0008 |
| Ray input | Raycast through ovphysx and migrate legacy mouse-force calculation into the adapter. | ADR-0009 |
| Output | Runtime publishes completed simulation results to ovstage using ovphysx output utilities. | ADR-0010 |
| Force load | Unsupported legacy operation; leave its callback unregistered. | This decision |
| Reset simulation | Unsupported; leave `resetSimulation` unregistered. Timeline Stop through `onReset` remains supported and distinct. | This decision |
| Release objects | Unsupported; leave `releasePhysicsObjects` unregistered. Stop and stage detach own session teardown. | This decision |

The state model is **Detached → Stopped → Running ↔ Paused**. Stage open enters Stopped without loading simulation. Play loads and enters Running; Pause retains the loaded session; Stop restores state, destroys the session and returns to Stopped without reattaching. Stage close enters Detached. Failures enter **Faulted**, prohibit subsequent simulation/output and retain only resources needed to finish cleanup/restoration. Recovery must finish that work before returning to Stopped or Detached; a subsequent Play creates a fresh session. Do not report a successful provider transition merely because Umbrella broadcasts timeline intent.

Duplicate attach to the same generation, start while Running, pause while Paused, Stop without a play epoch and detach while Detached are idempotent. Changing stage identity requires orderly detach/attach. Track host-stage and play-session generations separately; invalidate simulation callbacks and identifiers on every Stop even when the host stage remains open. Provider disable/shutdown drains and detaches before callback storage or libraries disappear.

**Out of scope:** stopped/editor updates, forced parsing, standalone reset/release APIs, Inspector preview, cooking/property-query services, controller/camera/vehicle feature migration, debug services and an independent explicit-step API. UX proposals do not expand this simulation contract.

## Companion scope clarification — 2026-10-01

The one-scene and no-stopped-session rules constrain this playback adapter and its owner A. They do not prohibit an explicitly requested companion Inspector/validation attachment B, nor stopped data-only cooking. B has independent input/output, tasks and lifecycle and cannot use this adapter to step A while stopped. The core facade still rejects a second live facade attachment. Native-owner and process-bootstrap accounting must distinguish these services; stage-open callbacks still create no playback instance.

See [Companion preview ownership](../../../../plc/adr/ADR-0007-preview-ownership.md) and [authoring provider](../../ux/adr/ADR-0027-authoring-runtime-provider.md).

## Alternatives considered

- Keep the broad runtime-services proposal: mixes simulation lifecycle with unrelated authoring and feature services.
- Delegate Stop to ovphysx reset: ovphysx does not supply the required initial-state restoration.
- Retain a separate PhysX/Fabric stage-update driver: introduces competing scheduling and lifecycle ownership.

## Consequences

- **Positive:** each supported operation has a concrete owner and separate decision.
- **Cost:** implement snapshot/restore, substep scheduling, interaction and output in the adapter; these are not supplied by attach/step alone.
- **Lifecycle:** remove unsupported capabilities from callers; null callbacks must not be presented as successful load/reset/release actions.

## References

- [Attach](ADR-0003-ovstage-attach.md).
- [Start and resume](ADR-0004-start-resume-state-capture.md).
- [Update and substepping](ADR-0005-simulation-update-and-substepping.md).
- [Pause](ADR-0006-simulation-pause.md).
- [Stop](ADR-0007-stop-and-state-restoration.md).
- [Detach](ADR-0008-ovstage-detach.md).
- [Ray input](ADR-0009-raycast-and-mouse-forces.md).
- [Output](ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-DRIVER-001](../requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
