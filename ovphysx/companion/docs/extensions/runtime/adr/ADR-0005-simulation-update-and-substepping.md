<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0005 — Own scene-rate substepping in the runtime update adapter
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

Umbrella already exposes scene timestep discovery. Legacy omni.physx accumulated elapsed time, bounded catch-up using the minimum frame rate, and executed forces and callbacks on each substep. ovphysx step uses the explicit simulation path without that frame-level substep scheduler; one Kit frame cannot be mapped blindly to one step.

## Decision

Accept updates only in Running with `enableUpdate=true` and a successfully loaded play session. Updates in Stopped never create an instance or load the remembered stage; only Play performs that transition. A false flag provides no editor-maintenance service and accumulates no elapsed time; cancel held interaction safely when gating closes. Zero elapsed time performs no physics step or output. Reject negative/nonfinite elapsed time or nonfinite timeline time without advancing or accumulating it. Pause/gating never creates a catch-up debt.

Assume exactly one physics scene. Read its timestep through Umbrella's existing timestep facility and maintain one rate and remainder for the play session. Preserve timestep resolution: authored PhysX timestep, then authored Newton fallback, then the recognized scene default of 60 Hz. A zero discovery result means missing/invalid scene information, not permission to divide by zero. Validate the effective rate before entering Running.

### Accumulation and catch-up

For the single physics scene, let `f` be its steps per second, `h = 1/f`, `r` its previous remainder, `dt` the admitted elapsed seconds and `m` the configured minimum frame rate. Reproduce the legacy policy:

```text
available = r + dt
n = floor(available / h)
if m > 0:
    cap = max(1, floor(f / m))
    n = min(n, cap)
    next_remainder = min(available - n*h, cap*h)
else:
    next_remainder = available - n*h
```

Carry `next_remainder` across enabled updates and pause/resume. It can exceed one step when catch-up is capped; do not replace it with modulo. Clear it on Stop, detach or a new play epoch. Freeze timestep/minimum-frame-rate configuration for a play epoch; edits take effect after Stop/start. Use sufficiently precise accumulation and avoid integer overflow for large elapsed intervals; do not introduce an undocumented cap when `m=0`.

At 60 Hz, two 1/120-second updates produce zero then one step. With 60 Hz, minimum frame rate 30 and a 0.1-second update from zero remainder, execute two steps, retain 2/60 second and discard the remaining 2/60 second. Dropped wall time never advances solver time.

### Substep ordering

1. Determine due substeps and consume only new sealed **control-input** ordinals before the affected simulation work; exclude all physics-output ordinals.
2. For each due substep, evaluate its control state, invoke ordered pre-step callbacks, apply the current ray command/forces for `h`, call ovphysx for **exactly one** interval `h`, and wait for completion. Preserve the legacy pre-step-before-ray ordering.
3. Publish output and deliver completed-result notifications in the order defined by ADR-0010. Each subscription has one notification owner: do not broadcast a second callback if ovphysx already supplies it. Backend post-step events mean simulation completion; presentation consumers wait for the additional output fence. Do not batch with a multi-step API when it would omit per-substep input, force or callback work.
4. Advance completed solver time/count only for successful steps. On any simulation/publication failure, enter Faulted; never retry a partly applied interval as though it did not run.

Legacy frame sampling placed the last substep at Kit `currentTime` and preceding samples at `currentTime - (n-1-i)*h`. Preserve this ordering when evaluating time-sampled host input. ovphysx tracks its own simulation time internally; do not pass frame elapsed time as a replacement for `h`, or claim its clock equals the Kit clock. Seek/rewind requires Stop and a new epoch. If the host cannot materialize distinct sealed inputs for those samples, animated-input parity is a prerequisite, not an implicit property of stepping.

## Alternatives considered

- Pass frame elapsed time to ovphysx: ignores authored fixed steps and makes interaction frame-rate dependent.
- Run another accumulator inside each ovphysx step: duplicates scheduling already performed by the adapter.

## Consequences

- **Positive:** deterministic step counts and legacy catch-up/force cadence are explicit.
- **Cost:** migrate the single-scene accumulator and catch-up policy into the adapter.
- **Lifecycle:** pause retains remainder; Stop starts a fresh epoch; failed updates do not run later queued work.

## References

- [Simulation-only updates](ADR-0001-ovphysx-stageupdate-lifecycle.md).
- [Play epochs](ADR-0004-start-resume-state-capture.md).
- [Pause](ADR-0006-simulation-pause.md).
- [Per-substep ray forces](ADR-0009-raycast-and-mouse-forces.md).
- [Output publication](ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-STEP-001](../requirements/simulation/REQ-RT-STEP-001.md).
- [REQ-RT-INPUT-001](../requirements/simulation/REQ-RT-INPUT-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
