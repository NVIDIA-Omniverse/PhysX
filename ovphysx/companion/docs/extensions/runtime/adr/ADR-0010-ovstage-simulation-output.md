<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0010 — Publish simulation results to ovstage from the runtime adapter
status: accepted
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

Stepping ovphysx does not automatically publish results to ovstage. ovphysx already provides `step_and_write_to_ovstage` and `OvStageOutputCache`; the adapter should use these utilities instead of inventing a second Fabric stage-update driver. Their output is a mixture of rendered pose attributes and simulation shadows, with explicit ordinal/lifetime rules.

## Decision

Make output part of the runtime simulation update and completion boundary. Use `ovphysx.utils.step_and_write_to_ovstage` with an actual attached `ovstage.Stage`, optionally with an adapter-owned `OvStageOutputCache`. Create these resources only after Play loads the simulation; stage open and stopped updates allocate none. A native runtime bridge must share the utility's implementation or reproduce this contract through public read/ovstage APIs; private legacy Fabric interfaces are not the output path.

### Exactly one step and one output transaction per substep

For each substep admitted by ADR-0005: consume its sealed controls, perform pre-step work and then ray forces, reserve a fresh output ordinal and call the helper once with `dt=h`. The helper performs the simulation step, waits, reads results, writes output and advances the write floor. Then complete the publication fence before downstream presentation consumers are released. **Do not call `step_sync` first and then call the helper to publish:** that would perform two steps.

There are exactly `n` dynamics steps and `n` output transactions for `n` admitted substeps. Presentation consumers observe completed published state; the last substep is the final frame output. Backend post-step subscriptions may fire before publication and must not be mistaken for a render-ready fence or emitted twice. A native composition uses the same step → wait → read → write → seal/complete ordering. Zero-substep updates, disabled updates and paused frames do not call this stepping helper. Stop writes its captured values directly; `dt=0` is not a restore operation.

### Output mapping

| Output kind | Existing utility behavior / integration obligation |
| --- | --- |
| Rigid bodies, articulation links, vehicle wheels | Read position and orientation together; combine them with scale derived from the current world matrix and write `omni:fabric:worldMatrix`. The helper does not write `omni:xform` or `omni:resetXformStack`. Preserve local source authoring. |
| Point-instancer rigid bodies | Write instancer-local native `positions` and `orientations`; preserve unsimulated slots, trailing rows, scales and prototype indices. Respect native quaternion dtype/order conversions. |
| Rigid/link velocities, articulation root/joint state | Write selected values to `sim:<name>` shadow attributes with their documented units/frames. These are not automatic authored `physics:*` updates. |
| Deformable/particle points and velocities | Existing defaults write `sim:points` and `sim:velocities`. Visible mesh/particle consumers must explicitly consume these shadows or use a defined output mapping; the helper alone does not update every render geometry attribute. |

Start from the utility's documented default output groups and select only admitted, restorable types. Fixed pose output always selects both position and orientation. Before first publication, the host must supply valid evaluated world matrices and a compatible scale decomposition. The utility preserves shear-free signed scale; do not advertise arbitrary sheared-transform support without a defined mapping.

The utility does not propagate descendant world transforms. Runtime output completion must include the host's required transform propagation for attached render descendants and explicit consumers for non-rigid shadows. Those mappings are prerequisites to declaring a type visibly supported, rather than reasons to add another physics stepping node.

### Ordinal and resource ownership

- Input ordinals contain sealed application controls; consume each through `ovphysx_update_from_ovstage` at most once. Its range is inclusive; an omitted start means only the endpoint. Never span an output ordinal while batching an input range.
- Output ordinals contain physics results and are **never** drained back as control input. Coordinate allocation with the host, identify provenance explicitly and complete the write-floor/fence boundary before rendering or reusing data.
- Release borrowed read views before incompatible stage mutation and observe CPU/CUDA completion semantics. A seal alone does not authorize reuse of GPU buffers while work is outstanding.
- Bind the output cache to one exact attachment. It contains copied scale/instancer baselines and reusable buffers/events, not permanent borrowed views and not the initial-state snapshot. Refresh or recreate it after supported authored transform, pose or topology changes; close and destroy it during Stop or stage detach. The next Play creates fresh output resources for its new simulation generation.
- If publication fails after the step completed, do not rerun the step. Enter Faulted and withhold a success notification; stage writes may already be partial, so recovery needs the retained initial snapshot or orderly detach.

## Alternatives considered

- Keep a separate legacy Fabric output node: splits lifecycle, output ordering and ownership from the sole runtime adapter.
- Copy USD transforms after every step: ignores the existing ovstage utilities and can overwrite authoring representations.
- Publish only once per frame using the stepping helper after all steps: adds an unintended extra step. A future write-only utility could permit final-only output, but is not assumed here.

## Consequences

- **Positive:** existing ovphysx utilities provide the baseline writeback, with explicit simulation and rendering boundaries.
- **Cost:** per-substep publication, host ordinal coordination and render/descendant mappings; optimize publication frequency only without changing step count or callback visibility.
- **Lifecycle:** Stop restores both source inputs and output presentation and destroys all output resources without reloading simulation; pause preserves the session and its last completed output; detach drains GPU/output work before closing resources.

## References

- [Immutable initial-state snapshot](ADR-0004-start-resume-state-capture.md).
- [Substep scheduling](ADR-0005-simulation-update-and-substepping.md).
- [Stop publication](ADR-0007-stop-and-state-restoration.md).
- [Detach](ADR-0008-ovstage-detach.md).
- [REQ-RT-INPUT-001](../requirements/simulation/REQ-RT-INPUT-001.md).
- [REQ-RT-OUTPUT-001](../requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-RT-CACHE-001](../requirements/simulation/REQ-RT-CACHE-001.md).
- [REQ-RT-FAULT-001](../requirements/simulation/REQ-RT-FAULT-001.md).
