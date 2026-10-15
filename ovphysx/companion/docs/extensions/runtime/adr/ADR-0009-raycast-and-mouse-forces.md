<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0009 — Migrate ray picking and mouse-force computation into the runtime adapter
status: accepted
date: 2026-09-18
last_updated: 2026-10-01
supersedes: none
---

## Status

Accepted for the runtime simulation design. Implementation is governed by the linked requirement and test specifications.

## Context / Motivation

The generic hook supplies a ray and button state. A raycast alone does not reproduce omni.physx interaction: the old runtime retained a picked point, calculated mass-scaled impulses, damped motion, compensated gravity and cleaned up temporary state. These calculations must move with the simulation integration.

## Decision

Use companion managed force picking as the default grab. Keep input/timeline policy and hit filtering in the host runtime adapter; keep the retained picked anchor, force equations, transient damping and cleanup in the native companion/runtime service. Use companion scene queries and exact owner/object identities. D6 joint grab is an explicit alternative. Never create a separate interaction stepping loop.

### Input and picking

Validate finite origins and nonzero directions, normalize direction, and use the legacy maximum distance of 1,000,000 stage length units. Choose the closest eligible hit, respecting collision/visibility filtering; if visibility filtering removes a closest hit, query/filter all hits before choosing. Static/kinematic/disabled targets receive no dynamic impulse. Preserve object and point-instancer identity; do not treat opaque hit IDs as prim paths. Copy needed hit values and resolved paths before borrowed storage becomes invalid.

Accept new interaction only in Running with a loaded play session; input while Stopped never loads physics or creates a picker. A press begins interaction using the configured mouse mode and picking/push settings, held input updates the target, and release terminates it. Keep one latest target and terminal release state per grab and play-session generation; no function-static picker shared across sessions. An initial grab captures the local picked point, hit distance and saved angular damping; subsequent due substeps apply forces to that same body rather than repicking another body. Push mode may raycast each application, as before.

Apply commands only between completed steps, immediately before the next scheduled substep, with that substep's `h`. An update with zero due substeps applies no force. A held command persists across due substeps; release cancels it even when no step is due. Pause, Stop, detach, target removal and presenter/timeline blocking perform the same cleanup and require a fresh press.

### Force model to migrate

All quantities below are in the simulation's consistent stage units. Let `m` be body mass, `d` the original pick distance, `u` the normalized ray direction, `p` the current world position of the stored local picked point, and `c` the world center of mass. Let `v` and `omega` be COM linear velocity and angular velocity in radians/second.

| Behavior | Legacy calculation to preserve per substep |
| --- | --- |
| Push | Apply impulse `J = u * pushAcceleration * m * h` at the current hit position. |
| Grab target | `target = rayOrigin + u*d`; `vPoint = v + omega × (p-c)`. |
| Grab force | `k = 50 * mousePickingForce`; `action = target-p-0.02*vPoint`; apply impulse `J = k*m*action*h` at `p`, including the resulting moment about COM. |
| Angular damping | Save original damping once; use `max(original, 0.025*initial k)` during the grab; restore the saved value on every termination path. |
| Gravity assistance | If gravity is enabled and `length(g)>0`, set `a = -g * max(0, length(g)-0.1*k*d)/length(g)` and add COM velocity change `a*h`. Otherwise add none. |

The force coefficient is nonnegative and may change while the grab is held. Each substep uses its latest value; angular damping is established once using the coefficient at press and restores on release. A zero coefficient produces no spring impulse but retains the legacy gravity assistance.

Preserve impulse semantics: the legacy implementation deliberately avoids retaining applied accelerations across steps. Do not multiply a force by `h` twice, omit point-velocity damping, apply gravity compensation at the picked point, substitute zero initial velocity or forget to restore angular damping. Wake the target after releasing a grab and remove its feedback.

The native service executes these equations for each selected CPU scene's actual substep, after input callbacks and before simulation. It uses an exact-generation CPU rigid-body record, including articulation links, and applies impulses rather than retained force/acceleration writes. The host supplies pointer targets, not mass, velocity, COM or computed forces. Explicit world-target updates support manipulators; normalized ray updates preserve the initial hit distance. Releasing a grab restores damping and removes its repeating target even when no substep is due. Final velocity is preserved by default; explicit zero/restore policies apply to dynamic rigid bodies, while links support preservation only.

GPU/DirectGPU force picking remains unavailable until damping, mass/COM/velocity reads and nonpersistent point impulses are all supported. Deformable picking is separately gated. Unsupported modes never fall back to joint dragging. Host filtering supplies visibility/collision selection rules; renderer visibility has no native dependency.

### Other legacy picking modes

Keep the existing joint-grab mode explicit rather than silently converting it to a spring force. Its temporary kinematic target and free-axis D6 joint use `tau=0.16`, damping `D=2*m/tau`, stiffness `K=D*D/(4*m)`, linear force cap `factor*m*3000` and angular cap `factor*length(inertia)*3.14`, with `factor=0.3` for a body with additional constraints and `1` otherwise. Move both target and kinematic pose to avoid injecting target velocity. Release adapter-owned joint/target objects on termination.

Deformable picking additionally requires the legacy barycentric hit/attachment/filter path and target radius of 10 stage length units; a rigid ray hit is not equivalent. These modes require verified public access for their object/attachment operations. If that access or DirectGPU force parity is unavailable, report that interaction mode as unavailable before accepting a drag; simulation may continue. Do not claim whole-feature parity by implementing only hit testing.

## Companion execution boundary

Companion `Interaction.grab`, `drag` and ray `pick` default to force grabbing; explicit joint mode selects D6 dragging. A separate versioned C picking table preserves the original interaction table layout and joint behavior. The native attachment owns the shared force/joint uniqueness registry. Deletion, detach and owner replacement retire the native grab before actor destruction. Pause/Stop and focus/input cancellation close the host-held grab.

Push retains its independent impulse contract; the host may use companion force-at-point primitives for that workflow. Managed force grabbing does not claim deformable picking or complete joint-mode constant parity.

## Alternatives considered

- Forward a generic grab request to an unspecified provider: leaves the required force behavior unimplemented.
- Apply forces once per Kit frame: makes the result depend on render cadence and catch-up count.
- Retain the legacy no-substep ray-force pump: contradicts the simulation-only update policy; cancellation remains available without stepping, but force application requires a due substep.
- Use scene-query IDs as durable native pointers: violates identity and attachment lifetimes.

## Consequences

- **Positive:** push and force-grab behavior have concrete equations and cleanup semantics to migrate.
- **Cost:** port the force/picker code and verify each execution mode; native joint/deformable modes need their own supported object access.
- **Lifecycle:** only adapter-created temporary objects are released; borrowed simulation objects and output views are invalidated on detach.

## Open questions

1. DirectGPU parity needs a verified route for temporary angular damping, mass/COM/velocity reads and nonpersistent point impulses. The proposed initial behavior is explicit mode unavailability until all components are supported, not reduced force behavior.

## References

- [Native force picking](../../../../plc/requirements/live/REQ-COMP-INTERACTION-001.md).
- [Per-substep execution](ADR-0005-simulation-update-and-substepping.md).
- [Pause cancellation](ADR-0006-simulation-pause.md).
- [Detach and pointer lifetime](ADR-0008-ovstage-detach.md).
- [REQ-RT-PICK-001](../requirements/simulation/REQ-RT-PICK-001.md).
- [REQ-RT-FORCE-001](../requirements/simulation/REQ-RT-FORCE-001.md).
