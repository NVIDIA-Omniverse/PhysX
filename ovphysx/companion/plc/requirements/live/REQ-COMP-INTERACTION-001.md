<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-INTERACTION-001
title: "Safe interaction and scoped native access"
status: implemented
owner: ovphysx
---

## Description

Runtime interaction binds an explicit borrowed ToolsSession or owned PreviewSession and does not require an inspector UI or preview creation. Pick/drag, collision-aware manipulation and exceptional native access execute under the simulation owner at a safe phase with deterministic restoration.

## Acceptance Criteria

- AC-1: **Native leases.** A native-access lease validates owner, attachment generation and SDK type and permits only documented operations at a quiescent phase or owner execution callback; raw pointers cannot outlive the lease.
- AC-2: **Force semantics.** Interaction defines force/impulse mode, frame, application point, instancing and CPU/GPU support, reusing core writes only where their semantics match.
- AC-3: **Restoration.** Drag end/cancel, deletion and stage close release temporary constraints and restore temporary angular damping and CCD/axis-lock settings, applying the declared velocity-reset policy.
- AC-4: **Scheduling.** Interaction mutations and invalidation share core ordering; waits requiring the same held lease/lock are rejected rather than deadlocking.

- AC-5: **Default force grab.** Python grab/drag/pick default to native managed force picking; D6 mode requires explicit selection. Preserve the existing C interaction ABI and expose force picking through a separate versioned table. The native service retains the picked local point/distance, uses k=50*pickingForce (nonnegative and tunable per grab while idle) and J=k*m*(target-p-0.02*vPoint)*h per actual substep, adds legacy COM gravity assistance, and restores angular damping on cancellation. No joint, axis-lock or CCD changes occur in force mode.
- AC-6: **Ray and lifetime.** Normalize ray directions and retain the original hit distance, owner, generation and exact instance across updates. Invalid/busy/stale commands preserve outputs/state. Input updates never step physics. Close cancels without a due step; retirement exposes inactive metadata. CPU links support preserve-velocity force grabbing, while static/kinematic/disabled/GPU/deformable targets are unavailable. Host-provided filters govern visibility and Pause/Stop/focus events close the grab.

## Test References

- [TEST-COMP-INTERACTION-001](../../tests/live/TEST-COMP-INTERACTION-001.md)
- [TEST-COMP-INTERACTION-002](../../tests/live/TEST-COMP-INTERACTION-002.md)

## Code References

- [Native force/drag table](../../../include/ovphysx_companion/interaction.h) and [bridge](../../../src/Interaction.cpp).
- [Build-matched C++ access](../../../include/ovphysx_companion/native_access.h) and [bridge](../../../src/NativeAccess.cpp).
- [Shared runtime object access](../../../python/ovphysx_companion/inspection.py).
- [Python interaction](../../../python/ovphysx_companion/interaction.py).
- [Native implementation](../../../../ovruntime/source/omni.physx/plugins/NativeInteraction.cpp).
- [Viewport gesture sample](../../../samples/viewport_drag.py) and [compiled read-only consumer](../../../samples/native/read_only.h).

## Advertised behavior

CPU dynamic rigid bodies support force/impulse accumulation for the next host
step, world or actor-local vectors and points, and exact point-instance identity.
Static, kinematic, disabled and GPU dynamics return Unavailable.
Direct force writes and D6 joint drags require dynamic rigid bodies. Managed force
picking also supports CPU articulation links with preserve velocity. grab/drag/pick
default to force, retain a local anchor, and apply legacy impulses and COM gravity
assistance per actual scene substep. Angular damping is temporarily at least
0.025*initial k and restores on close; live coefficient changes affect subsequent impulses only. Ray target is origin+normalized_direction*initial_distance.
No constraint, axis-lock or CCD edits occur in force mode. Explicit joint drag
retains the D6 spring and temporary axis-lock/speculative-CCD restoration.
End velocity preserves, zeros or restores begin velocity on dynamic rigid bodies.
Making a body immovable rejects further target movement while close still restores
transient properties. Deleted/reset sessions expose inactive metadata until close.
Pending close returns Busy and is retried after the host completes work.

SDK callbacks are C++17, build-matched, synchronous and read-only. Do not retain
pointers, mutate even indirectly returned children, enter an interpreter or wait
inside the lease. Python uses copied typed inspection instead. Core waiting and
recursive physics admission reject reentry. SDK type, record generation and exact
owner are checked before the callback. The host calls from idle phases.

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
