<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0007 - Inspector preview in a dedicated runtime attachment
status: accepted
date: 2026-09-30
last_updated: 2026-10-05
supersedes: none
relates_to: [ADR-0001, ADR-0002, ADR-0003, ADR-0006]
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Ovruntime supports several live ovstage attachments, each with its own scenes,
object database, source, callbacks, output and execution admission. Independent
attachments can step on separate client threads and have pending work at the
same time. They share the runtime/SDK services and compatible SDK-wide settings.

At the original design baseline, the ovphysx C/Python facade rejected a second live facade attachment.
Native multi-attachment support therefore changes the Inspector architecture,
but that initial integration did not make a second Python `PhysX` object sufficient.
Current trunk permits multiple public owners; the amendment below retains the
dedicated companion preview contract alongside them.

## Decision

1. **Default to a dedicated native preview attachment.** The companion preview
   service creates, owns, steps and retires attachment B through the versioned
   bridge in the same loaded runtime that hosts playback A. Borrowed tools
   sessions continue to operate only on their selected owner. No extra runtime
   DLL, SDK singleton or global active-attachment selector is introduced.
2. **Use a distinct ovstage instance for B.** Populate a snapshot of the selected
   content and its required physics dependencies into a preview-owned stage,
   retaining its dictionary and ordinals through detach. The runtime rejects
   attaching the same live ovstage instance twice. Resolve every object as
   attachment plus source-local identity, even when A and B have identical paths.
3. **Keep execution local to the selected attachment.** The preview session has
   one scheduler for B's attach/update/simulate/fetch/read/query/reset lifecycle.
   Core tasks remain the route for A. Same-owner conflicts and forbidden callback
   reentry report Busy; independent native owners need no process-wide step lock.
   This does not promise new thread-safety guarantees for the public facade.
4. **Negotiate resources before publication.** Pass explicit copied attachment
   and resource options; never temporarily rewrite process defaults to configure
   Inspector. Honor shared tolerance scale, recording-provider compatibility and
   hard CPU-only policy. Owner-specific dispatchers/devices may differ where
   supported. An incompatible candidate fails without altering A.
5. **Keep preview output isolated until commit.** Display B through a separate
   output destination or temporary layer. Capture source revision and mapping;
   commit only selected fresh results through customer-USD Python edit batches
   with host undo. Cancel destroys B's temporary output and state. Play, selection
   changes and undo invalidate stale results without resetting playback A.
6. **Integrate service lifetime before advertising support.** Retain the shared
   runtime while either owner or its work is live. Retiring B releases only B's
   scenes, views and callbacks; it cannot shut down the SDK or finalize A's
   recording. Coordinate process-wide recording transitions and final shutdown
   across both core and companion users. Core-only lifecycle paths do not
   establish that mixed core/companion integration is complete.
7. **Keep a capability-gated fallback.** Use a worker process when the bridge,
   lifetime integration or required backend/configuration combination is
   unavailable. Exclusive stopped playback reuse is an explicit alternate mode
   with capture/restore. A second public `PhysX` is available as another provider
   option, but replacing PreviewSession with it requires separate qualification.

**Out of scope**

- Replacing the dedicated PreviewSession owner with a public PhysX provider.
- Cross-attachment collisions/joints or automatic sharing of mutable scene data.
- Claiming preview readiness, backend qualification or legacy-runtime coexistence
  merely from the native multi-attachment implementation.

## Alternatives considered

- Keep worker processes mandatory: adds transport and process lifetime work when
  native attachments already provide the required scene separation.
- Construct a second Python `PhysX`: originally rejected; now possible, but it
  does not supply the dedicated preview API's options and lifecycle contract.
- Attach the playback ovstage again: violates source ownership and permits
  competing output; a preview needs its own stage.
- Call native getters ad hoc: bypasses explicit
  lifecycle, recording and configuration integration needed by the companion.

## Consequences

- **Positive:** Inspector can coexist with playback in one process with separate
  physics state, clocks, event streams and output.
- **Costs:** Implement the preview attachment session, stage snapshot/mapping,
  runtime service lease, process-transition coordination and mixed core/native
  isolation tests before advertising the capability.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence.

### Public multi-attachment amendment (2026-10-05)

The public facade now permits independently attached core instances with distinct
ovstage sources. PreviewSession continues to own its native attachment through
the companion bridge. Core peers and companion preview share the loaded runtime
without a process-wide public attach latch. Duplicate sources remain rejected;
retiring a public peer cannot retire playback or preview. Shared recording,
tolerance and final-shutdown constraints still require coordinated admission.

## References

- [REQ-COMP-PREVIEW-001](../requirements/live/REQ-COMP-PREVIEW-001.md)
- [REQ-COMP-OWNER-001](../requirements/architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-CONFIG-001](../requirements/architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-JOBS-001](../requirements/exchange/REQ-COMP-JOBS-001.md)
- [REQ-CAPI-ATTACH-OWNER-001](../../../plc/requirements/capi/REQ-CAPI-ATTACH-OWNER-001.md) - public per-instance attachment ownership.

## Snapshot and animation constraints (2026-10-01)

Capture composed physics prims, relationship dependencies and ancestor transforms
before serialization. Include the collision environment and connected articulation
state, but exclude unrelated render meshes and shader payload. Never flatten the
whole customer stage as an intermediate. Validate selection through ancestor
membership, with cost proportional to path depth. Numeric-time simulation freezes
physics inputs at that USD time. Pose commits refuse animated transform stacks
before editing; preserving arbitrary animation requires an explicit host mapping.

Capture preserves resolved physics material binding, face subsets, inactive
PointInstancer IDs and drive/limit/state values. Bake each resolved collider and
subset binding separately, preserving child material distinctions. Carry typed
particle properties and authored API metadata as data; execution qualification
and worker schema registration remain explicit consumer requirements.

A numeric capture of a static body commits a default-time pose; it must not
create animation that prevents a later fresh commit. Reject both time samples
and zero-sample spline animation before editing any selected destination.
Attachment construction preserves a retryable Busy result from configuration
admission separately from invalid-payload or construction failure.
