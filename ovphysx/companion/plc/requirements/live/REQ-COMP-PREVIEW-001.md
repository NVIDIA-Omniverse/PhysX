<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-PREVIEW-001
title: "Isolated preview and explicit commit or restore"
status: implemented
owner: ovphysx
---

## Description

Inspector owns a separate native ovruntime attachment and a distinct preview ovstage in the same process as playback. The companion bridge manages that attachment within the existing runtime/SDK service lifetime. Public core instances may independently attach distinct ovstages. PreviewSession continues to own its native attachment through the companion bridge rather than constructing another public PhysX owner.

## Acceptance Criteria

- AC-1: **Ownership.** When qualified, the companion preview service creates and owns native attachment B alongside playback A through the existing runtime bridge. Its exact handle is used for every operation; B cannot be an implicit active attachment; PreviewSession owns B through the bridge while independent public core peers remain permitted. Missing capability selects an explicit worker fallback or reports unavailable.
- AC-2: **Snapshot.** B uses a distinct ovstage instance, retained source dictionary/ordinals and revision-checked mapping to the authored source. Equal paths/local keys in A and B remain distinct. Selection/topology edits, undo and stage close invalidate pending results; a duplicate live ovstage attachment is rejected without affecting A.
- AC-3: **Restore and commit.** Preview output has its own destination or temporary layer. Cancel removes B's temporary output/state without restoring stale snapshots over live playback. Commit applies only selected fresh results through Python reversible USD edits; rest data and unrelated opinions remain unchanged. Explicit exclusive-main-owner fallback retains capture/restore semantics.
- AC-4: **Transitions.** Pause/resume, Play, undo/redo and close drain or invalidate the selected preview's work. Each attachment has one step coordinator and each destination one writer; transitions never reset, detach or consume playback A's pending results to dispose of B.
- AC-5: **Execution isolation.** A and B can have independently pending native simulate/fetch intervals. Conflicting work on B returns Busy without modifying either owner; B's callbacks, queries, views and debug/output frames resolve only B. Callback lifecycle reentry is rejected without blocking. No broader public-facade thread-safety guarantee is inferred.
- AC-6: **Configuration admission.** Preview passes copied attachment/resource options without changing global defaults. Shared tolerance/provider incompatibility fails before candidate publication; owner-local thread/device choices are capability-checked and hard CPU-only policy remains effective. Failure leaves A's settings, objects and output unchanged.
- AC-7: **Lifetime integration.** The bridge retains process services through both owners' work and supports teardown in either order without invalidating the survivor. Preview retirement drains only its resources and does not stop an unrelated recording. Preview attachment never starts or restarts recording; the core owner must start it explicitly. An open preview keeps its ovstage Stage alive independently of its Python owner until native close succeeds, so a cyclic garbage collection without explicit close cannot finalize the Stage first. Shared recording transitions and final shutdown are coordinated across core and companion users before the in-process preview capability is advertised.

## Test References

- [TEST-COMP-PREVIEW-001](../../tests/live/TEST-COMP-PREVIEW-001.md) - Inspector snapshot/output and host transitions, AC-1 through AC-4; rigid-body snapshot/commit subset executed.
- [TEST-COMP-PREVIEW-002](../../tests/live/TEST-COMP-PREVIEW-002.md) - independent execution and candidate compatibility, AC-1, AC-5 and AC-6; native CPU stepping/configuration subset executed.
- [TEST-COMP-PREVIEW-003](../../tests/live/TEST-COMP-PREVIEW-003.md) - mixed core/companion lifetime and recording, AC-1 and AC-7; both retirement orders and recording subset executed.

The companion native bridge now passes independent pending intervals, duplicate-source rejection, independent public peers and duplicate-source rejection, native close preserving playback receipts, and both retirement orders with/without recording in fresh processes. Articulation preview, copied inspection/query data and ordered events now have native integration coverage. Dedicated native and explicit worker modes are qualified. Exclusive-main-owner preview is not exposed; the operation matrix records GPU/device exclusions.

## Code References

- [Preview C table](../../../include/ovphysx_companion/preview.h)
- [Owned native attachment](../../../src/Preview.cpp)
- [Python preview session](../../../python/ovphysx_companion/preview.py)
- [Mixed integration tests](../../../tests_native/test_preview.py)
- [Customer USD snapshot/commit](../../../python/ovphysx_companion/usd/preview.py)
- [Explicit worker](../../../python/ovphysx_companion/preview_worker.py)
- [USD integration tests](../../../tests/test_preview_usd.py)
- [Inspector native sample](../../../samples/inspector_preview.py)

Native/core changes retain their own component traceability.

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/src/ovphysx/ovphysx.cpp](../../../../src/ovphysx/ovphysx.cpp)
- [ovphysx/python/ovphysx/utils/simulation.py](../../../../python/ovphysx/utils/simulation.py)

- [ovphysx/ovruntime/include/omni/physx/IPhysxSimulation.h](../../../../ovruntime/include/omni/physx/IPhysxSimulation.h)
- [ovphysx/ovruntime/include/omni/physx/AttachOptions.h](../../../../ovruntime/include/omni/physx/AttachOptions.h)
- [ovphysx/ovruntime/include/omni/physx/RuntimeResult.h](../../../../ovruntime/include/omni/physx/RuntimeResult.h)
- [ovphysx/ovruntime/source/omni.physx/plugins/RuntimeExecution.h](../../../../ovruntime/source/omni.physx/plugins/RuntimeExecution.h)

- [host.py](../../../python/ovphysx_companion/host.py)
- [test_host.py](../../../tests_native/test_host.py)

## Dependencies

- [REQ-COMP-CONFIG-001](../architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-OVSTAGE-001](../exchange/REQ-COMP-OVSTAGE-001.md)
- [Companion ADR-0007](../../adr/ADR-0007-preview-ownership.md)

## Implementation scope

Independent native CPU preview coexists with playback and retains its own ovstage,
resource options and process lease. Host coordination, pending work, foreign/stale
references, recording and retirement in both orders are tested. Customer-USD
capture/worker/selected commit supports rigid bodies and articulation links.
PreviewDisplay elects one writer for an empty local output layer and clears only
its own output on cancel/close, preserving newer input. Play/selection/undo invalidate
captured work. Worker fallback is explicit; exclusive-main-owner preview and
unqualified GPU execution are not exposed.

## Snapshot and edit limits

Capture retains composed physics inputs, relationship dependencies, the collision
environment and ancestor transforms before serializing, without flattening
unrelated render payload. Selection validation is bounded by path depth rather
than all selection pairs. Numeric preview time freezes the retained inputs;
commits refuse animated transform stacks before any edit, preserving animation
until an explicit host mapping is supplied. Authored collection rules travel with
their copied relationships, so material-binding and mesh-merge membership match
the source; a mesh-merge collider retains its member meshes as collision
geometry. A resolved material outside the traversal (inactive, abstract or
undefined) stays absent, so its collider remains effectively unbound as at
runtime. Worker source checkouts are recognized by the package's own
`pyproject.toml`, which the public export ships. Preview thread overrides are 1..255.
Attachment identities are published under their owning registry lock; no native
attach runs while that lock is held. An idle unrelated core may be destroyed while
another owner records and a preview is pending.
