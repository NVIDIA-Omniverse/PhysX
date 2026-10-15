<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PREVIEW-001
maps_to: REQ-COMP-PREVIEW-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-PREVIEW-001; executable evidence and qualification limits follow.

## Scenario

Preview an articulation, edit selection and transition to Play while a preview result is pending.

## Given

- Playback A with a live core attachment, saved authored opinions and an Inspector preview service capable of owning native B through the same runtime bridge.
- A separate preview ovstage with an articulation, equal source paths to A, a revision mapping, isolated output layer and per-attachment step/output recorder.
- A fallback-capability variant where in-process preview is unavailable.

## When

- Create B, preview/pause/resume, change selection or topology and attempt to commit an older result; attempt a duplicate attach of A's ovstage.
- Cancel B, create a fresh preview, transition to Play and commit/undo/redo fresh selected output. Close preview while A has pending work, then fetch A normally.
- Repeat opening with the in-process capability absent.

## Then

- Qualified preview owns B without replacing A; the unavailable capability selects an explicit worker fallback or reports unavailable, with independent public core peers permitted under their own attachment contract. (REQ AC-1)
- Equal paths resolve against their own stage/attachment; stale results and duplicate live ovstage attachment are rejected without changing A. (REQ AC-2)
- Cancel removes preview-owned output without overwriting playback; fresh commit changes only selected dynamic opinions and undo restores them, preserving rest data. (REQ AC-3)
- There is one coordinator per attachment and writer per destination; Play/close never resets A or consumes its pending result, and no preview callback follows close. (REQ AC-4)

## Executable coverage

[test_preview_usd.py](../../../tests/test_preview_usd.py):

Seven USD cases cover rigid/articulation worker snapshots, selected commit/undo, Play invalidation, and transient output cancellation without overwriting newer input. Native preview/host/event/inspection tests exercise independent owners, duplicate source refusal, pause/resume and closing B with A pending. Only dedicated native and worker modes are advertised.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

## Review regressions

Customer-USD tests reject animation-destructive commit before editing, exclude
100000 unrelated render points while retaining material/joint/body dependencies,
and validate 2048 sibling selections with a bound on public prefix comparisons.
They retain real worker simulation and selected undo/redo checks.


Round-two regressions in `test_preview_usd.py` preserve strong ancestor and
collection bindings, face-material subsets, authored/positional inactive instance
IDs, nondefault drive/limit/state inputs, repeatable default-time commits from a
numeric capture, zero-sample spline animation, and interpreter-local worker imports.
Native integration verifies independent previews alongside concurrent attachment
admission, with retryable Busy distinguished from construction failures.

`test_preview_retains_typed_and_unknown_particle_input` preserves particle type,
unprefixed values and authored unknown API metadata. Source/snapshot material
cases also compare actual native mass; stopped-mass regressions compare complete
inertia tensors with the native runtime in an isolated oracle worker.

Round-three regressions compare source and snapshot natively: inactive bound
physics and all-purpose materials (on the selection and an unselected collider)
stay unbound and undefined; a collider outside an `explicitOnly` material
collection keeps default density while the `expandPrims` control binds; and a
mesh-merge collider whose member mesh has no physics schema gives the same raycast
hit. `test_public_layout_preview_worker_uses_exported_checkout` runs the real
worker from a copied public layout without `AGENTS.md`.

A plain Xform collider whose child Mesh and Cube carry no physics schema keeps both
children as collision geometry in the snapshot; source and snapshot raycasts hit
each at the same distance
(`test_preview_retains_geometry_of_non_gprim_collider`).
