<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-HOST-001
title: Own contributions and capabilities without implicit simulation
status: proposed
owner: physics-ux
---

## Description

Own contributions and capabilities without implicit simulation. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Stable ownership.** Generic hosts shall key menu/widget/overlay contributions by stable feature ID and engine; reload/unload shall not duplicate registrations or retain callbacks into an unloaded provider.
- AC-2: **No implicit simulation.** Opening, refreshing or browsing UX shall not create, attach or step a simulation session. Generic USD authoring shall work without an engine. Explicit companion compute, Inspector preview and validation operations acquire only their declared resources after capability checks.
- AC-3: **Scoped completion.** All asynchronous UI results and input ownership shall be bound to engine, host-stage/selection and where relevant play generation; stage/selection/provider replacement shall discard stale results and release owned listeners/helpers.
- AC-4: **Runtime boundary.** Playback retains the accepted single-scene Play/Pause/Stop contract. Companion preview has its own explicit native attachment and output destination; PreviewSession owns that native attachment through the bridge while independent public core peers may coexist; it adds no private step/reset path to playback.
- AC-5: **Previously omitted surfaces.** Collision-group/pair-filter authoring and Simulation Info shall have explicit Python/companion workflows under REQ-UX-COLLISION-FILTER-001 and REQ-UX-SIM-INFO-001. Legacy force-load, Fabric-driver toggles and private reset controls are not restored with these views.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-HOST-001](../../tests/features/TEST-UX-HOST-001.md) — Reload provider contributions without creating simulation.
- [TEST-UX-HOST-002](../../tests/features/TEST-UX-HOST-002.md) — Discard completion after stage or provider replacement.
- [TEST-UX-HOST-003](../../tests/features/TEST-UX-HOST-003.md) — Do not register the retired Simulation Info or Collision Groups surfaces.

## Code References

- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.
- [native](../../../../../python/ovphysx_companion/native.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [ADR-0002](../../adr/ADR-0002-viewport-menu.md).
- [ADR-0003](../../adr/ADR-0003-debug-visualization.md).
- [ADR-0004](../../adr/ADR-0004-physics-preferences.md).
- [ADR-0005](../../adr/ADR-0005-stage-settings.md).
- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [ADR-0009](../../adr/ADR-0009-mouse-interaction.md).
- [ADR-0010](../../adr/ADR-0010-authoring-toolbar.md).
- [ADR-0011](../../adr/ADR-0011-physics-inspector.md).
- [ADR-0012](../../adr/ADR-0012-rigid-body-manipulator.md).
- [ADR-0013](../../adr/ADR-0013-mass-manipulator.md).
- [ADR-0014](../../adr/ADR-0014-character-controller.md).
- [ADR-0015](../../adr/ADR-0015-progress-overlay.md).
- [ADR-0016](../../adr/ADR-0016-omnipvd.md).
- [ADR-0017](../../adr/ADR-0017-physx-camera.md).
- [ADR-0018](../../adr/ADR-0018-physx-vehicle.md).
- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [ADR-0020](../../adr/ADR-0020-scene-properties.md).
- [ADR-0021](../../adr/ADR-0021-deformable-properties.md).
- [ADR-0022](../../adr/ADR-0022-tendon-properties.md).
- [ADR-0023](../../adr/ADR-0023-particle-properties.md).
- [ADR-0024](../../adr/ADR-0024-material-properties.md).
- [ADR-0025](../../adr/ADR-0025-demo-browser.md).
- [ADR-0026](../../adr/ADR-0026-authoring-stage-lifecycle.md).
- [ADR-0027](../../adr/ADR-0027-authoring-runtime-provider.md).
- [REQ-RT-DRIVER-001](../../../runtime/requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-STAGE-001](../../../runtime/requirements/simulation/REQ-RT-STAGE-001.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-AUTHOR-STAGE-001](REQ-UX-AUTHOR-STAGE-001.md).
- [REQ-UX-AUTHOR-PROVIDER-001](REQ-UX-AUTHOR-PROVIDER-001.md).
