<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-OVERLAY-001
title: Render only valid selected collision and simulation representations
status: proposed
owner: physics-ux
---

## Description

Render only valid selected collision and simulation representations. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Filter and geometry.** None/Selected/All, normals and camera-distance/AABB simplification shall act on actual provider collision geometry; schema-only approximation shall be labeled and unavailable geometry shall not be represented as an empty success.
- AC-2: **Joint and tendon ownership.** USD joint icons/attachment editing shall not duplicate omni.usdphysics.ui limit overlays; tendon overlays shall remain PhysX-specific and retain tendon instance identity.
- AC-3: **Deformable modes.** Provider-backed deformable drawing shall distinguish simulation default/bind/rest and collision default/bind plus attachments, disabling unsupported modes explicitly.
- AC-4: **Particle modes.** Particle/fluid-surface/diffuse views shall consume the active completed output and not stale authored positions when ovstage output is active.
- AC-5: **Snapshot lifetime.** Geometry shall carry engine, stage and completed-step validity; borrowed data shall be copied/released before invalidation and old drawings removed on Stop, target/output change or provider unload.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-OVERLAY-001](../../tests/features/TEST-UX-OVERLAY-001.md) — Filter actual collision and joint representations.
- [TEST-UX-OVERLAY-002](../../tests/features/TEST-UX-OVERLAY-002.md) — Draw deformable and particle data from the active completed snapshot.

## Code References

- [debug](../../../../../python/ovphysx_companion/debug.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.
- [usd/meshes](../../../../../python/ovphysx_companion/usd/meshes.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0003](../../adr/ADR-0003-debug-visualization.md).
- [ADR-0005](../../adr/ADR-0005-stage-settings.md).
- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
