<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MENU-PROVIDER-001
title: Route specialized creation to its declared authoring provider
status: proposed
owner: physics-ux
---

## Description

Route specialized creation to its declared authoring provider. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Particles.** Particle-system creation shall require an advertised schema/command and author only that provider's system; lack of the capability shall not start simulation or silently use another engine.
- AC-2: **Deformables.** Surface/volume/attachment creation shall validate selected meshes/endpoints and use an explicit authoring/cooking operation with completion/failure status; unsupported requests shall author nothing.
- AC-3: **Vehicle actions.** Create Vehicle shall invoke the owning PhysX wizard; tire-friction-table creation shall validate material targets and be independently undoable without changing unrelated materials.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MENU-PROVIDER-001](../../tests/features/TEST-UX-MENU-PROVIDER-001.md) — Route specialized creation without a simulation instance.
- [TEST-UX-MENU-PROVIDER-002](../../tests/features/TEST-UX-MENU-PROVIDER-002.md) — Reject missing services and invalid creation targets.

## Code References

- [usd/deformables](../../../../../python/ovphysx_companion/usd/deformables.py) — existing companion substrate; host integration remains proposed.
- [usd/particles](../../../../../python/ovphysx_companion/usd/particles.py) — existing companion substrate; host integration remains proposed.
- [usd/vehicle](../../../../../python/ovphysx_companion/usd/vehicle.py) — existing companion substrate; host integration remains proposed.
- [usd/attachments](../../../../../python/ovphysx_companion/usd/attachments.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
