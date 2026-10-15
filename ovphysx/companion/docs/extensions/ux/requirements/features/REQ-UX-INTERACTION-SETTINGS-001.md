<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-INTERACTION-SETTINGS-001
title: Share input and distance settings across UX surfaces
status: proposed
owner: physics-ux
---

## Description

Share input and distance settings across UX surfaces. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Enablement.** Disabling mouse interaction shall immediately cancel an active drag and clear feedback; toggles in other panels shall show the same effective value.
- AC-2: **Mode values.** Grab/Ignore Invisible/With Force, force coefficient 0–10 and push acceleration 0–1000 shall be validated and routed to the selected owning provider; unsupported modes shall be unavailable rather than fallback.
- AC-3: **Distance.** Visualization distance shall accept 1–10000 stage distance units and share one effective model across stage, viewport and debug controls.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-INTERACTION-SETTINGS-001](../../tests/features/TEST-UX-INTERACTION-SETTINGS-001.md) — Use validated shared grab and visualization settings.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [interaction](../../../../../python/ovphysx_companion/interaction.py) — existing companion substrate; host integration remains proposed.
- [queries](../../../../../python/ovphysx_companion/queries.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0005](../../adr/ADR-0005-stage-settings.md).
- [ADR-0009](../../adr/ADR-0009-mouse-interaction.md).
- [REQ-RT-PICK-001](../../../runtime/requirements/simulation/REQ-RT-PICK-001.md).
- [REQ-RT-FORCE-001](../../../runtime/requirements/simulation/REQ-RT-FORCE-001.md).
- [REQ-UX-STAGE-SETTINGS-001](REQ-UX-STAGE-SETTINGS-001.md).
