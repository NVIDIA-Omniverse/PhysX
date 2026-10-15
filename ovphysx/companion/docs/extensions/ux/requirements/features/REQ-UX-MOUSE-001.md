<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MOUSE-001
title: Keep each viewport drag owned by one engine
status: proposed
owner: physics-ux
---

## Description

Keep each viewport drag owned by one engine. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Dispatch.** The generic input host shall send world-space ray plus press/held/release intent to the selected capable engine, retain that owner until end, and never fallback to hidden PhysX interaction.
- AC-2: **Feedback.** Hit/instance identity and grabbed/target/push markers shall use generation-tagged completed provider feedback and disappear when stale or unavailable.
- AC-3: **Cancel.** Focus loss, input disable, Pause/Stop, selection/stage replacement or provider removal shall deliver one cancellation and release backend transient state before discarding UI ownership; no new drag shall begin while ovphysx is stopped.

- AC-4: **Force semantics.** Preserve legacy push/grab coefficient, acceleration, damping, gravity and ignore-invisible selection policy under runtime ADR-0009. Use companion managed force grabbing by default on exact instances; native code owns its force law and restoration. Push retains its separate impulse workflow. D6 drag is a separately labelled explicit mode and is not accepted as proof of force-law parity.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MOUSE-001](../../tests/features/TEST-UX-MOUSE-001.md) — Keep a drag with its selected engine and completed hit identity.
- [TEST-UX-MOUSE-002](../../tests/features/TEST-UX-MOUSE-002.md) — Cancel interaction on interruption without starting stopped physics.

## Code References

- [queries](../../../../../python/ovphysx_companion/queries.py) — existing companion substrate; host integration remains proposed.
- [interaction](../../../../../python/ovphysx_companion/interaction.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0009](../../adr/ADR-0009-mouse-interaction.md).
- [REQ-RT-PICK-001](../../../runtime/requirements/simulation/REQ-RT-PICK-001.md).
- [REQ-RT-FORCE-001](../../../runtime/requirements/simulation/REQ-RT-FORCE-001.md).
- [REQ-RT-PAUSE-001](../../../runtime/requirements/simulation/REQ-RT-PAUSE-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
