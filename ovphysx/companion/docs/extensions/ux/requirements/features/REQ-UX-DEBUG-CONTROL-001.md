<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEBUG-CONTROL-001
title: Route debug simulation controls through supported timeline behavior
status: proposed
owner: physics-ux
---

## Description

Route debug simulation controls through supported timeline behavior. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Timeline controls.** Run/Pause/Stop shall use generic timeline/stage hooks; ovphysx Step, force load, resetSimulation and releasePhysicsObjects actions shall be disabled with a reason and make no backend calls.
- AC-2: **Overrides.** CPU/GPU/solver and synchronous-CUDA-debug overrides shall show effective applicability and restart/reload or safe-boundary requirements; rejected changes shall retain the old effective state.
- AC-3: **No forced diagnostics.** Opening/debug inspection shall not load physics or use test-only interfaces. With no loaded session or optional diagnostic service, dependent views shall show unavailable rather than force-load or start a session.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-DEBUG-CONTROL-001](../../tests/features/TEST-UX-DEBUG-CONTROL-001.md) — Run pause and stop through the generic hooks.
- [TEST-UX-DEBUG-CONTROL-002](../../tests/features/TEST-UX-DEBUG-CONTROL-002.md) — Reject unsupported live overrides without lying about effective state.

## Code References

- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.
- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [preview](../../../../../python/ovphysx_companion/preview.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [REQ-RT-DRIVER-001](../../../runtime/requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-PAUSE-001](../../../runtime/requirements/simulation/REQ-RT-PAUSE-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
