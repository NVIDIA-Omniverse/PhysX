<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-HOST-001
title: "Kit adapter owns UI scheduling commands and presentation"
status: implemented
owner: ovphysx
---

## Description

Keep legacy UI facades in the host adapter while replacing their physics requests with companion/core capabilities. The companion remains usable without Kit.

## Acceptance Criteria

- AC-1: **UI ownership.** Widgets, selection, viewport rendering, graph registration, input devices and joint billboard/manipulator registration remain in the host; solver joint registration is a separate native capability.
- AC-2: **Timeline.** The host coordinates attach/update/step/read/output/pause/reset with one coordinator per attachment; umbrella and PhysX stageupdate adapters cannot both step the same attachment. Inspector has a separate preview owner and output destination, so its loop does not consume playback tasks or reset playback.
- AC-3: **Authoring coordination.** Batch collider creation, progress/cancellation, metrics/units conversion, notice suppression and undo use shared jobs/Python helpers on the host-approved thread.
- AC-4: **Optional features.** The host queries capabilities and disables unavailable actions with specific diagnostics instead of depending on surviving header names, private interfaces or test hooks.

## Test References

- [TEST-COMP-HOST-001](../../tests/host/TEST-COMP-HOST-001.md) - seven headless coordinator and deferred-edit cases pass.

## Code References

- [Host coordinator and deferred edits](../../../python/ovphysx_companion/host.py)
- [Real native host tests](../../../tests_native/test_host.py)
- [Customer-USD deferred edit tests](../../../tests/test_host_edits.py)
- [Shared stageupdate sample](../../../samples/host_stageupdate.py)

## Implementation scope

FrameCoordinator elects one approved-thread scheduler per native attachment and
deduplicates adapter ticks. DeferredEdit owns cancellation/stale publication policy.
ColliderBatch reuses core authoring, prepares bounded world-bound metrics, converts
meter contact offsets, and commits one reversible batch after full preflight.
OutputNotices suppresses output/undo/rollback feedback. Actual table capability
probes gate live actions. Concrete Kit widgets, command registration and devices
remain host responsibilities, as specified by the ADR.

## Dependencies

- [REQ-COMP-EVENTS-001](../live/REQ-COMP-EVENTS-001.md)
- [REQ-COMP-CONFIG-001](../architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
