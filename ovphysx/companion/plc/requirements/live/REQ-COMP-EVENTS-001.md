<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-EVENTS-001
title: "Ordered contact trigger and runtime event delivery"
status: implemented
owner: ovphysx
---

## Description

Deliver immutable runtime/object/contact/trigger events to Python and host graph adapters through an explicit bounded queue. Events are data, not permission to mutate the solver from arbitrary callbacks.

## Acceptance Criteria

- AC-1: **Phases.** Subscriptions declare pre-step, substep or post-results delivery; each event carries owner/generation, step/substep and documented ordering relative to object deletion and completed output.
- AC-2: **Semantics.** Contact filtering and trigger enter/stay/leave semantics preserve pair and instance identity; runtime error events are typed and remain distinct from log messages.
- AC-3: **Overflow.** Bounded queues report overflow and its documented recovery policy; event loss cannot occur silently.
- AC-4: **Disposal.** Unsubscribe/session close drains or discards queued events according to the published policy and prevents delivery after completion; Python/Kit presentation is marshalled onto an approved thread.

## Test References

- [TEST-COMP-EVENTS-001](../../tests/live/TEST-COMP-EVENTS-001.md).

## Code References

- [C event table](../../../include/ovphysx_companion/events.h)
- [Native owner/queue bridge](../../../src/Events.cpp)
- [Python queue and host delivery](../../../python/ovphysx_companion/events.py)
- [Extension-style event monitor](../../../samples/event_monitor.py)

## Implementation scope

Callback phases describe native production; Python only polls after completed
results on its creating host thread. Contact/trigger reporting is explicitly
authored, never silently enabled by monitoring. Contact and trigger point-instancer indices are
preserved through the copied native queue. Trigger
stay derives only from observed enter pairs. Subscribe before stepping for a
complete overlap history. Process errors have attachment zero because their
native stream has no reliable owner origin. Overflow fails closed with explicit
recovery instructions; disposal discards pending events. GPU contact/trigger
subscriptions are unavailable. Eight native Python integration cases and both native source-backend fixtures pass.

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)

## Trigger membership and history loss

Track subshape membership counts per public collider/instance pair. Derived Stay
continues after a partial compound leave and stops after the last leave. Count a
pair-capacity rejected record once. C poll returns EVENT_HISTORY_LOST with a valid
diagnostic batch; Python raises EventOverflow unless diagnostic draining is chosen.
Named kind/mask constants and the SDK-free checked C++ contact accessor preserve
tag-specific payload semantics without changing the copied record layout.
