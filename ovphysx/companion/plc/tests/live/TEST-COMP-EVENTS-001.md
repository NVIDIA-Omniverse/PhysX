<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-EVENTS-001
maps_to: REQ-COMP-EVENTS-001
type: integration
---

## Coverage status

Implemented and executed. Covers [REQ-COMP-EVENTS-001](../../requirements/live/REQ-COMP-EVENTS-001.md) AC-1 through AC-4.

## Scenario

Observe a body entering and leaving a trigger while generating contacts, then overflow and dispose its subscription.

## Given

- A deterministic stepping fixture, two substeps per host update, a filtered contact pair and a deliberately small event queue.
- An event recorder with thread and object-generation fields.

## When

- Step through entry/contact/exit, generate more events than queue capacity, delete an object and unsubscribe with queued events pending.

## Then

- Recorded phase and substep order matches the documented contract and completed output (REQ AC-1).
- Expected enter/leave and filtered contact pairs carry correct identities; injected runtime errors are typed (REQ AC-2).
- Queue exhaustion produces an explicit overflow record/status and documented recovery (REQ AC-3).
- No callback runs after unsubscribe completes, and Python/host delivery occurs on its approved thread (REQ AC-4).

## Executable coverage

- [Companion cases](../../../tests_native/test_events.py): real trigger enter/stay/leave and contact filtering, phase ordering, pending peer isolation, sticky overflow, creating-thread checks, close-during-host-dispatch, retirement copies, distinct point-instance pairs, articulation-link leave and ABI/output canaries.
- [Event monitor sample](../../../samples/event_monitor.py): frame coordinator plus host graph delivery.

Executed: ten native Python integration cases pass; the C++ source-backend shard and existing ContactReport, ContactsAndTriggers and SimulationExtended shards pass. GPU contact/trigger subscriptions are unavailable;
no unavailable hardware run is counted as passing qualification.

`test_compound_collider_contact_filter` additionally exercises filtered contact
queues on convex-decomposition and sphere-fill child colliders (AC-2).
