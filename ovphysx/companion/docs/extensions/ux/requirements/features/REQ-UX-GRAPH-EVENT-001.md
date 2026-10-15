<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-EVENT-001
title: Preserve contact and trigger semantics without silent unsupported success
status: proposed
owner: physics-ux
---

## Description

Preserve contact and trigger semantics without silent unsupported success. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Contact events.** Generic contact nodes shall retain actor/collider/instance identity and found/persist/lost semantics from completed provider events, copying borrowed data before invalidation.
- AC-2: **Trigger capability.** Trigger nodes shall preserve enter/leave, watched-path/catch-all selection and exact owner/object identity through companion EventQueue. Missing requested event fields or unsupported modes shall report unavailable, not an empty successful stream.
- AC-3: **Subscription lifetime.** Stop, node deletion and engine/stage change shall unsubscribe exactly once, clear validity and discard late events from the previous subscription.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-EVENT-001](../../tests/features/TEST-UX-GRAPH-EVENT-001.md) — Deliver contact and trigger lifecycle events with owned data.
- [TEST-UX-GRAPH-EVENT-002](../../tests/features/TEST-UX-GRAPH-EVENT-002.md) — Report unavailable subscriptions and discard late events.

## Code References

- [events](../../../../../python/ovphysx_companion/events.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-GRAPH-IDENTITY-001](REQ-UX-GRAPH-IDENTITY-001.md).
