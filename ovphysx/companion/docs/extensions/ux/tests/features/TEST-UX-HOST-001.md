<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-HOST-001
maps_to: REQ-UX-HOST-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Reload shared menu/widget contributions, activate explicit preview and verify owner-scoped teardown alongside playback and pure USD authoring.

## Given

A generic Physics menu/property host, two engine providers and stopped customer USD with no physics scene. Capture contribution keys and all native create/attach/step counters.

## When

Register, open all supported panels, refresh selection, disable/re-enable the PhysX provider twice and invoke a generic USD scene-creation command with it disabled.

## Then

- Each engine/feature key has one contribution after reload; unloading PhysX removes only its items and callbacks. (REQ AC-1).
- Panel opening/refresh creates no simulation or solver step. Generic scene creation authors the expected prim without an engine; explicit compute is capability-checked before resources are acquired. (REQ AC-2).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-HOST-001](../../requirements/features/REQ-UX-HOST-001.md).
