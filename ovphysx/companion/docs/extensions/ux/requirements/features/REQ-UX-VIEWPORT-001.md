<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-VIEWPORT-001
title: Keep a single provider-aware Physics display menu
status: proposed
owner: physics-ux
---

## Description

Keep a single provider-aware Physics display menu. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use companion copied visualization results through the selected provider. Python owns menu registration and customer USD; native services neither create a viewport nor access USD.

## Acceptance Criteria

- AC-1: **One category.** The viewport shall show one Physics category and one contribution per stable engine/feature key across open/reload/unload; generic joint visibility shall work without PhysX.
- AC-2: **Current model.** Menu opening shall reflect effective visibility/settings and capability state; None/Selected/All edits shall update the shared model rather than a second independent preference.
- AC-3: **Cleanup.** Menu/provider unload shall remove its listeners and owned windows without removing another engine's items.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-VIEWPORT-001](../../tests/features/TEST-UX-VIEWPORT-001.md) — Share display state across menu and provider reloads.

## Code References

- [host](../../../../../python/ovphysx_companion/host.py) — explicit host scheduling and provider lifetime.
- [debug](../../../../../python/ovphysx_companion/debug.py) — copied visualization substrate; viewport UI remains proposed.


## Dependencies

- [ADR-0002](../../adr/ADR-0002-viewport-menu.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
