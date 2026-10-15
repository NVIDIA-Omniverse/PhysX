<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PACKAGE-001
maps_to: REQ-UX-PACKAGE-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Keep extension bundles and fixture dependencies explicit, including failure, source replacement and exact undo/teardown.

## Given

Fresh host processes with core-only, core-plus-companion, missing codeless schema and incompatible companion ABI package sets; another provider is already registered. The demo catalog supplies per-example qualification rows.

## When

Enable, disable and reload the bundles twice; attempt startup with each missing/incompatible dependency and launch each qualified example in isolation, recording missing assets/capabilities rather than skipping silently.

## Then

- Schema resources are registered before USD registry initialization; ABI incompatibility fails before partial host registration. Exactly one USD/SDK image is loaded. (REQ AC-1).
- Every catalog example has a required-capability/asset disposition and a targeted launch result before qualification; benchmarks and test-only helpers are never product dependencies. (REQ AC-2).
- Reload leaves one owned registration per interface and no abandoned callbacks. Failure cleanup and shutdown preserve the other provider and authored stage data. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-PACKAGE-001](../../requirements/features/REQ-UX-PACKAGE-001.md).
