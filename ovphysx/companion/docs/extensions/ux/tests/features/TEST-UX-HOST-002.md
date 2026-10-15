<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-HOST-002
maps_to: REQ-UX-HOST-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Reload shared menu/widget contributions, activate explicit preview and verify owner-scoped teardown alongside playback and pure USD authoring.

## Given

A held async result for selected prim P in stage S1, a current provider generation and a new stage S2 with another selection. Include a held callback from the old provider.

## When

Replace selection, stage and provider independently before delivering each held result; dispatch subsequent input from the current viewport.

## Then

- Old-generation results cannot change the current model, author USD or capture new input. Listener/helper counts return to baseline for each retired owner. (REQ AC-3).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-HOST-001](../../requirements/features/REQ-UX-HOST-001.md).
