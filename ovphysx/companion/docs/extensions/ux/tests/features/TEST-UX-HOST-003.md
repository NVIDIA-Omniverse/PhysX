<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-HOST-003
maps_to: REQ-UX-HOST-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Reload shared menu/widget contributions, activate explicit preview and verify owner-scoped teardown alongside playback and pure USD authoring.

## Given

Playback A with one scene, a stopped collision-group editor and information panel, plus independent preview B only after explicit activation.

## When

Edit filters and view statistics, use Play/Pause/Stop A, then activate/close B and inspect available menu actions.

## Then

- A loads only on Play, persists on Pause and is destroyed/restored on Stop. B has a distinct copied source and output destination; it installs no step/reset route into A. (REQ AC-4).
- Filter edits are reversible Python transactions; information counters come from the selected completed owner. Force-load, Fabric-driver switching and private reset actions are absent; opening either panel creates no owner. (REQ AC-5).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-HOST-001](../../requirements/features/REQ-UX-HOST-001.md).
