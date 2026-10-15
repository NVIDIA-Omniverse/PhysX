<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-AUTHOR-PROVIDER-002
maps_to: REQ-UX-AUTHOR-PROVIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Load the companion provider with compatible, missing and mismatched service tables and verify copied-data USD isolation without a legacy fallback.

## Given

A customer-USD host with a triangle mesh and edit layer, plus a worker configured with an incompatible population USD image. Capture native-call arguments, process module lists and thread identities.

## When

Compute an attachment proposal from copied mesh data, populate the isolated preview worker, return copied poses and commit on the host thread. Kill the worker during a second request and reload the companion provider.

## Then

- Native calls contain arrays, values and revision IDs, never a USD object. The worker protocol contains no SDK/USD pointer; edits occur only in the host customer-USD layer. The incompatible image is absent from the host process. (REQ AC-2).
- Worker failure edits nothing and loads no replacement legacy provider. Owned requests/handles retire before reload; registration and SDK image counts return to the expected single-owner baseline. (REQ AC-3).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-AUTHOR-PROVIDER-001](../../requirements/features/REQ-UX-AUTHOR-PROVIDER-001.md).
