<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PREF-COOK-001
maps_to: REQ-UX-PREF-COOK-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Cook copied geometry, invalidate only the companion result cache, retain existing Cooked handles and distinguish runtime disk-cache configuration from unsupported purge.

## Given

Fresh processes with an unset cache path and a writable persistent directory; companion CookingService holds a completed cached result with an outstanding consumer. Place a sentinel file in the persistent directory.

## When

Set the path before bootstrap, cook, change the requested path after bootstrap, invalidate companion results and continue using the outstanding result.

## Then

- Unset selects runtime temporary storage and the explicit path selects persistent storage before bootstrap. The later path is pending restart; old Ujitso switches never reach Carbonite. (REQ AC-1).
- Subsequent companion lookup misses the invalidated cache while the outstanding result remains valid. The sentinel and runtime disk files remain; no releaseObjects or filesystem purge runs. (REQ AC-2).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-PREF-COOK-001](../../requirements/features/REQ-UX-PREF-COOK-001.md).
