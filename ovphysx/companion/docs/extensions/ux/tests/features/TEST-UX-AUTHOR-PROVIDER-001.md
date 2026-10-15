<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-AUTHOR-PROVIDER-001
maps_to: REQ-UX-AUTHOR-PROVIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Load the companion provider with compatible, missing and mismatched service tables and verify copied-data USD isolation without a legacy fallback.

## Given

Fresh processes with compatible tables, wrong ABI major, undersized table and absent operation variants; thread-count preference 4 is requested before any SDK operation. An unrelated engine provider is registered.

## When

Load the selected companion, perform stopped mass computation, then Play. Retry each incompatible package and request thread count 8 after bootstrap. Unload with a held job.

## Then

- Compatible operations resolve from the same native owner used by playback; no second SDK image loads. Wrong ABI/size fails before resource acquisition; absent operations fail before their resource acquisition. (REQ AC-1).
- One contribution per feature remains registered, and the unrelated provider survives. Bootstrap observes thread count 4; the later request is pending restart. Unload drains the job before module release and no legacy fallback is loaded. (REQ AC-3).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-AUTHOR-PROVIDER-001](../../requirements/features/REQ-UX-AUTHOR-PROVIDER-001.md).
