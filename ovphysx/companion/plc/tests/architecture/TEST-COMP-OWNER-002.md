<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-OWNER-002
maps_to: REQ-COMP-OWNER-001
type: integration
---

## Scenario

Borrow a CPU attachment, reject owner destruction, detach/reattach, close one peer and retain another.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Opening performs no attach/step; statistics use the captured attachment (REQ AC-1).
- Destroy/shutdown reject live leases and Python preserves ownership for retry (REQ AC-2).
- Stale generations and closed handles fail; closing a peer or garbage collection preserves other sessions (REQ AC-3, AC-4).

## Executable coverage

- [tests_native/test_bridge.py](../../../tests_native/test_bridge.py).
