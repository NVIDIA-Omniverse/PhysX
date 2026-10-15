<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEBUG-002
maps_to: REQ-COMP-DEBUG-001
type: integration
---

## Scenario

Read real scene counters through a borrowed session.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Known scene body counts are copied with the selected owner identity; subsequent reads leave earlier records independent (REQ AC-2).

## Executable coverage

- [tests_native/test_bridge.py](../../../tests_native/test_bridge.py).
