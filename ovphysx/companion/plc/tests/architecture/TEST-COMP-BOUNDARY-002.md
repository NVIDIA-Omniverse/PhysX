<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-BOUNDARY-002
maps_to: REQ-COMP-BOUNDARY-001
type: integration
---

## Scenario

Use the C bridge and isolated Python/customer-USD consumers against the installed SDK.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Runtime headers have no companion dependency; installed bridge headers are separate from core headers (REQ AC-1).
- SDK closure validation rejects USD payloads; customer-USD sample passes only arrays to its native worker (REQ AC-2).
- The bridge is acquired from the existing library and C tests operate without the companion Python package (REQ AC-3, AC-4).

## Executable coverage

- [tests_native/test_bridge.py](../../../tests_native/test_bridge.py).
- [tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py).
