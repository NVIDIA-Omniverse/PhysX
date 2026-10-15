<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-API-002
maps_to: REQ-COMP-API-001
type: integration
---

## Scenario

Negotiate mismatched tables, acquire the supported table, and exercise absent/stale handles.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Wrong size/version leaves output unchanged; Python checks the returned layout (REQ AC-1).
- Supported operations use typed results and separate capability bits without exposing internal tables (REQ AC-2, AC-3).
- ABI version 1 is required by the Python wrapper; backend qualification remains partial (REQ AC-4).

## Executable coverage

- [tests_native/test_bridge.py](../../../tests_native/test_bridge.py).
