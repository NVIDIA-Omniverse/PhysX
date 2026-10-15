<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-API-001
maps_to: REQ-COMP-API-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-API-001; executable evidence and qualification limits follow.

## Scenario

Probe acquisition with matching and mismatching client descriptors and backend capabilities.

## Given

- A companion-enabled owner, an absent optional service, and client tables surrounded by sentinel bytes.
- A manifest specifying supported ABI and backend combinations.

## When

- Request supported, unknown and incompatible versions and short/oversized tables; repeat against unsupported backend configurations and an incompatible package.

## Then

- Invalid descriptors leave sentinel bytes intact and publish no callable table (REQ AC-1).
- Each failure is the documented typed category, distinguishable without parsing logs (REQ AC-2).
- The acquired public surface contains no unrestricted internal-interface acquisition escape hatch (REQ AC-3).
- Advertised capabilities match successful probes, and mismatched packages fail before allocation or callback registration (REQ AC-4).

## Executable coverage

[test_compatibility.py](../../../tests_native/test_compatibility.py):

Twenty cases verify all 19 package-declared table versions and size boundaries with surrounding canaries, plus distinct absent/incompatible discovery before resource acquisition. Service-specific mode/lifecycle tests exercise the operation matrix in docs/compatibility.md.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
