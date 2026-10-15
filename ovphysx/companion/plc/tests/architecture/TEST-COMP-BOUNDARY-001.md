<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-BOUNDARY-001
maps_to: REQ-COMP-BOUNDARY-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-BOUNDARY-001; executable evidence and qualification limits follow.

## Scenario

Inspect and exercise a core-only installation and the same installation with the companion enabled.

## Given

- A build of the core and companion with compile commands, link maps and packaged dependency lists.
- A tiny rigid-body ovstage scene and two clean Python environments, one with only core installed.

## When

- Run the same simulation smoke in both environments; acquire the companion service in the second and inspect its loaded owner identity.
- Inspect shipped native headers, compile/link dependencies and package payloads.

## Then

- Source/include dependencies point from companion to lower layers only (REQ AC-1).
- Native compilation and the packaged companion closure have no USD/Kit dependency or hidden native USD handle route (REQ AC-2).
- Core and companion observe the same actor and runtime owner; no extra owner image or registry is introduced (REQ AC-3).
- The core-only smoke completes and the primary simulation header has no editor widget/authoring entry points (REQ AC-4).

## Executable coverage

[test_boundary.py](../../../tests_native/test_boundary.py):

Two isolated Python processes simulate/read/detach with the companion non-importable or explicitly enabled; the enabled process observes the same core instance and actor counts. Source/header dependency direction and native ELF closure are checked. Installation separately validates the USD-free packaging lock.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
