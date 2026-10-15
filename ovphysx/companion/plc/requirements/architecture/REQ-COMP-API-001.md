<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-API-001
title: "Versioned companion discovery and capability negotiation"
status: implemented
owner: ovphysx
---

## Description

A dedicated acquisition entry point returns size/versioned data-oriented service tables from the existing owner. Conceptual service names in these records are not final exported symbols.

## Acceptance Criteria

- AC-1: **Negotiation.** Acquisition validates requested ABI version and table size before writing output; incompatible requests return a typed error and no usable table.
- AC-2: **Availability.** Discovery distinguishes service absent, operation unsupported for this backend, invalid lifecycle state and execution failure; a declared internal interface alone never advertises support.
- AC-3: **Narrow tables.** Public tables contain selected operations, opaque owned handles and data layouts; they do not expose arbitrary internal IPhysx tables or parser descriptors through void pointers. The explicitly included, build-matched custom C++ interfaces are a separate typed SDK surface governed by REQ-COMP-NATIVE-ABI-001; they are not exposed by the general C/Python tables.
- AC-4: **Compatibility.** Each package declares compatible core/companion ABI versions and per-operation CPU, GPU and DirectGPU capabilities; loading an incompatible pair fails before acquiring resources.

## Implementation scope

All 19 Python-bound C service tables negotiate ABI 1 and exact size before resource
acquisition; native C++ SDK access additionally matches the build/compiler ABI.
The package declaration and operation matrix distinguish absent, unsupported,
invalid lifecycle and execution failures. Canary tests cover unknown ABI and both
short/oversized tables. Narrow dedicated headers keep editor services outside the
primary simulation header.

## Test References

- [TEST-COMP-API-002](../../tests/architecture/TEST-COMP-API-002.md) - executed native/Python subset.

- [TEST-COMP-API-001](../../tests/architecture/TEST-COMP-API-001.md) - executed supported-contract scenario.

## Code References

Companion implementation (native/core changes retain their own PLC ownership):

- [include/ovphysx_companion/companion.h](../../../include/ovphysx_companion/companion.h)
- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/include/ovphysx/ovphysx_types.h](../../../../include/ovphysx/ovphysx_types.h)
- [ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h](../../../../ovruntime/include/omni/physx/PhysXRuntime.h)

- [compatibility.py](../../../python/ovphysx_companion/compatibility.py)
- [compatibility.md](../../../docs/compatibility.md)
- [test_compatibility.py](../../../tests_native/test_compatibility.py)

## Dependencies

- [REQ-COMP-BOUNDARY-001](REQ-COMP-BOUNDARY-001.md)
- [Companion ADR-0001](../../adr/ADR-0001-optional-companion-and-owner-bridge.md)
