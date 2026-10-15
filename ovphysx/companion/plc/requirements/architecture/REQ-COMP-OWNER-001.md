<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-OWNER-001
title: "Owner-scoped sessions and teardown"
status: implemented
owner: ovphysx
---

## Description

Borrowed tools sessions retain an existing owner and cannot create another simulation. Owned preview/validation sessions explicitly create their own native attachment through the companion bridge. Both session kinds share one loaded runtime/SDK service host while preserving attachment identity and lifetime. Public core peers may also own distinct attachments under the core per-instance ownership contract; no owner can attach the same ovstage twice.

## Acceptance Criteria

- AC-1: **Borrowing.** Opening a tools session does not attach, detach, reset or step the simulation; all live operations validate the supplied instance owner and attachment generation.
- AC-2: **Teardown.** Session close stops admission and drains accepted work/subscriptions; owner destruction returns a retryable lifecycle refusal (core `INVALID_STATE`; companion `Busy` where applicable) while external native leases or zero-copy views still require it, unless their storage has been made independently owned.
- AC-3: **Isolation.** Foreign-session and stale-generation handles fail without mutating another session; independent companion-native and public core attachments may coexist; duplicate live ovstage sources remain rejected.
- AC-4: **Reuse.** Repeated open/close and detach/reattach do not retain callbacks, registrations, buffers or references to the previous attachment.
- AC-5: **Process lease.** Owned companion attachments retain the shared runtime independently of borrowed playback objects. The bridge coordinates final shutdown, global configuration/recording transitions and dependent resource release across core and companion users; destroying either attachment first preserves the other. Existing core-only lifecycle bookkeeping is not sufficient to advertise this capability.

## Implementation scope

Borrowed sessions retain the exact core owner/generation without attaching or
stepping. Destruction is retryable while leases remain. Independent native preview
owners retain process services, coordinate recording/settings/shutdown and drain
only their own pending work. Recreated/foreign references fail explicitly.
Tests cover both retirement orders, retained views/events and duplicate-source refusal and independent public peers.

## Test References

- [TEST-COMP-OWNER-002](../../tests/architecture/TEST-COMP-OWNER-002.md) - executed native/Python subset.

- [TEST-COMP-OWNER-001](../../tests/architecture/TEST-COMP-OWNER-001.md) - executed supported-contract scenario.

## Code References

Companion implementation (native/core changes retain their own PLC ownership):

- [Runtime coordinator](../../../../src/include/internal/sdk/CompanionRuntime.hpp) owns admission, reentry, service leases and typed registries.
- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/src/ovphysx/ovphysx.cpp](../../../../src/ovphysx/ovphysx.cpp)

- [ovphysx/ovruntime/include/omni/physx/IPhysxSimulation.h](../../../../ovruntime/include/omni/physx/IPhysxSimulation.h)
- [ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h](../../../../ovruntime/include/omni/physx/PhysXRuntime.h)

- [Preview.cpp](../../../src/Preview.cpp)
- [test_preview.py](../../../tests_native/test_preview.py)
- [test_events.py](../../../tests_native/test_events.py)

## Dependencies

- [REQ-COMP-API-001](REQ-COMP-API-001.md)
- [Companion ADR-0001](../../adr/ADR-0001-optional-companion-and-owner-bridge.md)
