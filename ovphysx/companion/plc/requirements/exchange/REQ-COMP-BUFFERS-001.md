<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-BUFFERS-001
title: "Owned data buffers and device readiness"
status: implemented
owner: ovphysx
---

## Description

Data services return copied records or explicit leases. Bindings preserve the same allocator, lifetime and device-readiness contract as native clients.

## Acceptance Criteria

- AC-1: **Metadata.** Every array result specifies scalar type, shape/count, stride, device, coordinate frame, units and producing revision/step where applicable.
- AC-2: **Ownership.** Each result has one documented release path or Python-owned copy; retained callback buffers are copied or leased before callback return, and dependent memory stays valid until release.
- AC-3: **Device transfer.** Device results expose completion/stream dependencies; CPU access waits or performs an explicit transfer and never treats device memory as a CPU pointer.
- AC-4: **Python lifetime.** Buffer-protocol or DLPack views retain their native owner until all views are released; releasing one view cannot invalidate another retained view.

## Implementation scope

The current operations return copied CPU records/tuples with documented float32
inputs, int32 correspondence and explicit SI/shared-frame conventions. Python rejects
device arrays before implicit transfer and mass jobs capture immutable scalar data.
Actual service results retain immutable numeric storage or Python-owned copies.
Native cooking/event buffers are copied before release; independent retained views
must survive producer updates and disposal. GPU completion/transfer and DLPack
remain delegated to core read sessions, with explicit CPU-copy opt-in in output
helpers. No generic companion buffer wrapper or GPU result capability is exported.

## Test References

- [TEST-COMP-BUFFERS-002](../../tests/exchange/TEST-COMP-BUFFERS-002.md) - executed copied-CPU subset.

- [TEST-COMP-BUFFERS-001](../../tests/exchange/TEST-COMP-BUFFERS-001.md) - executed CPU view-lifetime scenario; GPU device views remain delegated to core.

## Code References

- [C records](../../../include/ovphysx_companion/companion.h)
- [Python copying and host-data checks](../../../python/ovphysx_companion/native.py)
- [Immutable job capture](../../../python/ovphysx_companion/jobs.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx_types.h](../../../../include/ovphysx/ovphysx_types.h)
- [ovphysx/python/ovphysx/api.py](../../../../python/ovphysx/api.py)
- [ovphysx/python/ovphysx/utils/simulation.py](../../../../python/ovphysx/utils/simulation.py)

- [Explicit core output copies](../../../python/ovphysx_companion/output.py)

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-IDENTITY-001](REQ-COMP-IDENTITY-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)
