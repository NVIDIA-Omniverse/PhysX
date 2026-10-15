<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-USD-EDIT-001
maps_to: REQ-COMP-USD-EDIT-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-USD-EDIT-001; executable evidence and qualification limits follow.

## Scenario

Apply a prepared USD edit batch, inject an error partway through, then repeat through a host undo adapter.

## Given

- A stage with a selected edit layer, an unrelated prim, a saved layer-content snapshot and a generated result.
- Failure injection after the first authored property and a minimal host thread/undo wrapper.

## When

- Try stale and read-only targets; apply with the injected failure; apply a valid batch and perform undo/redo in the host wrapper.

## Then

- Only the supplied layer/time destination is eligible for writes and no save occurs (REQ AC-1).
- Invalid inputs leave the serialized layer unchanged (REQ AC-2).
- The injected error is reported with rollback or a documented recoverable patch; partial work is never reported as success (REQ AC-3).
- The standalone path needs no Kit import and host undo/redo restores the exact authored opinions on the approved thread (REQ AC-4).

## Executable coverage

[test_usd_helpers.py](../../../tests/test_usd_helpers.py):

Executed cases cover whole-batch preflight, approved thread/layer, time mapping, false Set and throwing application rollback, recoverable guard failure, stale topology, undo conflict and notice/reentry suppression. tests/test_host_edits.py verifies the same patches in a host command delivery flow.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
