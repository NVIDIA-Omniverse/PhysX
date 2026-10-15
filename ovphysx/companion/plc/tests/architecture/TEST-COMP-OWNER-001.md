<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-OWNER-001
maps_to: REQ-COMP-OWNER-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-OWNER-001; executable evidence and qualification limits follow.

## Scenario

Close a tools session with work outstanding, then detach and reopen on a new generation.

## Given

- Core instance A with a playback attachment, unattached core peer C, and a borrowed tools session with pending work, subscription and buffer lease.
- A companion-owned native attachment B on another ovstage, plus shared-service and per-owner allocation/callback counters.

## When

- Try A's tools handles through C and B; close the tools session with work pending, test a retained buffer lease, release it and finish cleanup.
- Attempt a second facade attachment through C while A remains attached; independently keep B live through native companion acquisition.
- Retire A before B, then repeat with B retired first. Step/read the survivor, recreate the retired owner and try stale handles; request final shutdown through the coordinating bridge.

## Then

- Borrowed session acquisition performs no attach/step; operations validate the selected owner. (REQ AC-1)
- Close drains tasks/delivery, retained leases prevent premature destruction, and release permits safe completion. (REQ AC-2)
- Foreign/stale access and C's facade attach fail without peer mutation, while compatible native B coexists with A. (REQ AC-3)
- Repeated cycles return owner-local counters to baseline and old-generation callbacks cannot target recreated owners. (REQ AC-4)
- Either survivor remains usable; the process lease retains its services and coordinated final shutdown drains all users before releasing SDK/Carbonite resources. (REQ AC-5)

## Executable coverage

[test_bridge.py](../../../tests_native/test_bridge.py):

Borrowed session lifetime, retryable owner destruction, stale generation and GC are executed. tests_native/test_preview.py covers independent pending owners, duplicate-source rejection and independent public peers, copied options and both destruction orders with recording/process leases. Event and interaction cases cover disposal while peer receipts remain uncollected.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

[Tools session process-exit regression](../../../tests_native/test_bridge.py), `test_tools_session_closes_before_core_at_process_exit`, verifies lease cleanup precedes core destruction (AC-4).
