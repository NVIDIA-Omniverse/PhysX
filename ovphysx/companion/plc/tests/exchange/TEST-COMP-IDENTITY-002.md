<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-IDENTITY-002
maps_to: REQ-COMP-IDENTITY-001
type: integration
---

## Scenario

Reject a captured Python source after edits, deletion/recreation, cancellation or use in another session.

This automated scenario covers the Python subset only; it does not claim the
full native/GPU/Kit obligation. See the [requirement scope](../../requirements/exchange/REQ-COMP-IDENTITY-001.md).

## Given

- Two source-scoped EditSessions and captured polygon-mesh records.

## When

- Change topology, delete/recreate an endpoint, invalidate or close its session, or apply through another session.

## Then

- Source/revision/topology mismatches fail before changing the destination layer (REQ AC-3).
- Deleted and foreign-session results never resolve through an implicit active stage (REQ AC-4).

## Executable coverage

[tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py):

- `test_stale_results_fail_without_edits`
- `test_output_rejects_changed_topology_and_instances`

Run the separate customer-USD invocation in the [sample instructions](../../../samples/README.md).
