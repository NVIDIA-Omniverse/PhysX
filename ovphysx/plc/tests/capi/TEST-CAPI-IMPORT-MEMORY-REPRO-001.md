<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-IMPORT-MEMORY-REPRO-001
maps_to: REQ-CAPI-IMPORT-MEMORY-REPRO-001
type: integration
---

## Scenario

Run the standalone SDK allocation diagnostic on Linux/glibc. Build commands,
package versions, and observed results are recorded in the
[repro README](../../../tests/repros/ovstage_memory_retention/README.md).

## Given

A four-body scene and explicitly selected ovphysx/ovstage SDK packages.

## When

Two fresh processes perform 50,000 pose writes and seals. One also imports each
sealed ordinal into physics. Samples at 10,000 and 50,000 exclude initial settling.

## Then

The runner reports excess allocated growth, retains raw samples, and fails when
that excess exceeds 4 MiB. API failures, missing samples, or process timeouts
produce an invalid-run diagnostic instead of being treated as retention.

This test characterizes a reported defect. A retention failure is the expected
reproduction result, not a passing memory-regression gate. The allowance is
not a portable SDK contract. The standalone project is not registered in the
normal test suite.
