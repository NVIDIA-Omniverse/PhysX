<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-IMPORT-MEMORY-REPRO-001
title: Standalone OVStage Import Memory Diagnostic
status: implemented
owner: ovphysx
---

## Description

Provide an opt-in reproducer that compares allocation growth during repeated
public `ovphysx_update_from_ovstage` calls with identical writes and seals
without physics imports. This is a diagnostic requirement, not a new SDK memory
budget or a claim that retention is fixed.

## Acceptance Criteria

- AC-1: Use a fixed four-body scene, public SDK APIs, and CPU-only physics.
  Perform no simulation steps or output reads during the measured loop.
- AC-2: Run import and write-only cases in separate processes. Record live
  allocator bytes, mappings, RSS, and teardown checkpoints.
- AC-3: Compare growth from iteration 10,000 to 50,000 and report a failing test
  when import growth exceeds control growth by more than 4 MiB. Distinguish API
  failures and timeouts from a reproduced retention failure.
- AC-4: Build independently against explicitly selected released SDK packages.
  Do not add this known-failing diagnostic to normal SDK validation.

## Test References

- [TEST-CAPI-IMPORT-MEMORY-REPRO-001](../../tests/capi/TEST-CAPI-IMPORT-MEMORY-REPRO-001.md)

## Code References

- ovphysx/tests/repros/ovstage_memory_retention/repro.cpp
- ovphysx/tests/repros/ovstage_memory_retention/test_memory_retention.py
- ovphysx/tests/repros/ovstage_memory_retention/run_matrix.py
- ovphysx/tests/repros/ovstage_memory_retention/CMakeLists.txt
