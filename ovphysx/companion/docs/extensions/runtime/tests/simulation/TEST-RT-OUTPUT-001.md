<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-OUTPUT-001
maps_to: REQ-RT-OUTPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Two admitted substeps produce two steps and two publications. Verifies REQ-RT-OUTPUT-001 AC-1.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One 60 Hz scene, helper backed by the real adapter, spies around both helper invocation and underlying solver calls, and a known initial pose.

## When

- Play, deliver dt=2h with uncapped scheduling, then a zero-due frame, Pause updates and Stop.

## Then

- The frame makes exactly two helper/solver step calls and two distinct output transactions, never four steps. Nonadvancing frames call no stepping helper, including during Stop. (REQ-RT-OUTPUT-001 AC-1).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-OUTPUT-001](../../requirements/simulation/REQ-RT-OUTPUT-001.md).
