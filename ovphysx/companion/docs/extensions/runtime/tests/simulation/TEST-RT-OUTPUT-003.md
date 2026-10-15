<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-OUTPUT-003
maps_to: REQ-RT-OUTPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Native instancer output preserves holes, tail and quaternion conventions. Verifies REQ-RT-OUTPUT-001 AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One-scene instancer with simulated slot 0, an unsimulated sentinel slot 1 and an authored trailing slot 2; positions have distinct local values, orientations include a nonidentity rotation, and scale/prototype arrays are recorded.

## When

- Publish changed poses for simulated slots through the runtime helper on each supported CPU/CUDA output path.

## Then

- Only simulated native positions/orientations change; local frame and xyzw-to-native quaternion conversion are correct (float16 quaternion tolerance 2e-3). Holes, trailing slots, scale and prototype data remain unchanged. (REQ-RT-OUTPUT-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-OUTPUT-001](../../requirements/simulation/REQ-RT-OUTPUT-001.md).
