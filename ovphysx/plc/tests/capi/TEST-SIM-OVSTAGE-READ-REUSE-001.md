<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-SIM-OVSTAGE-READ-REUSE-001
maps_to: REQ-SIM-OVSTAGE-READ-REUSE-001
type: integration
---

## Scenario

Consume fetched OVStage tensor rows without repeating the read, while retaining
correct row identity, missing values, and update freshness.

## Given

- Native source-bucket fixtures with real OVStage instances and synthetic CPU
  groups whose prim map differs from the data map.
- Explicit compact tensor strides, a wider transported tensor, repeated mapped
  rows, a nonzero byte offset, and a presence mask spanning two 64-bit words.
- A GPU physics stage with five distinguishable bodies and known velocities.

## When

- Getters read every seeded row, then read a replacement bucket.
- A map references a row outside the transported tensor.
- Two successive drains author distinct velocities to differently ordered
  subsets of the bodies.

## Then

- Valid mapped and masked bucket reads perform zero live attribute reads and
  return the expected values or known absence (AC-2, AC-3).
- Invalid mapped rows take the live-read fallback without decoding out of
  bounds (AC-4).
- GPU velocity readback matches each object's last authored value; untouched
  bodies retain their previous values (AC-1, AC-5).

- Compact-layout unit tests accept explicit strides and empty tensors, and
  reject malformed, overflowing, and non-compact tensors (AC-6).
- The existing particle-points drain test verifies that explicit-stride
  array values reach the GPU backend (AC-6).

## Automation

- Native target `ovstage_source_bucket_unittests`; CTest `ovstage-source-bucket`.
- `tests/internal_unittests/test_ovstage_source_bucket.cpp`.
- `tests/python_tests/test_ovstage_drain.py`, especially
  `test_drain_rigid_velocity_reordered_subsets_remain_fresh`.
- The 4k DELMIA benchmark measures drain time separately; timing thresholds are
  intentionally excluded from correctness tests.
