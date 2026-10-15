<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-SAMPLE-UNITS-001
maps_to: REQ-PACKAGING-SAMPLE-UNITS-001
type: integration
---

## Scenario

`tests/python_tests/utils_tests/test_sample_stage_metadata.py` checks stage
metadata on each CPU/GPU sublayer variant (AC-1).

## Given

- The four sample variants listed in the requirement and their base scenes.
- OpenUSD in the separate `utils_tests` Python process, with codeless PhysX
  schemas registered before opening any stage.

## When

- Open each variant and its base as separate USD stages.

## Then

- Each variant has authored `metersPerUnit` and `upAxis`, and both match
  its base stage (AC-1).
