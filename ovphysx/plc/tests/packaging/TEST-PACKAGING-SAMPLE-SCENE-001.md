<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-SAMPLE-SCENE-001
maps_to: REQ-PACKAGING-SAMPLE-SCENE-001
type: integration
---

## Scenario

`tests/python_tests/utils_tests/test_sample_stage_metadata.py::test_mbp_scene_configuration`
checks the MBP sample's applied scene schema and authored settings (AC-1).

## Given

- The shipped MBP sample.
- OpenUSD in the separate `utils_tests` process, with codeless PhysX schemas
  registered before opening the stage.

## When

- Open the sample and inspect `/World/physicsScene`.

## Then

- The PhysicsScene applies `PhysxSceneAPI` and reads GPU dynamics disabled and
  broadphase type `MBP` from its scene attributes (AC-1).
