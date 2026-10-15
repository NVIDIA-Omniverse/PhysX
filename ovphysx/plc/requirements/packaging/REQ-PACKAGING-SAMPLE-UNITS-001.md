<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-SAMPLE-UNITS-001
title: Preserve stage units in pipeline sample variants
status: implemented
owner: ovphysx
---

## Description

CPU/GPU sublayer variants of the shipped sample scenes preserve their base
scene's stage units and up axis. USD stage metadata is not inherited from sublayers.

## Acceptance Criteria

- AC-1: **Root metadata.** The four variants below explicitly author
  `metersPerUnit` and `upAxis` equal to their respective base scenes.

## Test References

- [TEST-PACKAGING-SAMPLE-UNITS-001](../../tests/packaging/TEST-PACKAGING-SAMPLE-UNITS-001.md)

## Code References

- `ovphysx/tests/data/boxes_falling_on_groundplane_gpu.usda`
- `ovphysx/tests/data/mixed_base_articulations_gpu.usda`
- `ovphysx/tests/data/empty_dynamic_boxes_cpu.usda`
- `ovphysx/tests/data/simple_physics_scene_cpu.usda`

## Dependencies

- None
