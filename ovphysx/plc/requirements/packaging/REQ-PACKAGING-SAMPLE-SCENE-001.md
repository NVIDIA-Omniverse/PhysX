<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-SAMPLE-SCENE-001
title: MBP sample scene configuration
status: implemented
owner: ovphysx
---

## Description

The shipped MBP sample declares the schema required to consume its CPU dynamics
and MBP broadphase settings.

## Acceptance Criteria

- AC-1: **Scene settings.** `/World/physicsScene` in
  `boxes_falling_on_groundplane_mbp.usda` applies `PhysxSceneAPI`, with
  `physxScene:enableGPUDynamics = false` and `physxScene:broadphaseType = "MBP"`.

## Test References

- [TEST-PACKAGING-SAMPLE-SCENE-001](../../tests/packaging/TEST-PACKAGING-SAMPLE-SCENE-001.md)

## Code References

- `ovphysx/tests/data/boxes_falling_on_groundplane_mbp.usda`

## Dependencies

- None
