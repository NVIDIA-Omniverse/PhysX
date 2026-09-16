<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-USD-KINEMATIC-SUPPORT-001
title: Kinematic Support Geometry Control
status: implemented
owner: physics-runtime
---

## Description

Applications shall drive translating kinematic support geometry through
ovstage transform updates so PhysX applies kinematic targets rather than
teleporting actors. Public guidance shall distinguish support translation from
surface velocity and explain when both mechanisms are combined.

## Acceptance Criteria

- AC-1: A world-transform update published through ovstage and drained before
  stepping moves a kinematic rigid body through the PhysX kinematic-target path,
  preserving its authored scale and the step velocity needed to carry dynamic
  bodies through contact without reconstructing the PhysX actor.
- AC-2: Public documentation distinguishes translating kinematic supports from
  stationary `PhysxSurfaceVelocityAPI` conveyors and documents their additive
  combination, friction, sleeping, ordering, reset/readback, the local
  `omni:xform` and resolved `omni:fabric:worldMatrix` contract, descendant world
  updates, and DirectGPU limitations.
- AC-3: Public C and Python samples demonstrate a translating support, a
  stationary surface-velocity conveyor, and their additive combination by
  measuring dynamic rider motion.

## Test References

- TEST-USD-KINEMATIC-SUPPORT-001

## Code References

- ovphysx/tests/python_samples/kinematic_support.py
- ovphysx/tests/c_samples/kinematic_support_c/main.c
- ovphysx/tests/data/kinematic_support.usda
- ovphysx/docs/simulation_setup/kinematic_support.md

## Dependencies

- None
