<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-SAMPLE-001
maps_to: REQ-PYTHON-SAMPLE-001
type: integration
---

## Scenario

The shipped Python tensor-bindings sample displays and validates motion from the
fixed-base articulation chain.

## Given

- `links_chain_sample.usda`, whose link 0 is fixed to the world and whose
  remaining links form a driven revolute chain.
- The sample's alternating articulation velocity targets and periodic link-pose
  readback loop.
- The Python sample runner, which treats a nonzero sample exit as failure.

## When

- The sample writes its velocity targets, runs 1000 simulation steps, and reads
  the selected link pose at each output checkpoint.

## Then

- The output identifies the non-root link selected for display (REQ AC-1).
- Every displayed position and quaternion component is finite (REQ AC-2).
- At least two displayed positions differ, and their maximum absolute position
  delta exceeds `1.0e-3` before success is reported (REQ AC-2).

## Development-Time Negative Control

- In a disposable copy, selecting fixed link 0 instead makes the movement guard
  report its zero delta and the sample exit nonzero. This mutation check is not
  part of the committed automated test run.
