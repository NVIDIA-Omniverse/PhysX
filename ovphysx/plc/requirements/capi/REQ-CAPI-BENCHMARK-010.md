<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-010
title: Equivalent-Workload Comparison of the Session API and Tensor Bindings
status: implemented
owner: ovphysx
---

## Description

Benchmark rows that compare the session read and write API with the deprecated tensor binding on the same work,
with the same data placement: IsaacLab's Humanoid control step and masked reset over 8192 environments, and
device-sourced write rows facing the binding write rows. Every row is hidden and runs on DirectGPU only.

## Acceptance Criteria

- AC-1: **Device-sourced writes.** `OutputWrite.writedevice_{rb,arti_root,arti_dof}_8192_gpu` write the attribute
  of the `writeonly` row of the same objects over a held query, with the payload uploaded to device memory once per
  run. Each fill is one device-to-device copy over the group's column, and each timed write waits for its commits'
  device work, which the binding's write blocks for. During warm-up, a group whose tensors are not one column of the
  payload's size fails the row (REQ-CAPI-BENCHMARK-006 AC-4).
- AC-2: **Control step.** `Lab.humanoid_8192_control_step_{tensorbinding,session,session_captured}_gpu` run the same
  sequence over 8192 Humanoids: a device copy of one of two action buffers into the policy's output buffer; twice, a
  force write from that buffer followed by a 1/120 s step, the session filling its actuation-force column from it
  with one device copy; then reads of the root position, orientation and linear and angular velocity and of the
  joint position and velocity. The action gives every DOF of an environment the same force, its
  sign alternating between environments and samples, so both APIs drive the same motion. A sample covers the whole
  control step, physics included, and ends with the device idle. Each row prints its write, step and read time per
  sample. The captured row replays the fill and commit, and the reads, from CUDA graphs captured once, the fill
  captured while its group is live; the write session and the reads stay open, and the policy and the steps run
  outside the graphs.
- AC-3: **Masked reset.** `Lab.humanoid_8192_reset_{tensorbinding,session}_gpu` write the root pose and velocity and
  the joint position and velocity back to the initial state, held on the device, for the environments whose index
  is congruent to the sample's phase modulo 16: the binding with masked writes, the session with row-masked commits.
  Each row builds its masks at setup from its own rows' prim paths. One untimed step separates samples.
- AC-4: **Validation and failure visibility.** Setup validates each Humanoid row before anything is timed: the
  scene holds 8192 floating-base Humanoids of 16 bodies and 21 DOFs. Every value a control step row reads is finite, its
  joints move between two control steps, and the joint force PhysX applies, read through the session over the joint
  query whichever API the row writes with, is the last action for every DOF. A reset row moves every environment's
  root position and velocities and joint positions and velocities, resets phase 0, and reads all of them back: the
  environments phase 0 selects hold the initial state and the others the moved one. Orientations are written but
  not moved, so a failed orientation write would go unseen. A failed setup, validation or API call, or an error
  OVPhysX logs, throws, so no row publishes and the process fails.

## Test References

- [TEST-CAPI-BENCHMARK-010](../../tests/capi/TEST-CAPI-BENCHMARK-010.md): DirectGPU host runs of every row and
  source inspection.

## Code References

- ovphysx/tests/benchmarks/benchmarks/LabHumanoid.cpp
- ovphysx/tests/benchmarks/benchmarks/OutputWrite.cpp
- ovphysx/tests/benchmarks/data/humanoid_envs_gpu.usda
- ovphysx/tests/benchmarks/data/humanoid.usda

## Dependencies

- REQ-CAPI-BENCHMARK-006: the OutputWrite rows the device-sourced rows extend, and their failure handling.
