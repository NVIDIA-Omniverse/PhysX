<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-007
title: CPU Movement Command Delivery Without Simulation Timing
status: implemented
owner: ovphysx
---

## Description

Six hidden `WriteOnly.velocity_{ovstage,tensor}_{4096,8192,16384}_cpu`
rows measure the time to send movement commands to physics. Each command sets
the speed and direction of every box. The timer includes command completion
but excludes advancing the simulation. The existing `WriteScaling` and
`WriteScalingHighN` rows continue to include both operations.

## Acceptance Criteria

- AC-1: **Separate CPU inventory.** `write_only_inventory.json` names exactly
  six hidden CPU rows, two paths at each of 4,096, 8,192 and 16,384 bodies.
  They cannot match any existing L1B or high-scale family filter, and the
  combined inventories have no duplicate or substring collisions, including
  harness-decorated report names. Normal execution without `--hidden` and
  execution under `--forceGpu` publish no write-only rows.
- AC-2: **Explicit timer boundary.** The OVStage timer includes query creation,
  the linear-velocity write and its completion wait, query release and wait,
  the velocity-scoped floor and wait, and the update into physics. The tensor
  timer includes the persistent velocity-binding write and `waitAll()`.
  The following `step(1/60 s)` and completion wait execute outside the timer.
  The existing write-plus-step rows retain their original timer and operation
  sequence. No phase subtraction produces the write-only measurement.
- AC-3: **Shared fixture and correctness.** The generated scene, CPU dynamics
  and MBP checks, setup, warmup, value fill, and teardown reuse
  `WriteScalingBase`. Defaults remain 20 steps over five runs with the
  existing middle-three-per-step statistic. Outside timing, a distinctive
  velocity is written and read back from every body after stepping. Any setup,
  write, wait, drain, step, readback or teardown error fails the process and
  suppresses the failed row.
- AC-4: **Interpretation and execution ownership.** Results are absolute
  latencies in microseconds, lower is better. OVStage writes linear velocity;
  tensors write linear and zeroed angular velocity, so the comparison describes
  two paths to the same validated linear motion rather than identical bytes.
  Setup, simulation, transport and application scheduling are excluded.
  Persistent-query or all-scope-seal benchmarks have different boundaries.
  The positive contract always exercises both 4,096-body paths; the larger
  rows are opt-in via `OVPHYSX_BENCHMARK_CONTRACT_WRITE_ONLY=1`.
  Regular collection belongs to
  FrameCore/fpp's separate `write_only_hidden` suite. Consumer thresholds must
  be calibrated from these rows on the matching build and hardware. Alarm
  activation is a separate consumer change; ratios are descriptive only.

## Test References

- [TEST-CAPI-BENCHMARK-007](../../tests/capi/TEST-CAPI-BENCHMARK-007.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/tests/write_only_inventory.json`,
  `ovphysx/tests/benchmarks/tests/test_verify_inventory.py`,
  `ovphysx/scripts/test_benchmark_contract.cmake`
- AC-2, AC-3, AC-4: `ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp`

## Dependencies

- [REQ-CAPI-BENCHMARK-001](REQ-CAPI-BENCHMARK-001.md)
- [REQ-CAPI-BENCHMARK-005](REQ-CAPI-BENCHMARK-005.md)
