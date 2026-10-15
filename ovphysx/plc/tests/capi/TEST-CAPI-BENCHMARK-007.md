<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-007
maps_to: REQ-CAPI-BENCHMARK-007
type: integration
---

## Scenario

CPU movement-command latency is reported separately from simulation latency,
with the existing write-plus-step rows and their execution filters preserved.

## Given

- An installed ovphysx SDK and a successfully compiled `ovphysx_benchmarks`
  target built with `OVPHYSX_BUILD_BENCHMARKS=ON`.
- The six-row write-only inventory and the existing four inventories.

## When

- Inventory unit tests validate all five inventories together and require the
  six new names to be hidden, CPU-only, and outside every existing write family.
- The benchmark contract lists `WriteOnly.*` and executes it without `--hidden`.
- The contract selects all six rows with `--hidden --forceGpu` and verifies
  each decorated name was skipped and published no metric.
- Every contract invocation executes the two 4,096-body rows with
  `--hidden --regenerate --steps=1 --runs=3`. Setting
  `OVPHYSX_BENCHMARK_CONTRACT_WRITE_ONLY=1` expands this to all six rows, preserving
  the developer baseline through the shared transaction.
- The unchanged L1B and high-scale contracts validate their original row sets.

## Then

- The exact hidden list matches the new inventory; the no-hidden control
  publishes nothing; all inventory collision checks pass (AC-1).
- GPU mode selects and skips all six decorated names exactly once and
  publishes no write-only metrics, including zero placeholders (AC-1).
- The positive process exits zero and reports exactly the two selected
  4,096-body timings, or all six when the larger scenes are enabled,
  each after its shared correctness gate read the distinctive velocity back
  from every body. No failed row may publish (AC-2, AC-3).
- Source inspection establishes that only `WriteOnlyBase` overrides
  `timedStep()`, stopping the timer after `writeOnce()` and before
  `stepOnce()`. Existing rows still inherit the default write-plus-step timer
  (AC-2).
- Source inspection verifies that the shared WriteScaling/WriteOnly OVStage
  path checks update admission, waits for and collects its task result before
  stepping or velocity readback, and suppresses the row on failure (AC-2, AC-3).
- Default farm runs use 20 steps and five runs; their command names separate
  write-only data from all existing series (AC-3, AC-4).

## Coverage Limitations

- Only the 8,192/16,384-body positive rows are opt-in because constructing
  the larger scenes is costly. The two 4,096-body rows always exercise the
  new timer and shared correctness gates. Shortened contract counts validate
  behavior, not performance thresholds.
- Threshold calibration and alarm delivery are consumer responsibilities.
