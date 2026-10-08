<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-002
title: Benchmark All-Sample Timing Diagnostics
status: implemented
owner: ovphysx
---

## Description

The C++ benchmark harness can write an opt-in JSONL sidecar that summarizes
every actual timed-step sample before the harness trims any extrema. The
canonical report, baseline, and comparison value remain unchanged.

## Acceptance Criteria

- AC-1: **Opt-in compatibility.** Without `--timing-diagnostics=<file>`,
  report generation, baseline comparison, and the existing trimmed `avg_us`
  and `std_dev` calculations are unchanged. With the option, each successfully
  emitted benchmark row produces one schema-version-1 JSONL object keyed by
  its exact decorated report name.
- AC-2: **All-sample summary.** Each JSONL object contains
  `all_steps_count`, the arithmetic `all_steps_mean_us`, population
  `all_steps_sd_us`, `all_steps_min_us`, and `all_steps_max_us` over every
  actual recorded `timedStep()` call. The unrecorded warm-up is excluded, the
  fastest and slowest samples are included, and the synthetic triplication
  used by the harness's one-shot compatibility path still counts as one
  actual sample. No raw timing array is written.
- AC-3: **Fail-closed output.** The diagnostics file is truncated for one
  benchmark process. An open, serialization, write, flush, or close failure
  makes that process fail. A row that is skipped, fails, or does not pass
  report emission produces no JSONL object. The diagnostics and report paths
  must resolve to different files, including when their path strings use
  different lexical or symbolic-link aliases.
- AC-4: **Deterministic CUDA configuration.** A benchmark build discovers the
  staged CUDA toolkit in the ovphysx parent CMake scope before adding ovruntime
  and exposes one parent-owned CUDA runtime target to the benchmark directory.
  On Linux it enables the staged CUDA compiler before toolkit discovery. When
  `OVPHYSX_REQUIRE_BENCHMARK_CUDA=ON`, missing compiler, headers, imported
  static runtime, or target propagation fails configuration instead of
  silently producing a CPU-only benchmark executable. An explicitly optional
  local build may still configure the CPU-only suite.

## Test References

- [TEST-CAPI-BENCHMARK-002](../../tests/capi/TEST-CAPI-BENCHMARK-002.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/Harness.h`,
  `ovphysx/tests/benchmarks/Harness.cpp`,
  `ovphysx/tests/benchmarks/TimingDiagnostics.h`, and
  `ovphysx/tests/benchmarks/TimingDiagnostics.cpp`
- AC-2: `ovphysx/tests/benchmarks/Harness.cpp`,
  `ovphysx/tests/benchmarks/TimingDiagnostics.h`, and
  `ovphysx/tests/benchmarks/TimingDiagnostics.cpp`
- AC-3: `ovphysx/tests/benchmarks/Harness.h`,
  `ovphysx/tests/benchmarks/Harness.cpp`, and
  `ovphysx/tests/benchmarks/TimingDiagnostics.cpp`
- AC-4: `ovphysx/CMakeLists.txt`,
  `ovphysx/build.bat`,
  `ovphysx/scripts/build.cmake`, and
  `ovphysx/tests/benchmarks/CMakeLists.txt`
- Contract coverage: `ovphysx/scripts/test_benchmark_contract.cmake`,
  `ovphysx/tests/benchmarks/CMakeLists.txt`,
  `ovphysx/tests/benchmarks/tests/TimingDiagnosticsTest.cpp`,
  `ovphysx/tests/benchmarks/tests/verify_benchmark_diagnostics.py`, and
  `ovphysx/tests/benchmarks/tests/test_verify_benchmark_diagnostics.py`

## Dependencies

- None.
