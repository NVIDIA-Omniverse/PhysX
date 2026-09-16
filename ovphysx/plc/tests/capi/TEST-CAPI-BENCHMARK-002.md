<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-002
maps_to: REQ-CAPI-BENCHMARK-002
type: integration
---

## Scenario

The benchmark harness exposes every actual timing sample as bounded
diagnostics without changing its alarm-facing report.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`.
- A writable diagnostics path.
- GPU CI builds configured with `OVPHYSX_REQUIRE_BENCHMARK_CUDA=ON`, including
  the `--devphysx` integration flavor.

## When

- `TimingDiagnosticsTest` calculates and serializes known, constant, empty,
  large-value, and escaped-name cases and exercises enabled, disabled, and
  failed writers.
- A 20-step, five-run benchmark invocation writes both its normal report and
  `--timing-diagnostics` sidecar.
- The DirectGPU Cartpole control row runs with its defaults and writes both
  outputs.
- Contract cases use an invalid diagnostics path, resolve differently spelled
  report and diagnostics paths to the same file, skip a device-gated row, and
  fail a selected row.
- Linux and Windows GPU benchmark jobs configure against the staged CUDA
  toolkit before adding ovruntime and build the benchmark executable with its
  parent-owned CUDA runtime target.

## Then

- The 20-by-5 sidecar row reports `all_steps_count=100`; its mean, population
  standard deviation, minimum, and maximum use all 100 actual samples while
  the normal four-field report and trimmed `avg_us` retain their prior format
  and calculation (REQ AC-1, AC-2).
- The exact DirectGPU Cartpole control row also reports
  `all_steps_count=100`, pinning the producer contract consumed by FrameCore
  (REQ AC-1, AC-2).
- The sidecar is valid one-object-per-line JSON with `schema_version=1`, keyed
  by the decorated report command. It excludes the warm-up, includes extrema,
  and reports one sample for the one-shot harness path (REQ AC-1, AC-2).
- An invalid output path or a diagnostics/report path collision makes the
  benchmark process nonzero. Skipped, failed, and unemitted rows add no
  diagnostic object (REQ AC-3).
- Required-CUDA benchmark configuration fails before compilation if the staged
  compiler, toolkit, static runtime target, or parent-to-child target
  propagation is unavailable. Successful normal and `--devphysx` GPU jobs
  report native CUDA tensors enabled and execute the DirectGPU contract, while
  an explicitly optional local build may remain CPU-only (REQ AC-4).
