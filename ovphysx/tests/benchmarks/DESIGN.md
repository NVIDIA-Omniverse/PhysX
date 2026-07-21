# ovphysx benchmark suite — design

This document captures the design of our C++ benchmark suite under
`tests/benchmarks/`: what it does, the principles behind it, and how it
fits with the rest of the project. Audience: SDK / physics team. For how
to *use* the suite, see [`README.md`](README.md).

A parallel Python suite exists under `tests/python_benchmarks/` driven by
`pytest-benchmark`; this document focuses on the C++ side and gives the
Python suite a one-paragraph mention at the end.

## Context

ovphysx had no performance-regression suite before this work. The existing
`tests/c_unittests/` and `tests/python_tests/` validate *correctness*;
nothing was watching the *performance* side.

The harness is imported, near-verbatim, from the internal runtime benchmark
suite. That code in turn descends from the upstream PhysX SDK benchmark
harness. Using it unchanged (rather than building a parallel one) means:

- Bug fixes and improvements to the shared harness merge into ovphysx
  cleanly.
- Anyone who already knows the shared benchmark code can read this one
  without re-learning.
- The benchmark *cases* (the things we actually want to measure for
  ovphysx) are the only ovphysx-specific code; everything around them is
  shared.

## What's unmodified vs adapted

| File | Status | Reason |
|---|---|---|
| `framework/BmBenchmark.h` | **byte-identical** to shared runtime harness | pure virtual base, depends only on `BmTime.h` |
| `framework/BmTime.cpp/.h` | byte-identical | std + Linux/Windows time APIs |
| `framework/BmUtils.h` | byte-identical | printf / stricmp wrappers |
| `framework/BmOutput.cpp/.h` | byte-identical | uses `BmGlobals::getInstance().getFileSystem()` which we provide |
| `framework/UsdPCH.h` | ovphysx-only stub | upstream includes a real PCH; we provide a minimal include set so the unmodified `.cpp` files compile |
| `framework/BmGlobals.cpp/.h` | adapted | upstream bootstraps via app-level Carbonite startup. ovphysx bootstraps via `ovphysx::PhysX::create()` which encapsulates Carbonite startup; everything else (registry, glob filter, accumulator) is preserved verbatim |
| `Harness.cpp/.h` | adapted | the run loop / CLI / statistics are verbatim. We use `CARB_STATIC_BINARY_GLOBALS(...)` in place of upstream's `OMNI_APP_GLOBALS(...)` because ovphysx benchmarks use the static Carbonite SDK bootstrap, not omni-app |
| `BenchmarkList.cpp/.h` | adapted | same shape; the `extern void init...()` set lists ovphysx's tests, not the source harness tests |
| `benchmarks/*.cpp` | ovphysx-specific | the cases below |

The benchmark-side tests are deliberately ours: the source benchmark cases
touch internal runtime and raw PhysX scene objects that the ovphysx wrapper
hides. Our tests call only
`ovphysx::PhysX` and `ovphysx_*` C ABI.

## Architecture at run time

```
main()                                            (Harness.cpp)
  └─ harnessImpl()
       ├─ parse CLI                                (Harness.cpp)
       ├─ bmInitialize(...)                        (BenchmarkList.cpp)
       │    └─ bmCreateGlobals(...)                (BmGlobals.cpp)
       │         └─ ovphysx::PhysX::create(...)    (the SDK boot)
       ├─ bmGetRegister(filter,…)                  (BmGlobals.cpp)
       ├─ for each benchmark:
       │    ├─ unrecorded warmup pass
       │    ├─ R runs × S steps, timing each step
       │    ├─ sort, drop fastest 20% / slowest 30%,
       │    │   take trimmed mean and stddev
       │    └─ BmRecord goes into the results array
       ├─ BmOutput::emit(...)                      (BmOutput.cpp)
       │    – writes report and compares against baseline
       └─ bmTerminate() → bmDestroyGlobals()
```

The harness binary is launched twice by `scripts/test_benchmarks_cpp.cmake`
— once with `--forceGpu`, once without — to cover both CPU and GPU benchmarks
in separate processes. CPU-only and GPU-only
benchmarks gate themselves via `BmBenchmark::isValid()` checking
`BmGlobals::getInstance().forceGpu()`, so the wrong-device pass skips
them with "failed to initialize, skipping".

## Benchmark cases

| Name | Device | What it measures |
|---|---|---|
| `Smoke.no_op` | any | Harness liveness — 1 ms sleep × 10 steps × 5 runs. |
| `UsdLoad.basic_simulation` | any | ovstage population + initial attach cost for a tiny scene. |
| `UsdLoad.articulation_pileup` | any | ovstage population of 16 articulations + 100 obstacles. |
| `UsdLoad.warehouse` | any | ovstage population of ~2.4k dynamic + ~600 static (warehouse fixture). |
| `UsdLoad.cartpole` | any | Single-env cartpole fixture parse cost. |
| `UsdLoad.cubes20` | any | 20-cube overhead-probe fixture parse cost. |
| `Step.basic_simulation_cpu` | cpu | 60 timed steps at outer dt=1/60 against a PhysicsScene authoring 240 timestepsPerSecond. |
| `Step.articulation_pileup_cpu` | cpu | 16 floating articulations fall onto a dense scatter of obstacles. |
| `Step.warehouse_cpu` | cpu | Steady-state stepping of the ~2.4k-body warehouse. |
| `Step.cubes20_{cpu,gpu}` | cpu/gpu | Minimal scene per-step (60-step avg). Re-run with `--threads=1` in the cpu_st pass for the single-threaded baseline. |
| `Step.two_articulations_gpu` | gpu | Short anchored articulations, GPU-authored sceneDesc. |
| `Step.articulation_pileup_gpu` | gpu | Same fixture as the CPU variant, GPU pass. |
| `Step.warehouse_gpu` | gpu | Same warehouse fixture, GPU pass. |
| `Clone.envs_64` / `_256` / `_1024` | gpu | GridCloner-style `clone()` of env0 into N targets. |
| `TensorIo.pose_{create,read,write}_{1024,8192}_cpu` | cpu | DLPack binding I/O on cubes20 cloned to N envs. |
| `LowLoad.first_step_after_reload` | any | Time to step() once after reloading the scene; not true cold-start. |
| `LowLoad.empty_step` | any | Per-step dispatch overhead with no scene loaded. |
| `LowLoad.noop_ovstage_attach` | any | Per-call ovstage population + initial attach cost on a 1-prim USDA. |
| `LowLoad.reset` | any | Per-call `ovphysx_reset` cost on a loaded scene. |
| `Lab.cartpole_<N>_{step,reset,tensor_read,tensor_write}` | gpu | IsaacLab-style cartpole replication: 4 ops × 3 sizes (4096/8192/16384). |
| `Lab.anymal_<N>_{step,reset,tensor_read,tensor_write,clone}` | gpu | IsaacLab-style Anymal replication: 5 ops × 2 sizes (1024/8192). Skip until asset published. |

### Why these particular fixtures

Fixtures live in `tests/benchmarks/data/`, generated by per-fixture
`gen_*.py` scripts for reproducibility. Articulation pileup gives a real
signal under the articulation solver; cubes20 acts as a deliberately
minimal overhead-probe; warehouse covers larger-scene startup. All
benchmark fixtures author `physxScene:timeStepsPerSecond = 240` so
step-timing comparisons across fixtures are consistent.

## Trim and statistics

The harness runs `R` runs × `S` steps, transposes to `[S][R]`, sorts each
step's timings across runs, and computes a trimmed mean — drop the
fastest 20% and the slowest 30% (at minimum 1 of each), average what's
left. Standard deviation is computed over the same trimmed set. This
matches what the shared harness does and is what the baseline file
records.

## CLI

| Flag | Meaning |
|---|---|
| `--filter=<glob>` | glob-pattern filter (`*`, `?`, `:` for OR, `-` prefix to exclude) |
| `--list` | print and exit |
| `--forceGpu` | bring up the runtime as GPU |
| `--data=<dir>` | data folder for fixtures |
| `--report=<file>` | report path |
| `--regenerate` | rewrite baseline with current results |
| `--slop=<pct>` | baseline tolerance (default 10) |
| `--steps=<N>` / `--runs=<N>` | override per-benchmark counts |
| `--detail` | per-step CSV dump |
| `--verbose` | name-of-current-benchmark logging |
| `--hidden` | also include benchmarks flagged hidden |

`--filter=` syntax matches the shared harness and is intentionally
not reduced.

## Build & CI integration

- Opt-in via `OVPHYSX_BUILD_BENCHMARKS=ON`. `./build.sh --benchmarks`
  forwards it.
- Driver script `scripts/test_benchmarks_cpp.cmake` runs the binary
  twice (GPU then CPU) and writes per-pass logs under
  `_build/benchmark_results/`.
- ctest label `benchmarks` is excluded from `validate_all` via
  `-LE benchmarks`, so timing-sensitive jobs never run alongside
  correctness tests.

## Python suite

`tests/python_benchmarks/` is pytest-based, driven by `pytest-benchmark`.
Scoped per review on MR !7247 to signals the C++ harness genuinely can't
measure: DLPack ↔ numpy/torch.cuda roundtrip cost (`bench_tensor_io_*.py`)
and true process-restart cold start via subprocess
(`bench_process_cold_start.py`). The earlier Python step/usd_load/clone/lab
benches were removed — they ran the same workloads as the C++ side with
sub-µs Python wrapper overhead on top, which wasn't worth the CI wall time.

## Baselines

Baselines are **per-machine and not committed**: wall-clock performance
depends on CPU, GPU, driver, kernel and background load. The runtime
writes its `_baseline.txt` to
`_build/<platform>/data/benchmarkData/_baseline.txt`, which is under
the gitignored `_build/` tree. Bootstrap once locally with
`BENCHMARK_REGENERATE=1` and then compare against your own prior runs.
A CI-host-specific baseline strategy is sketched as item 4 in
[`docs/internal/benchmark_suite_notes.md`](../../docs/internal/benchmark_suite_notes.md).

## Open items

See [`docs/internal/benchmark_suite_notes.md`](../../docs/internal/benchmark_suite_notes.md)
for the tracked follow-ups (per-CI-host baselines, hardware-counter
metrics, multi-prefix scaling, etc.).
