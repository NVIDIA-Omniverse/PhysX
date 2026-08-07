# ovphysx benchmarks (C++)

Opt-in performance regression suite for ovphysx.

The harness is a near-verbatim copy of the internal runtime benchmark suite.
The framework files
under `framework/` (`BmBenchmark.h`, `BmTime.cpp/.h`, `BmOutput.cpp/.h`,
`BmUtils.h`) are byte-identical to the source harness so future syncs are
trivial; `BmGlobals.cpp/.h`, `Harness.cpp/.h` and `BenchmarkList.cpp/.h` are
adapted only at their bootstrap point (ovphysx uses `ovphysx::PhysX::create()`
instead of the source harness's app-level bootstrap). The
benchmark cases under `benchmarks/` are ovphysx-specific.

## Build

The suite is **off by default** so it never affects normal `validate_all`
runs. Opt in by passing `--benchmarks` to the build script:

```bash
cd ovphysx
./build.sh --benchmarks                    # add --rebuild for a clean build
cmake -P scripts/install.cmake
```

`--benchmarks` is forwarded to `scripts/build.cmake`, which sets
`-DOVPHYSX_BUILD_BENCHMARKS=ON` at configure time. Equivalent low-level
invocations:

```bash
CMAKE_EXTRA_ARGS="-DOVPHYSX_BUILD_BENCHMARKS=ON" cmake -P scripts/build.cmake
# or
cmake -DBENCHMARKS=ON -P scripts/build.cmake
```

The binary lands at `_build/<platform>/<config>/ovphysx_benchmarks` (Linux)
or `_build\<platform>\<config>\ovphysx_benchmarks.exe` (Windows). It runs
against the installed SDK under `_install/`, matching the c_unittests
pattern.

## Run via the cmake driver (recommended)

```bash
# First run on a new host: bootstrap the baseline file (always needed
# locally; baselines are per-machine and not committed).
BENCHMARK_REGENERATE=1 cmake -P scripts/test_benchmarks_cpp.cmake

# Subsequent runs compare against the locally-bootstrapped baseline.
cmake -P scripts/test_benchmarks_cpp.cmake

# Common knobs:
BENCHMARK_FILTER=Step.*  cmake -P scripts/test_benchmarks_cpp.cmake
BENCHMARK_TOLERANCE=15   cmake -P scripts/test_benchmarks_cpp.cmake
BENCHMARK_GPU=0          cmake -P scripts/test_benchmarks_cpp.cmake  # CPU pass only
```

The driver runs two passes -- GPU first (`--forceGpu`), then CPU -- in
separate processes to exercise both device configurations.

## Run the binary directly

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks --help
./_build/linux-x86_64/release/ovphysx_benchmarks --list
./_build/linux-x86_64/release/ovphysx_benchmarks --filter=Step.* --verbose
```

CLI mirrors the shared harness:

| Flag | Purpose |
|------|---------|
| `--filter=<glob>` | Limit to benchmarks matching the glob pattern (`*`, `?`, `:`-separated alternatives, `-` prefix to exclude). |
| `--list` | Print benchmark names and exit. |
| `--forceGpu` | Run with the PhysX runtime initialized for GPU. |
| `--data=<dir>` | Data folder for the fixtures. |
| `--report=<file>` | Report path. |
| `--regenerate` | Rewrite the baseline (golden) file with current results. |
| `--slop=<pct>` | Tolerance vs baseline, in percent (default 10). |
| `--steps=<n>` / `--runs=<n>` | Override per-benchmark step / run counts. |
| `--threads=<n>` | Plumbed for thread-aware benchmarks. |
| `--detail` | Include per-step times in the report. |
| `--verbose` | Print each benchmark name as it executes. |
| `--hidden` | Also run benchmarks flagged hidden. |

## Benchmark inventory

| Name | Device | Notes |
|------|--------|-------|
| `Smoke.no_op` | any | Sleep-based smoke test for the harness loop. |
| `UsdLoad.basic_simulation` | any | ovstage population + attach/update on a tiny scene. |
| `UsdLoad.articulation_pileup` | any | 16 articulations + 100 obstacles (generated). |
| `UsdLoad.warehouse` | any | ~2.4k-body warehouse fixture (generated). |
| `UsdLoad.cartpole` | any | Single-env cartpole fixture parse cost. |
| `UsdLoad.cubes20` | any | 20 falling cubes + ground plane (overhead-probe fixture). |
| `Step.basic_simulation_cpu` | cpu | 60 timed steps at dt=1/60 with the PhysicsScene authoring 240 timestepsPerSecond. |
| `Step.articulation_pileup_cpu` | cpu | 16 articulations (link counts 3..18) falling onto 100 dynamic obstacles. |
| `Step.warehouse_cpu` | cpu | Steady-state stepping of the ~2.4k-body warehouse. |
| `Step.cubes20_cpu` / `_gpu` | cpu / gpu | Minimal scene per-step (20 cubes). The single-threaded baseline reuses this bench in a separate harness pass with `--threads=1` (see "Single-threaded baseline" below). |
| `Step.two_articulations_gpu` | gpu | GPU-tagged short articulations fixture. |
| `Step.articulation_pileup_gpu` | gpu | Same articulation_pileup fixture, GPU pass. |
| `Step.warehouse_gpu` | gpu | Same warehouse fixture, GPU pass. |
| `Clone.envs_64` / `_256` / `_1024` | gpu | env0 -> envN GridCloner-style replication. |
| `TensorIo.pose_create_1024_cpu` / `_8192_cpu` | cpu | DLPack binding create+spec+destroy cycle on cubes20 cloned to N envs. |
| `TensorIo.pose_read_1024_cpu` / `_8192_cpu` | cpu | DLPack read of pose tensor across N envs (persistent binding). |
| `TensorIo.pose_write_1024_cpu` / `_8192_cpu` | cpu | DLPack write of pose tensor across N envs. |
| `LowLoad.first_step_after_reload` | any | Time to step() once after reloading the scene; not true cold-start. |
| `LowLoad.empty_step` | any | Per-step dispatch cost with no scene loaded — the per-call floor. |
| `LowLoad.noop_ovstage_attach` | any | Per-call cost of ovstage population + attach/update on a 1-prim scene. |
| `LowLoad.reset` | any | Per-call cost of ovphysx_reset on a loaded scene. RL-style episode reset. |
| `Lab.cartpole_<N>_step` | gpu | Per-step throughput at N cartpole envs (N=4096/8192/16384). |
| `Lab.cartpole_<N>_reset` | gpu | Per-call reset cost at N cartpole envs. |
| `Lab.cartpole_<N>_tensor_read` | gpu | DLPack pose read of all N cartpole envs. |
| `Lab.cartpole_<N>_tensor_write` | gpu | DLPack pose write to all N cartpole envs. |
| `Lab.anymal_<N>_step` | gpu | Per-step throughput at N anymal envs (N=1024/8192). Run scripts/fetch_anymal_asset.py once to activate. |
| `Lab.anymal_<N>_reset` | gpu | Per-call reset cost at N anymal envs. |
| `Lab.anymal_<N>_tensor_read` | gpu | DLPack pose read of all N anymal envs. |
| `Lab.anymal_<N>_tensor_write` | gpu | DLPack pose write to all N anymal envs. |
| `Lab.anymal_<N>_clone` | gpu | Per-call clone() cost from /World/envs/template into N targets. |

CPU-only and GPU-only benchmarks gate themselves via `isValid()` checking
`BmGlobals::getInstance().forceGpu()`, so the wrong-device pass skips them
with "failed to initialize, skipping".

GPU tensor-binding I/O is covered by the Python suite
([`tests/python_benchmarks/bench_tensor_io_gpu.py`](../python_benchmarks/bench_tensor_io_gpu.py))
which uses `torch.cuda` for CUDA-resident DLPack tensors. The C++ side is
CPU-tensor only by design.

### Fixture details

- `articulation_pileup.usda` — generated by `data/gen_articulation_pileup.py`;
  16 free-falling articulations (link counts 3, 4, ..., 18) over a scatter
  of 100 dynamic obstacles on an extended ground plane. Adapted from a
  runtime articulation demo with the demo context stripped
  out. PhysicsScene authors 240 Hz.
- `cartpole.usda` — generated by `data/gen_cartpole.py`; single cartpole
  articulation (base + prismatic cart + revolute pole) under
  `/World/envs/env0/cartpole`. Source fixture for the `Lab.cartpole_*`
  benches, cloned into N envs at run-time.
- `anymal/anymal_envs.usda` — committed wrapper around the IsaacLab
  ANYmal-C asset. The binary (`anymal.usd`) + meshes are fetched via
  `scripts/fetch_anymal_asset.py` from NVIDIA's public Isaac Assets S3
  bucket and gitignored. The wrapper mounts the source under
  `/World/envs/template/anymal` so `Lab.anymal_*` benches clone from
  `/World/envs/template` and bind to `/World/envs/env*/anymal/*`
  matching only the clones. Lab.anymal_* benches skip cleanly with a
  `run scripts/fetch_anymal_asset.py` hint if the binary isn't fetched.
- `warehouse.usda` — generated by `data/gen_warehouse.py`; grid of racks
  with 4 static corner posts each and shelves of dynamic pallet items
  (defaults to 2400 dynamic + 401 static).
- `cubes20.usda` / `cubes20_envs.usda` — two variants from
  `data/gen_cubes20.py`. Default is 20 falling cubes + ground.
  `--envs` reshapes the scene to put the cubes under
  `/World/envs/template` so TensorIo benches can clone it into env1..envN.
  Fixed seed (`SEED = 20260603`) for reproducibility.

Both generator scripts live next to the generated USDA so the fixtures are
reproducible:

```bash
python3 tests/benchmarks/data/gen_articulation_pileup.py > \
    tests/benchmarks/data/articulation_pileup.usda
python3 tests/benchmarks/data/gen_warehouse.py > \
    tests/benchmarks/data/warehouse.usda
python3 tests/benchmarks/data/gen_cartpole.py > \
    tests/benchmarks/data/cartpole.usda
python3 tests/benchmarks/data/gen_cubes20.py > \
    tests/benchmarks/data/cubes20.usda
python3 tests/benchmarks/data/gen_cubes20.py --envs > \
    tests/benchmarks/data/cubes20_envs.usda
```

### Single-threaded baseline

The cmake driver runs a third `cpu_st` pass after `gpu` and `cpu` that
invokes the binary with `--threads=1`. The harness wires that through to
the `/physics/numThreads` Carbonite setting BEFORE PhysX bootstrap, so the
dispatcher comes up with one worker. There is no USD attribute for the
per-scene thread count — only the global Carbonite setting takes effect.

The pass is scoped to `Step.cubes20_cpu` via `--filter` so CI cost stays
bounded. Skip it with `BENCHMARK_CPU_ST=0`. Compare against the multi-
threaded `cpu` pass to read the threading delta.

## Adding a new benchmark

1. Add a `.cpp` under `benchmarks/` whose class derives from `BmBenchmark`.
   Override `getNbSteps()`, `getNbRuns()`, `isValid()`, `startRun()`,
   `endRun()`, `preStep()` and `step()` as needed.
2. Register it: `Register<MyClass> sMyClass("Group.name");` at namespace
   scope.
3. Add a `void initMyFile() {}` symbol in the same translation unit and
   declare + call it from `BenchmarkList.cpp::bmInitialize()`. This forces
   the translation unit into the binary so the static registration fires.
4. Add the source to
   `tests/benchmarks/CMakeLists.txt::OVPHYSX_BENCHMARK_SUITE_SRCS`.
5. After it passes locally, regenerate the baseline:
   `BENCHMARK_REGENERATE=1 cmake -P scripts/test_benchmarks_cpp.cmake`.

## Baselines

The shared harness writes its baseline to
`<exe_dir>/../data/benchmarkData/_baseline.txt` (resolves to
`_build/<platform>/data/benchmarkData/_baseline.txt` for ovphysx, which
sits under the gitignored `_build/` tree).

**Baselines are per-machine and are NOT committed.** Wall-clock
performance depends on CPU model, GPU, driver, kernel, and background
load, so a baseline produced on one host is meaningless on another.
Bootstrap the baseline locally with `BENCHMARK_REGENERATE=1` on the
first run, then re-run without it to compare against your own previous
results. CI-host-specific baselines and the broader gating story are
tracked in
[`docs/internal/benchmark_suite_notes.md`](../../docs/internal/benchmark_suite_notes.md).
