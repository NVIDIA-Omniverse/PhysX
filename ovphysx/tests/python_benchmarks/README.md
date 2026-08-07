# ovphysx benchmarks (Python)

Opt-in `pytest-benchmark` suite scoped to signals the C++ harness can't
measure. Per review (aborovicka): running the full step / clone /
lab grid through Python on top of the C++ suite was duplicate machine time —
the wrapper overhead on top of native compute is sub-µs and didn't add signal.

The suite now keeps only the Python-unique benches:

- **`bench_tensor_io_cpu.py`** — DLPack ↔ numpy roundtrip cost across cubes20
  cloned to N envs. The C++ side measures the C ABI; this measures what RL
  users actually pay through the wrapper.
- **`bench_tensor_io_gpu.py`** — DLPack ↔ `torch.cuda` roundtrip cost (CUDA-
  resident tensors). Requires the `[gpu]` extra.
- **`bench_process_cold_start.py`** — true process-restart cold start via
  subprocess. The C++ harness shares one PhysX instance across runs and
  cannot measure this from within itself; `LowLoad.first_step_after_reload`
  is the closest in-process signal but isn't the same thing.

Step / USD-load / clone / Lab benches were removed: those workloads are
covered by the C++ suite (`Step.*`, `UsdLoad.*`, `Clone.envs_*`, `Lab.*`)
and the Python wrapper layer is thin enough that a regression in it would
show up as a tensor-I/O regression here anyway.

## Run via the cmake driver

```bash
cd ovphysx
cmake -P scripts/install.cmake
cmake -P scripts/test_benchmarks_python.cmake
```

The driver runs two passes (CPU then GPU) in separate processes, passing
`--bench-device=cpu|gpu` to pytest for each pass; tests use the
`bench_device` fixture to skip when the active device doesn't match.

Common environment overrides:

```bash
BENCHMARK_REGENERATE=1 cmake -P scripts/test_benchmarks_python.cmake   # save baseline
BENCHMARK_FILTER=tensor cmake -P scripts/test_benchmarks_python.cmake  # pytest -k filter
BENCHMARK_GPU=0        cmake -P scripts/test_benchmarks_python.cmake   # CPU pass only
```

JSON reports land at `_build/benchmark_results/python/{cpu,gpu}.json`. The
default compare mode fails the run when any benchmark's mean regresses by
more than 10% vs the most recent saved baseline.

## Run pytest directly

```bash
cd tests/python_benchmarks
uv sync
uv run pytest --benchmark-only --bench-device=cpu -v
uv run pytest --benchmark-only --bench-device=gpu -v
```

GPU tensor I/O requires the optional `[gpu]` extra:

```bash
uv pip install -e .[gpu]
```

## Adding a benchmark

Only add Python benchmarks for signals that genuinely can't be measured on
the C++ side. Use the `benchmark` fixture and call it once with a no-arg
callable:

```python
def test_my_thing(benchmark, physx, data_dir, bench_device):
    cpu_only(bench_device)             # or gpu_only(bench_device)
    attach_usd_with_ovstage(physx, data_dir / "fixture.usda", "my-benchmark")
    physx.wait_all()

    def step():
        physx.step(1.0/60.0)
        physx.wait_all()

    benchmark(step)
```

`pytest-benchmark` handles trimmed mean, std-dev, and golden comparison.
