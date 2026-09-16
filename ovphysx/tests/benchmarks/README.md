<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx benchmarks (C++)

Opt-in performance regression suite for ovphysx.

The harness is a near-verbatim copy of the internal runtime benchmark suite.
The framework files
under `framework/` (`BmBenchmark.h`, `BmTime.cpp/.h`, `BmOutput.cpp/.h`,
`BmUtils.h`) are byte-identical to the source harness so future syncs are
trivial; `BmGlobals.cpp/.h`, `Harness.cpp/.h` and `BenchmarkList.cpp/.h` are
adapted for OVPhysX bootstrap, registration, and fail-closed exception handling.
The benchmark cases under `benchmarks/` are ovphysx-specific.

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
BENCHMARK_HIDDEN=1       cmake -P scripts/test_benchmarks_cpp.cmake
BENCHMARK_DIRECTGPU=1    cmake -P scripts/test_benchmarks_cpp.cmake  # adds --directGpu to the GPU pass
```

The driver normally runs GPU (`--forceGpu`), CPU, and single-threaded CPU
passes in separate benchmark processes. `BENCHMARK_HIDDEN=1` appends
`--hidden` to every enabled pass. `BENCHMARK_EXPECT_ROWS=N` requires exactly
`N` positive rows in each pass report and is accepted only when exactly one
pass is enabled; selecting no passes is also an error. Disable the other two
passes before using it, for example:

```bash
BENCHMARK_GPU=0 BENCHMARK_CPU_ST=0 BENCHMARK_HIDDEN=1 \
BENCHMARK_FILTER='Authoring.*_cpu:WriteScaling.*_cpu' \
BENCHMARK_EXPECT_ROWS=12 BENCHMARK_REGENERATE=1 \
cmake -P scripts/test_benchmarks_cpp.cmake
```

**Warning: filtered `BENCHMARK_REGENERATE=1` truncates your baseline.** Each enabled pass's
`BmOutput` destructor rewrites the *entire* `_baseline.txt` from scratch with only the rows that
pass ran (`--filter` narrows what ran). Regenerating with a `BENCHMARK_FILTER` set therefore
replaces your full local baseline with just the filtered rows, silently dropping every other row
to `No baseline` on your next unfiltered comparison run. Only regenerate with a filter set when you
mean to narrow the baseline permanently; otherwise snapshot `_baseline.txt` first (or wrap the call
with `scripts/benchmark_baseline_txn.cmake`, the same crash-recoverable snapshot/restore protocol
`scripts/test_benchmark_contract.cmake` and the `measure-benchmark-change` skill use) and restore it
after. The unfiltered bootstrap command at the top of this section is safe -- it always covers every
row -- and `--directGpu` can be requested for the GPU pass with `BENCHMARK_DIRECTGPU=1`, needed for
`Probe.*` and `OutputRead.*_gpu` rows.

## Run the binary directly

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks --help
./_build/linux-x86_64/release/ovphysx_benchmarks --list
./_build/linux-x86_64/release/ovphysx_benchmarks --filter=Step.* --verbose
```

The Cartpole control-step probe requires DirectGPU so its DLPack tensors stay
on the same CUDA device as the simulation. Run it in a dedicated process and
keep the filter; DirectGPU changes scene behavior for other GPU benchmarks.
It is hidden from default wildcard runs, so its dedicated FrameCore task and
any direct invocation must select it explicitly and include `--hidden`. Alarm
policy is owned outside this producer.

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks \
    --forceGpu \
    --directGpu \
    --hidden \
    --filter=Probe.cartpole_4096_control_step \
    --data=tests/data \
    --report=_build/cartpole_probe.txt \
    --verbose
```

`Probe.cartpole_4096_tensor_binding_create` uses the same fixture and flags.
With its exact filter in a fresh process, it measures creation of the effort,
position-target, velocity-target, joint-position, and joint-velocity bindings
and queries each specification. Load, clone, warm-up, caller CUDA data-buffer
allocation, tensor I/O, physics steps, validation, and destruction are outside
the timer. Internal work and allocation performed by `createTensorBinding()`
remain inside. A second requested measurement fails.

`--timing-diagnostics=<file>` optionally writes one JSONL object for each
successfully reported row. The object summarizes every actual timed call as
count, arithmetic mean, population standard deviation, minimum, and maximum.
For the default Cartpole control row this means all 100 timings, including the
fastest and slowest; its ordinary report still uses the existing trimmed
calculation. The one-shot binding-creation row reports a count of one. No raw
timing array is written. The diagnostics path and `--report` must resolve to
different files; alternate spellings of the same file are rejected.

The `OutputRead.*_gpu` rows require DirectGPU for the same reason: both the output
read and the tensor binding branch on `PxSceneFlag::eENABLE_DIRECT_GPU_API`, which
is raised only when `suppressReadback` is set, so a plain `--forceGpu` pass would
measure their host paths instead. They are hidden from default wildcard runs and
refuse to run without both flags:

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks \
    --forceGpu \
    --directGpu \
    --hidden \
    --filter=OutputRead.*_gpu \
    --data=tests/data \
    --verbose
```

The `ContactReport.*` pair does the opposite: its GPU row requires conventional
GPU dynamics and rejects DirectGPU. A contact is one place where two collision
shapes touch. After each physics step, a contact report is the list of touching
pairs plus the point, normal, separation, and impulse data generated for those
touches. This fixture intentionally expects no friction anchors. Run the CPU
and GPU rows in separate fresh processes:

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks \
    --hidden \
    --filter=ContactReport.persistent_pairs_512_step_read_cpu \
    --report=_build/contact_report_cpu.txt \
    --verbose

./_build/linux-x86_64/release/ovphysx_benchmarks \
    --forceGpu \
    --hidden \
    --filter=ContactReport.persistent_pairs_512_step_read_gpu \
    --report=_build/contact_report_gpu.txt \
    --verbose
```

Do not add `--directGpu` to the second command. That mode disables the host
simulation-state readback path used by the public raw getter, so the row fails
instead of publishing a misleading zero or fallback measurement.

`OutputRead` rows fail closed like the rest of the suite. A read that does not complete returns
early and therefore times FAST, so a failed setup, a failed read, or a drain that stops resolving
rows is recorded against the exact registered row: the harness discards that timing and the process
exits non-zero. `scripts/test_benchmark_contract.cmake` gates both halves: step 10 requires a CPU row to publish
and exit cleanly, and step 10b requires a row pointed at a missing data directory to publish nothing
and exit non-zero. Step 10c does the same clean-exit check on a DirectGPU row and is **opt-in** —
it runs only when `OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU=1`, which the benchmark CI job exports, so
it gates in CI but is skipped by a plain local run even on a GPU host.


The `InstancingAttach.*` rows capture NVBug 6532970 by isolating initial ovphysx
attach scaling as unrelated render-only scene-graph prototypes grow. They are
hidden, CPU-only diagnostics; the first version records the current behavior
without a wall-clock gate or automatic production schedule.

Trunk already carries the first scoping fix for that bug (`16f12f0cc6`), so
these rows measure the residual cost, not the originally filed cost. The
`*_flat_*` control rows populate the same prims with `instanceable = false`. If
a flat row costs the same as its instanced sibling, the residual is not
specific to scene-graph instancing and reworking instancing will not remove it.
Run the family directly with the exact filter:

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks \
    --hidden \
    --filter=InstancingAttach.* \
    --data=tests/data \
    --report=_build/instancing_attach.txt \
    --verbose
```

CLI mirrors the shared harness:

| Flag | Purpose |
|------|---------|
| `--filter=<glob>` | Limit to benchmarks matching the glob pattern (`*`, `?`, `:`-separated alternatives, `-` prefix to exclude). |
| `--list` | Print benchmark names and exit. |
| `--forceGpu` | Run with the PhysX runtime initialized for GPU. |
| `--directGpu` | Use OVPhysX DirectGPU settings; requires `--forceGpu`. |
| `--data=<dir>` | Data folder for the fixtures. |
| `--report=<file>` | Report path. |
| `--timing-diagnostics=<file>` | Write all-sample count, mean, population standard deviation, minimum, and maximum as JSONL without changing the normal report; it must resolve to a different file from `--report`. |
| `--regenerate` | Rewrite the baseline (golden) file with current results. |
| `--slop=<pct>` | Set the tolerances written during baseline regeneration (default 10); it does not override an existing baseline during comparison. |
| `--steps=<n>` / `--runs=<n>` | Override per-benchmark step / run counts. |
| `--threads=<n>` | Plumbed for thread-aware benchmarks. |
| `--detail` | Include per-step times in the report. |
| `--verbose` | Print each benchmark name as it executes. |
| `--hidden` | Also run benchmarks flagged hidden. |
| `--nvtx` | Emit NVTX ranges so the run can be captured with Nsight Systems (equivalent to `OVPHYSX_NVTX=1`). Refer to [NVTX Profiling](../../docs/tutorials/nvtx_profiling.md). |

## Benchmark inventory

The row-by-row inventory (name, device, hidden status, and what each row actually measures) lives
in a generated catalogue instead of a hand-maintained table here, checked against its sources on
every regeneration. Generate it directly:

```bash
python scripts/get_benchmark_summary.py    # writes _build/generated/benchmark-summaries/ovphysx/BENCHMARK_SUMMARY.md
```

It groups every registered row by subsystem category and tags, and opens with a "Heavy / do not
loop" section for every row hidden by default (`Register<T, true>` / `--hidden`). Categories,
descriptions, and per-row notes are authored in the source-controlled side-car
`scripts/benchmark_summary_analysis.json`; add or refresh an entry there directly, in the same
shape as its existing entries.

**Teardown:** `bmTerminate()` previously faulted after the report file was written and closed.
`~BmGlobals()` cleared `g_carbFramework` -- this binary's own module-local Carbonite pointer, not a
borrowed one -- before `delete mPhysX` and `ovphysx_shutdown()`, both of which log through it. It is
now cleared last, and only when this object acquired it. `bmDestroyGlobals()` is idempotent and
nulls the shared instance. Step 10 of `scripts/test_benchmark_contract.cmake` requires a clean exit
from a CPU `OutputRead` row, so a teardown regression fails the job rather than going unnoticed.
Step 10c adds the DirectGPU row under the same contract, opt-in behind
`OVPHYSX_BENCHMARK_CONTRACT_DIRECTGPU=1` — exported by the benchmark CI job, skipped by a plain
local run.

CPU-only and GPU-only benchmarks gate themselves via `isValid()` checking
`BmGlobals::getInstance().forceGpu()`, so the wrong-device pass skips them
with "failed to initialize, skipping".

General GPU tensor-binding I/O is covered by the Python suite
([`tests/python_benchmarks/bench_tensor_io_gpu.py`](../python_benchmarks/bench_tensor_io_gpu.py))
which uses `torch.cuda` for CUDA-resident DLPack tensors. The C++ Cartpole
probe also uses native CUDA DLPack tensors, while the generic C++ `TensorIo`
rows remain CPU-tensor measurements.

The control probe fixes both OVPhysX and its native CUDA buffers to visible
device 0, uses IsaacLab's DirectGPU writeback suppression, and defaults to
eight PhysX worker threads. It deliberately freezes stable bulk backend
traffic rather than reproducing every current adapter call. In particular,
indexed/subset writes are outside this v1 row and require separate coverage.
The binding-creation row creates no data buffers. OVPhysX reports a binding's
native no-staging DLPack device through
`ovphysx_get_tensor_binding_native_device()`. OMPE-103958 can still track any
measured CPU/CUDA transfers when benchmark storage differs from that reported
native device.

### Fixture details

- `articulation_pileup.usda` — generated by `data/gen_articulation_pileup.py`;
  16 free-falling articulations (link counts 3, 4, ..., 18) over a scatter
  of 100 dynamic obstacles on an extended ground plane. Adapted from a
  runtime articulation demo with the demo context stripped
  out. PhysicsScene authors 240 Hz.
- `particles_envs_128.usda` — generated by
  `data/gen_particles.py`; N envs each holding one particle set and its own
  particle system. **Authored rather than cloned**, for the same reason the
  deformable assets below are: `ovphysx_clone` drives `IPhysxReplicator`, whose
  object switch mentions neither `ePTParticleSet` nor `ePTParticleSystem`, so
  cloning a subtree holding a particle set succeeds and yields N envs with no
  particles in them, silently. Each `env_<i>` internally references the `class`
  template, and USD retargets `physxParticle:particleSystem` into the referencing
  subtree — which is what gives every env its own system rather than collapsing
  them onto one shared one.
- `deformables_envs_128.usda` — generated by
  `data/gen_deformables.py`; N envs each holding one volume deformable, one
  surface deformable and their own materials. **Authored rather than cloned,
  and that is not a style choice:** `ovphysx_clone` drives `IPhysxReplicator`,
  whose object switch falls through `default: break` for
  `PxDeformableVolume` / `PxDeformableSurface`, so cloning a subtree holding a
  deformable succeeds and produces N envs with no deformable in them, silently.
  Every other `OutputRead` family scales by cloning one template; this one
  cannot. The template is a `class`, so it composes where referenced and is
  skipped by the default traversal predicate rather than simulated as an
  N+1th env — unlike the cloning families, where the template IS in the scene
  and N clones cover N+1 objects. One material per env, because the
  per-material read wants M == N rather than one shared material.
- `cartpole_probe.usda` — generated by `data/gen_cartpole_probe.py`; synthetic Y-up
  Cartpole source under `/World/envs/env_0/Robot`. The probe retains the
  IsaacLab joint names and drive values, then clones env 0 into 4095 targets.
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
- `InstancingAttach.*` builds its deterministic USDA string in the benchmark
  object before timing. Every case has exactly five non-instanceable leaf Mesh
  prims carrying `PhysicsRigidBodyAPI` and `PhysicsCollisionAPI`. The only
  variable is 100, 250, or 500 distinct render-only prototype/instance pairs;
  paired `physics` and `all` rows expose whether irrelevant rendering data
  affects the attach operation. The counts match the table filed on NVBug
  6532970 so the rows can be compared against it directly. The `*_flat_*`
  control rows emit the same fixture with the `instanceable` flag as the only
  difference, which isolates instancing-specific cost from cost that only
  scales with populated prim count. The benchmark verifies that all five bodies were
  realized after every timed attach.

The generator scripts live next to the generated USDA so the fixtures are
reproducible:

```bash
python3 tests/benchmarks/data/gen_articulation_pileup.py > \
    tests/benchmarks/data/articulation_pileup.usda
python3 tests/benchmarks/data/gen_warehouse.py > \
    tests/benchmarks/data/warehouse.usda
python3 tests/benchmarks/data/gen_cartpole_probe.py > \
    tests/benchmarks/data/cartpole_probe.usda
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


### L1B Authoring and WriteScaling rows

The `Authoring.*` rows are ported from the ovphysx `c_samples` in
`NVIDIA-Omniverse/physx-internal` PRs #80, #101, and #104. The
`WriteScaling.*` pair preserves the 4,096-body case from
`NVIDIA-dev/omniverse-physics` PR #2. All eighteen registrations are hidden;
use `--hidden --filter=Authoring.*:WriteScaling.*` to select them.

These are **synthetic in-process producer-path proxies**, not an end-to-end
customer reproduction. Their boundaries begin at public population, OVStage,
or tensor calls and include the associated physics step. They exclude every
container and gRPC boundary, serialization, transport, client scheduling, and
application loop. Do not present them as complete customer latency.

Only two lower-is-better absolute-latency rows are primary KPIs:

- `Authoring.population_add_drip_cpu`
- `WriteScaling.velocity_ovstage_4096_cpu`

The other nine CPU rows are diagnostics or comparators. In particular,
`population_churn` uses population-reference add/remove plus attribute deletion
and never calls `OVSTAGE_PRIM_MODE_UPSERT`; its name and boundary describe
churn, not an upsert lifecycle. Ratios, deltas, and slopes are descriptive only
and are not KPIs.

For every row, the harness sorts the `R` timings independently at each measured
step index. It takes the mean over the inclusive zero-based index range
`[max(1, floor(0.2*R)), max(min(floor(0.7*R), R-2), 1)]`, then reports the mean
of those per-step trimmed means. Both primary rows use `R=5`, so indices 1
through 3 are retained:

- `Authoring.population_add_drip_cpu` has one measured step. Its result is the
  middle-three mean of five complete 80-cycle timings.
- `WriteScaling.velocity_ovstage_4096_cpu` has 20 measured step indices. Each
  index first becomes the middle-three mean across five runs, and the reported
  value is the mean of those 20 values.

The report's `std_dev` is not run-to-run benchmark variance. For each step
index it is the population standard deviation of the retained samples, and the
reported value is the mean of those per-step deviations within that one
invocation. Characterize cross-invocation variance from separate accepted
runs.

All scene construction, population and attachment, `ovphysx_warmup()`,
same-stage warmup, and correctness readback stay outside the measured windows.
`ovphysx_warmup()` is the current API name; it initializes PhysX lazy
structures and disables per-step Fabric synchronization overhead. Teleport and
drain additionally preserve their source samples' five and ten untimed
same-stage iterations.

Every row fails closed. Setup, operation, wait, release, drain, step, readback,
or teardown errors are recorded against the exact row; the harness discards
that timing and exits nonzero. Untimed positive gates require the expected body
count for the two growth paths, return-to-floor plus exact-path create/remove
for churn, the authored pose or velocity for teleport/drain, and the written
velocity on every one of the 4,096 WriteScaling bodies. A fast no-op therefore
cannot publish as a successful sample.

#### Device evidence and fixture truth

L1B is CPU-canonical. The twelve CPU rows are the dedicated contract; the six
`Authoring.*_gpu` rows are hidden, unscheduled diagnostics. They are not
baselined or compared with CPU. `forceGpu()` selects the requested row and the
scene authors the requested dynamics policy, but neither is proof of the
realized execution device. A callback scoped across requested-GPU attachment
and warm-up detects known CPU-fallback and GPU contact-capacity warnings. Either
warning fails the requested-GPU Authoring row; silence in that window is only
negative fallback and capacity evidence. This changes the pass/fail gate for
all six hidden GPU diagnostics, not their frozen names or the CPU KPI contract.
ovphysx exposes no first-class realized-device query, so these rows make no
stronger GPU claim.

The Authoring rows depend on four shipped fixtures under `ovphysx/tests/data/`:

| Fixture | Rows | Dynamics and timestep truth |
|---|---|---|
| `empty_dynamic_boxes.usda` | requested-GPU population add, runtime-write spawn, and population churn | Plain PhysicsScene request; no `physxScene:timeStepsPerSecond` authoring, so the schema default is 60 Hz. |
| `empty_dynamic_boxes_cpu.usda` | CPU population add, runtime-write spawn, and population churn | Sublayers the plain fixture and explicitly authors CPU dynamics plus MBP; timestep remains unauthored and defaults to 60 Hz. |
| `simple_physics_scene.usda` | requested-GPU teleport | Plain PhysicsScene request; timestep is unauthored and defaults to 60 Hz. |
| `simple_physics_scene_cpu.usda` | CPU teleport and the three drain rows | Sublayers the plain fixture and explicitly authors CPU dynamics plus MBP; timestep remains unauthored and defaults to 60 Hz. |

The directory is copied into the SDK (`_install/samples/data/`) and wheel
(`ovphysx/samples/data/`). WriteScaling instead builds its 4,096-body stage in
memory, verifies CPU dynamics and MBP on its PhysicsScene, authors
`physxScene:timeStepsPerSecond = 240`, and still advances the outer benchmark
step by 1/60 second.

#### Comparability boundary

Compare a WriteScaling row only with the other WriteScaling row. Its preserved
prototype boundary waits for the OVStage write, releases the query, waits for
the scoped floor, updates physics, and uses `step()` plus `waitAll()`.
Authoring's OVStage lanes use the later single-seal-wait shape (the seal covers
the write transitively) and `ovphysx_step_sync()`, which bypasses asynchronous
operation machinery. The resulting numbers differ for reasons other than body
count or API choice and are not comparable across the two families.

Within the WriteScaling pair, the OVStage lane creates and releases a path-list
query on every measured step and waits separately for the write, query release,
and scoped floor before stepping. The tensor lane reuses one persistent
`[N,6]` binding and has only its write-completion wait before the common
asynchronous step/wait. The observed gap therefore compares the two end-to-end
control paths, including addressing, query lifetime, and synchronization; it
is not a pure write-cost difference. Keep both rows on the same plot only for
that end-to-end path comparison, never to attribute the delta to the write
operation alone. The tensor lane writes linear and zeroed angular velocity,
while the OVStage lane writes linear `physics:velocity`; their post-timing gate
proves the same linear-velocity outcome, not identical full state.

### High-scale diagnostics rows

Five hidden CPU-only diagnostics extend the picture above without touching the
frozen L1B contract. They live in their own inventory
(`tests/high_scale_inventory.json`) and under family prefixes the L1B selection
cannot match, so `--hidden --filter=Authoring.*:WriteScaling.*` still returns
exactly the eighteen L1B rows. Select them with:

```bash
./_build/linux-x86_64/release/ovphysx_benchmarks \
    --hidden \
    --filter='WriteScalingHighN.*_cpu:RuntimeSpawnScaling.*_cpu' \
    --data=tests/data \
    --report=_build/high_scale.txt \
    --verbose
```

| Row | What it measures |
|---|---|
| `WriteScalingHighN.velocity_ovstage_8192_cpu` | the `WriteScaling` OVStage velocity sequence at 8,192 bodies |
| `WriteScalingHighN.velocity_tensor_8192_cpu` | the `WriteScaling` tensor velocity sequence at 8,192 bodies |
| `WriteScalingHighN.velocity_ovstage_16384_cpu` | the `WriteScaling` OVStage velocity sequence at 16,384 bodies |
| `WriteScalingHighN.velocity_tensor_16384_cpu` | the `WriteScaling` tensor velocity sequence at 16,384 bodies |
| `RuntimeSpawnScaling.collider_heavy_1280_cpu` | seal, structural drain, and first step of one runtime-spawned body into 1,280 existing static colliders |

The four `WriteScalingHighN.*` rows are `WriteScaling.cpp`'s fixture with only
the body count changed: same generated CPU scene, same lane writes, same 20
measured step indices over five runs, same untimed gate that reads the written
velocity back from every body. They exist because the 4,096-body L1B point sits
before the observed OVStage slowdown becomes obvious. Read them against each
other and against the 4,096-body pair as a scaling picture; the comparability
boundary above applies unchanged.

`RuntimeSpawnScaling.collider_heavy_1280_cpu` reuses the
`Authoring.runtime_write_spawn` column writes to author one body into a
generated scene that already holds 1,280 static Cube colliders. Everything up to
and including the thirteen waited UPSERT writes, plus one waited `omni:xform`
write that carries the same pose, is outside the timer; the timed sample is the
seal (`ovstage_advance_write_floor` plus its wait), the
`ovphysx_update_from_ovstage` drain, and one `ovphysx_step_sync(1/60 s)`. The
`omni:xform` write exists because ovphysx resolves a created prim's local
transform from that data-plane matrix, not from the `xformOp:*` columns, so the
spawn columns alone realize the body at the identity pose; the existing
`Authoring.runtime_write_spawn_cpu` row is left as it was. The untimed gate
then requires exactly one dynamic rigid body, that `ovphysx_get_object_type()`
classifies `/World/Runtime/Body_0000` as a rigid body, and that its pose reads
back at the authored height, so a dropped or deferred drain cannot publish as a
fast ingestion. The collider count is verified in the generated text (the
public API has no realized-static-shape query), the scene authors zero gravity
and the CPU dynamics/MBP lines, and the timestep is the unauthored 60 Hz
default like the Authoring fixtures.

All five rows are diagnostics: no primary-KPI status, no GPU variant, and any
consumer threshold is an absolute lower-is-better non-paging fence. The
contract's high-scale section (`scripts/test_benchmark_contract.cmake`) proves
the collision-free inventory union, the exact hidden list, and vacuity without
`--hidden` on every run, and adds one positive five-row run at
`--steps=1 --runs=3` when `OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1` is set.
That run is opt-in because populating and attaching a 16,384-body scene takes
minutes per run on the current runtime, which does not fit a per-MR job; run
it locally before changing these rows. Nothing in this repository's CI
executes the five rows (the per-MR benchmark job keeps the L1B filter and
does not opt in). Their owner is the FrameCore `ovphysx` project's
`high_scale_hidden` suite, defined outside this repository in the
FrameCore/fpp repository (`pipelines/ovphysx`, `OVPHYSX_SUITE=high_scale_hidden`)
and run on a recurring schedule on the L40S/EPYC 7313P cohort; that run
produces and validates the default 20-by-5 statistic.

### ContactReport persistent-pair rows

The two hidden ContactReport rows are a separate L1 contract, not additions to
the eighteen-row Authoring/WriteScaling inventory. Both are
lower-is-better absolute latencies. The CPU row is reported under its raw name;
the GPU harness appends `_GPU`, producing
`ContactReport.persistent_pairs_512_step_read_gpu_GPU`.

The generated scene has 512 isolated cells. Each cell contains one awake
dynamic reporting sphere overlapping one static sphere. Cell spacing prevents
cross-pair contacts; zero gravity, disabled sleeping, and disabled contact
solving keep every overlap present without solver motion. The reporter applies
`PhysxContactReportAPI` with threshold zero. CPU explicitly uses CPU dynamics
and MBP. GPU explicitly uses GPU dynamics and GPU broadphase. Its three
authored capacities are pinned to the exact schema defaults: 524288 rigid
contacts, 81920 rigid patches, and 262144 found/lost pairs.

Each timer starts immediately before
`ovphysx_step_sync(instance, 1.0f / 60.0f)` and stops immediately after the
following `ovphysx_get_contact_report()` call returns.

Generation, population, attachment, expected-path construction, warm-up, and
semantic validation are outside the timer. Warm-up is bounded at 32 steps and
ends only after three consecutive structurally valid, all-persistent reports
contain exactly 512 headers, 512 points, and zero friction anchors. Each row
then uses 20 measured step indices and five runs.

The getter returns borrowed buffers. Validation therefore finishes before the
next simulation step can invalidate them. It requires exactly 512 persistent
pair headers and 512 points, the current attach handle, the exact authored
Reporter/Static identity set, a complete non-overlapping point partition, zero
global and per-header friction anchors, and finite point positions, normals,
impulses, and separations with sane normal length and separation range. Any
mismatch or API error suppresses the row and makes the process fail.

This fixture deliberately covers one contact point per pair/header, zero
friction anchors from its `solveContact=false` shape, and one default-material
case. It does not cover multi-point manifolds, friction-anchor report payloads,
or material diversity.

`--forceGpu` selects the requested GPU row, but ovphysx has no public query for
the realized simulation device. Known CPU-fallback and contact-capacity warning
messages fail the row; silence is negative fallback evidence, not positive GPU
proof. DirectGPU is outside this contract. The rows also exclude scene attach,
contact-report parameter parsing, contact solving, Contact Binding, transport,
and application-loop cost.

The ordinary report remains the KPI source. Qualification noise uses its
emitted `std_dev / avg` ratio. The optional all-sample diagnostics sidecar is
used only to prove that the default 20-by-5 run executed 100 timed calls.

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
6. **If it is an `OutputRead.*` row, add it to
   `tests/outputread_inventory.json` with an `owner`.** Step 10d of
   `scripts/test_benchmark_contract.cmake` compares the registered rows against
   that file and rejects a row with no owner, so the family cannot grow
   anonymously — it reached 157 registrations once. Record the `hidden` flag
   too: the CPU rows are visible and the DirectGPU rows are hidden, and a GPU
   row that stops being hidden runs on every default pass and fails on a host
   with no CUDA device.

### Who owns which rows

Four inventories, with different jobs:

| File | Covers | Contract |
|------|--------|----------|
| `tests/producer_inventory.json` | the frozen L1B `Authoring.*` / `WriteScaling.*` rows | exact row set, all hidden; steps 1–9 |
| `tests/outputread_inventory.json` | the `OutputRead.*` matrix (15 CPU visible, 19 DirectGPU hidden) | exact row set, per-row `hidden`, and an `owner` per row; steps 10–10d |
| `tests/contact_report_inventory.json` | the two hidden `ContactReport.*` rows | exact row set, CPU plus conventional GPU; the ContactReport section |
| `tests/high_scale_inventory.json` | the five hidden `WriteScalingHighN.*` / `RuntimeSpawnScaling.*` CPU rows | exact row set, all hidden, collision-free union with the other three inventories (also against harness-decorated record names); the high-scale section |

`OutputRead` is a performance family rather than a frozen numeric contract. Its
rows are gated on behaviour (clean exit, a published metric, fail-closed on
missing data) and on ownership — not on a regression threshold. The per-row
tolerance the harness seeds from `--slop` is compared on non-regenerate runs;
CI's benchmark job runs the 15 CPU rows with `--regenerate`, so it publishes
them into the `benchmark_results` artifact rather than failing on them. Gating
those rows on regression needs a committed baseline, which does not exist yet.

**So this matrix is an INVENTORY, not a performance gate.** Be precise about what
that means before citing it as evidence:

- The 15 CPU rows are regenerated and published every run. Nothing compares them
  against a reference, so a regression changes the published number and fails
  nothing.
- The 19 DirectGPU rows are `hidden`, so they do not run in CI's default pass at
  all. **No GPU number is produced or compared in CI**; GPU coverage there is the
  clean-exit behaviour check, not a measurement.
- A row existing therefore says a path was exercised once and has an owner. It
  does not say the path is fast, that it got faster, or that it will not regress.

Consequently the matrix is **not evidence for any particular optimization claim**.
Where a claim needs evidence, it needs a named consumer, a threshold and a
committed baseline — none of which exist yet. In particular the point-instancer
FP64 polar-decomposition orientation path has **no row here**, so this matrix
says nothing about it.

Curating the matrix — which pairs answer the CPU-vs-GPU migration decision, which
unpaired diagnostics belong in an opt-in suite, and which duplicate sizes earn
their runtime — is an owner decision, not a documentation one. `owner` is
recorded per row in `outputread_inventory.json` for exactly that conversation.

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

The benchmark contract driver protects this file when its narrow checks use
`--regenerate`. If that driver is interrupted, rerun
`cmake -P scripts/test_benchmark_contract.cmake`; its first step restores the
saved baseline before any benchmark process starts. Do not delete its
`_baseline.txn` or `_baseline.developer.bak` recovery files.
