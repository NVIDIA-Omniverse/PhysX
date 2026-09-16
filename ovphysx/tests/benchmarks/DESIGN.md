<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

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
| `Harness.cpp/.h` | adapted | the normal run loop and trimmed report statistics are preserved; we add the DirectGPU CLI opt-in, optional all-sample JSONL diagnostics, static Carbonite bootstrap, RAII ownership that keeps benchmark destruction before `malloc_trim()`, and a nonzero result when a benchmark throws or diagnostics output fails |
| `TimingDiagnostics.cpp/.h` | ovphysx-specific | computes count, arithmetic mean, population standard deviation, minimum, and maximum without changing or storing the raw sample array |
| `BenchmarkList.cpp/.h` | adapted | same shape; the `extern void init...()` set lists ovphysx's tests, not the source harness tests |
| `benchmarks/*.cpp` | ovphysx-specific | the cases below |

The benchmark-side tests are deliberately ours: the source benchmark cases
touch internal runtime and raw PhysX scene objects that the ovphysx wrapper
hides. Timed benchmark operations call the public `ovphysx::PhysX` or
`ovphysx_*` C ABI. ContactReport has one untimed validation-only exception: the
public report exposes runtime SdfPath-node identities but no public function
converts an expected path to that representation, so the benchmark uses the
internal `sdfPathToInt()` encoder to construct its expected pair set. That
internal conversion is not part of the measured or supported consumer path.

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
       │    ├─ optional all-sample summary before trimming
       │    ├─ sort, retain the harness's inclusive index window,
       │    │   then take trimmed mean and stddev
       │    └─ BmRecord goes into the results array
       ├─ BmOutput::emit(...)                      (BmOutput.cpp)
       │    – writes report and compares against baseline
       ├─ optional JSONL write after successful emit
       └─ bmTerminate() → bmDestroyGlobals()
```

The harness binary is launched in separate GPU, CPU, and single-threaded CPU
processes by `scripts/test_benchmarks_cpp.cmake`. The GPU process receives
`--forceGpu`; the single-threaded process receives `--threads=1` and is scoped
to `Step.cubes20_cpu`. CPU-only and GPU-only
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
| `UsdLoad.cubes20` | any | 20-cube overhead-probe fixture parse cost. |
| `InstancingAttach.{physics,all}_n{0100,0250,0500}_cpu` | hidden cpu | Initial ovphysx attach + completion wait for five fixed leaf colliders as unrelated render-only scene-graph prototypes grow. |
| `InstancingAttach.{physics,all}_flat_n0500_cpu` | hidden cpu | Non-instanced control for the 500-prototype rows (`instanceable = false`, same prim count). |
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
| `ContactReport.persistent_pairs_512_step_read_{cpu,gpu}` | hidden cpu/requested conventional gpu | One 1/60-second synchronous step plus the public raw contact-report pull for 512 persistent isolated reporter/static pairs. DirectGPU is refused. |
| `Probe.cartpole_4096_control_step` | hidden direct gpu | Physics-owned stable backend policy step: two 1/120 s substeps with CUDA DOF control writes and state reads. |
| `Probe.cartpole_4096_tensor_binding_create` | hidden direct gpu | Creation and spec lookup for the five `[4096,2]` bindings used by the Cartpole control row; exact-filter fresh-process runs measure first use. |
| `Lab.cartpole_<N>_{step,reset,tensor_read,tensor_write}` | gpu | IsaacLab-style cartpole replication: 4 ops × 3 sizes (4096/8192/16384). |
| `Authoring.population_add_drip_{cpu,gpu}` | hidden cpu/requested gpu | Primary KPI on CPU only. 80 one-body population add/apply/seal/drain/step cycles; GPU is diagnostic. |
| `Authoring.population_add_packed_{cpu,gpu}` | hidden cpu/requested gpu | Diagnostic. 80 bodies in five 16-body population cycles. |
| `Authoring.runtime_write_spawn_{cpu,gpu}` | hidden cpu/requested gpu | Diagnostic. 80 bodies via direct OVStage column writes in five batches, no population references. |
| `Authoring.population_churn_{cpu,gpu}` | hidden cpu/requested gpu | Diagnostic. Population add/drain/remove/delete/drain/step; no `OVSTAGE_PRIM_MODE_UPSERT`. |
| `Authoring.teleport_{tensor,ovstage}_{cpu,gpu}` | hidden cpu/requested gpu | Diagnostic. Existing-body transform refresh through a tensor write or the OVStage control plane. |
| `Authoring.drain_{ovstage,tensor,step_only}_cpu` | hidden cpu | Diagnostic decomposition of velocity-control cost against a step-only floor. |
| `WriteScaling.velocity_ovstage_4096_cpu` | hidden cpu | Primary KPI. 4,096-body OVStage linear-velocity write and step. |
| `WriteScaling.velocity_tensor_4096_cpu` | hidden cpu | Comparator. Persistent `[4096,6]` velocity tensor write and step. |
| `WriteScalingHighN.velocity_{ovstage,tensor}_{8192,16384}_cpu` | hidden cpu | Diagnostic. The WriteScaling pair at 8,192 and 16,384 bodies; separate family so the L1B selection is unchanged. |
| `RuntimeSpawnScaling.collider_heavy_1280_cpu` | hidden cpu | Diagnostic. Seal, structural drain, and first step of one runtime-spawned body into 1,280 existing static colliders. |
| `Lab.anymal_<N>_{step,reset,tensor_read,tensor_write,clone}` | gpu | IsaacLab-style Anymal replication: 5 ops × 2 sizes (1024/8192). Skip until asset published. |

### Why these particular fixtures

Fixtures normally live in `tests/benchmarks/data/` and are generated by
per-fixture `gen_*.py` scripts for reproducibility. Articulation pileup gives a
real signal under the articulation solver; cubes20 acts as a deliberately
minimal overhead probe; warehouse covers larger-scene startup. Those generated
stepping fixtures author `physxScene:timeStepsPerSecond = 240`; the Cartpole
probe fixture authors 120 Hz to match its pinned control-step contract.
The probe is hidden from the default passes; its dedicated runner and the
documented direct invocation opt into both DirectGPU and hidden rows. It fixes
OVPhysX and DLPack buffers to visible CUDA device 0 and uses the IsaacLab
DirectGPU bootstrap settings, including eight worker threads.
Its two preallocated force tensors reverse signs on every outer control step,
keeping the carts away from their joint limits during the default run.
Indexed/subset writes are not part of this first stable bulk-control row.
The separate creation row loads and clones the same fixture, warms the runtime,
then starts its timer immediately before the first of five
`createTensorBinding()` calls and stops after the fifth `spec()` call. It
allocates no caller data buffers and performs no reads, writes, or physics
steps; any internal allocation performed by `createTensorBinding()` is part of
the number. Specification and topology validation plus binding destruction
follow the timer. The supported runner launches this exact filter in a fresh
process, and the row rejects a second timed sequence so that path cannot turn a
first-use measurement into a warmed recreation.
OVPhysX reports a binding's native no-staging DLPack device through
`ovphysx_get_tensor_binding_native_device()`. OMPE-103958 can still track any
measured CPU/CUDA transfers when benchmark storage differs from that reported
native device.

The instancing-attach family captures NVBug 6532970 as a diagnostic, not a
committed wall-clock regression threshold. It builds the USDA string once
outside the run loop, then populates and seals OVStage before each measurement.
Only initial `ovphysx_attach_ovstage` plus its completion wait is timed; ordinal
1 is not replayed through an update, so the family covers the attach phase only
and not the structural-update phase the bug reports at comparable cost. The
`physics` and `all` population-domain pairs keep five realized rigid bodies
fixed while changing only the number of unrelated render-only prototypes. The
rows remain hidden until stable same-machine history and an explicit monitoring
policy exist.

Two things bound how these numbers should be read. Trunk already carries the
first scoping fix for the bug (`16f12f0cc6`), so the rows measure the residual
rather than the originally filed cost, and prototype counts are 100/250/500 to
match the table filed on the bug. The `*_flat_*` rows are the control: same
composition, same populated prim count, `instanceable = false`. A flat row that
costs as much as its instanced sibling means the residual is not
instancing-specific, and the owning fix is not an instancing rework. Read the
two terms separately: the instanced-minus-flat difference is the instancing
cost, and the flat-minus-physics difference is the part that only tracks
populated prim count and would survive an instancing rework.

Subtract the paired `physics` row before fitting a scaling exponent. That row
is the fixed attach floor, and leaving it in biases the exponent downward --
most at the smallest prototype count, where the floor is the larger share of
the measurement.

### L1B measurement policy and fixtures

The L1B producer contract is the eighteen hidden `Authoring.*` and
`WriteScaling.*` rows: twelve CPU rows and six requested-GPU diagnostics.
Normal wildcard runs do not see them. CPU is canonical, and only two absolute
latencies are primary lower-is-better KPIs:

- `Authoring.population_add_drip_cpu`: one measured step contains 80 complete
  one-body population add/wait, apply/wait, seal/drain, and 1/60-second
  synchronous-step cycles.
- `WriteScaling.velocity_ovstage_4096_cpu`: each of 20 measured step indices
  contains a 4,096-body velocity query/write and completion wait, query
  release, velocity-scoped floor and completion wait, OVStage update, then a
  1/60-second asynchronous step and `waitAll()`.

The other CPU rows are diagnostics or comparators. `population_churn` is
specifically population-reference create/drain/remove plus attribute deletion,
a second drain, and a step. It never invokes `OVSTAGE_PRIM_MODE_UPSERT`.
WriteScaling's tensor comparator uses one persistent `[4096,6]` binding, write
completion, and the same asynchronous step/wait shape. Ratios, deltas, and
slopes derived from any pair remain descriptive rather than KPIs.

All setup is outside the timer: scene creation/load, population, attachment,
write-path construction, and `ovphysx_warmup()`. `ovphysx_warmup()` is the
current name of the API that initializes PhysX lazy structures and disables
per-step Fabric synchronization overhead. Teleport and drain add five and ten
same-stage untimed iterations respectively. Semantic gates also run after
timing. Growth rows check realized body counts; churn checks return to the
body-count floor, proves exact-path creation, then proves exact-path removal;
write rows read back their pose or velocity; WriteScaling checks the distinctive
velocity on all 4,096 bodies.

Every operational or semantic error is recorded against one exact row. The
harness omits that row's record and exits nonzero. A local non-regenerate
baseline breach may already have printed a failed comparison before the exit;
the dedicated contract uses `--regenerate`, so it has no baseline comparison.
The CMake driver additionally treats a nonzero process, missing report, and
requested exact-row mismatch as fatal.

The Authoring sources consume four fixtures from `ovphysx/tests/data/`, shared
with the c_samples and shipped in the SDK/wheel sample payload:

| Fixture | Consumers | Authored/default timestep |
|---|---|---|
| `empty_dynamic_boxes.usda` | requested-GPU population add, runtime-write spawn, and population churn | No `physxScene:timeStepsPerSecond`; schema default 60 Hz. |
| `empty_dynamic_boxes_cpu.usda` | CPU population add, runtime-write spawn, and population churn | CPU dynamics + MBP overlay; timestep remains unauthored, default 60 Hz. |
| `simple_physics_scene.usda` | requested-GPU teleport | No `physxScene:timeStepsPerSecond`; schema default 60 Hz. |
| `simple_physics_scene_cpu.usda` | CPU teleport and drain | CPU dynamics + MBP overlay; timestep remains unauthored, default 60 Hz. |

WriteScaling uses no file fixture. Its generated stage explicitly authors
`physxScene:timeStepsPerSecond = 240` and verifies CPU dynamics plus MBP on the
PhysicsScene, while each outer benchmark step advances 1/60 second.

The requested-GPU Authoring rows are retained only as hidden, unscheduled
diagnostics. Their `forceGpu()` gate and scene authoring express intent, not
realized-device proof. A warning callback scoped across attachment and warm-up
recognizes known CPU-fallback and GPU contact-capacity messages. Either warning
fails the requested-GPU Authoring row; silence in that window is only negative
fallback and capacity evidence because ovphysx has no realized-device query.
This pass/fail rule applies to all six hidden GPU diagnostics. Their frozen
names and the CPU KPI contract are unchanged, and the rows are neither
baselined nor compared with CPU.

Finally, the two families are not timing-compatible. WriteScaling deliberately
preserves its prototype's write-completion wait, query release, scoped-floor
wait, and `step()`/`waitAll()` path. Authoring OVStage rows use the later
single-seal-wait boundary and `ovphysx_step_sync()`, which avoids asynchronous
operation machinery. Compare WriteScaling only within its pair.

Even within that pair, the OVStage lane creates and releases a path-list query
per measured step and waits separately for the write, query release, and
scoped floor before the common asynchronous step/wait. The tensor lane reuses a
persistent `[N,6]` binding and has only its write-completion wait before that
step. Their gap is therefore an end-to-end control-path comparison, including
addressing, query lifetime, and synchronization, not a pure write-cost
difference. Put both rows on one plot only to show that end-to-end comparison;
do not attribute the delta to the write operation alone. The OVStage lane also
writes only linear `physics:velocity`, while the tensor lane writes linear plus
zeroed angular velocity; the shared claim is the validated linear-velocity
outcome, not identical full-state work.

### High-scale diagnostics policy

Five hidden CPU-only diagnostics sit beside L1B without joining it
(REQ-CAPI-BENCHMARK-005). They have their own inventory,
`tests/high_scale_inventory.json`, and their own family prefixes,
`WriteScalingHighN.*` and `RuntimeSpawnScaling.*`, chosen so that neither the
L1B contract filter `Authoring.*:WriteScaling.*` nor the scheduled L1B filter
`Authoring.*_cpu:WriteScaling.*_cpu` can select them. The contract proves the
union of all four inventories (L1B, OutputRead, ContactReport, high-scale) is
free of duplicate and substring-colliding names, and that no name can match
inside another row's record after the harness appends its `_<N>T` or `_GPU`
postfix, because every row shares one strstr()-matched baseline lookup.

The four `WriteScalingHighN.*` rows are `WriteScalingBase` at 8,192 and 16,384
bodies and nothing else: the same generated CPU scene, the same lane writes,
the same 20-by-5 trimmed statistic, and the same untimed gate that reads the
written velocity back from every body. They extend the write-scaling picture
past the 4,096-body L1B point without moving that point. The generated scene
text is cached per scale because one process runs all three scales in sorted
row order.

`RuntimeSpawnScaling.collider_heavy_1280_cpu` prices structural ingestion into
a populated scene. Its generated scene carries exactly 1,280 static
`PhysicsCollisionAPI` cubes, zero gravity, and the CPU dynamics/MBP lines; the
row verifies all of that in the text, populates and attaches the scene, warms
it, reads back a zero rigid-body seed count, and authors one body through the
unchanged `Authoring.runtime_write_spawn` column writes plus one `omni:xform`
write carrying the same pose, all outside the timer. The `omni:xform` write is
needed because ovphysx resolves a created prim's local transform from that
data-plane matrix rather than from the `xformOp:*` columns; the spawn columns
alone realize the body at the identity pose.
The timed sample is exactly the seal (`ovstage_advance_write_floor` plus its
completion wait), the `ovphysx_update_from_ovstage` drain, and one
`ovphysx_step_sync(1/60 s)`. The untimed gate requires exactly one dynamic
rigid body, a rigid-body classification of `/World/Runtime/Body_0000` through
`ovphysx_get_object_type()`, and a pose readback at the authored height. The
public API has no realized-static-shape query, so the collider count is a
text-level gate.

None of the five rows is a KPI. Any consumer threshold is an absolute
lower-is-better non-paging fence, no ratio or slope derived from them is a KPI,
no GPU variant exists, and they are never compared with the L1B
`WriteScaling.*` rows as one series.

### ContactReport measurement policy and fixture

ContactReport is a separate two-row hidden contract. It does not change the
eighteen-row L1B inventory. Both rows are lower-is-better absolute
latencies; the CPU registration is emitted unchanged and the GPU harness emits
`ContactReport.persistent_pairs_512_step_read_gpu_GPU`.

The runtime-generated stage places 512 isolated sphere pairs on a fixed grid.
Each pair consists of one awake dynamic reporting sphere and one overlapping
static sphere. Four-unit cell spacing prevents cross-pair contacts. Gravity and
sleeping are disabled, and `physxRigidBody:solveContact = false` keeps the
overlap stable without charging the row for contact solving. The dynamic actor
applies `PhysxContactReportAPI` with threshold zero. CPU authors CPU dynamics
and MBP. GPU authors GPU dynamics and GPU broadphase, with the three capacities
pinned to the exact schema defaults: 524288 rigid contacts, 81920 rigid
patches, and 262144 found/lost pairs.

Scene generation, population, attachment, expected-pair construction, and
warm-up precede the timer. Warm-up advances at most 32 steps and requires three
consecutive structurally valid, all-persistent reports with exactly 512 headers,
512 points, and zero friction anchors. A measured sample times one 1/60-second
`ovphysx_step_sync()` followed
immediately by `ovphysx_get_contact_report()`. Semantic validation starts only
after the timer stops and completes before another simulation step invalidates
the borrowed views. The default 20 steps by five runs therefore produce 100
actual timed calls in the diagnostics sidecar.

The semantic gate requires exactly 512 headers and 512 points, persistent event
type, the live attach handle, the exact Reporter/Static identity set, valid
actor/collider and prototype fields, a complete non-overlapping point partition,
zero global friction anchors, zero per-header friction-anchor offsets and counts,
and finite point positions, normals, impulses, and separations with sane normal
length and separation range. A known GPU fallback or contact-capacity warning
is also a failure. Any operational or semantic failure suppresses the row and
makes the process nonzero.

The fixture intentionally exercises one contact point per pair/header, zero
friction anchors from `solveContact=false`, and one default-material case. It
does not represent multi-point manifolds, friction-anchor report payloads, or
material-diverse contact reporting.

The GPU row is conventional GPU, selected with `--forceGpu` and without
`--directGpu`. DirectGPU disables the host simulation-state readback path used
by the public raw getter and is rejected. Requested GPU plus silence from the
warning detector is not positive realized-device proof; no public
realized-device query exists. The rows also exclude initial attach and
parameter parsing, solver cost, Contact Binding, transport, and application
scheduling.

Qualification noise is calculated from the ordinary report's emitted
`std_dev / avg` values. The all-sample diagnostics statistics do not define that
gate; their contract here is the exact count of 100 actual timed calls.

## Trim and statistics

The harness runs `R` runs × `S` steps, transposes to `[S][R]`, and sorts each
step's timings across runs. It retains the inclusive zero-based index range
`[max(1, floor(0.2*R)), max(min(floor(0.7*R), R-2), 1)]`, which guarantees at
least one discarded run at each end but is not an exact 20%/30% split for small
run counts. It computes each step index's mean and population standard
deviation over that same retained set, then `bmGetDefaultResult()` takes the
mean of the `S` per-step means and the mean of the `S` per-step deviations.

Both primary L1B rows use `R=5`, retaining the middle three samples at every
step index. Population-add-drip uses `S=1`; WriteScaling uses `S=20`. The
trimmed-mean result is persisted in `_baseline.txt`, while standard deviation
is emitted only in regenerate-mode console/report output. That `std_dev`
describes within-invocation run spread averaged over step indices; it is never
cross-invocation variance.

When `--timing-diagnostics=<file>` is present, the harness also summarizes the
unmodified sequence of actual timed calls before sorting or trimming it. The
JSONL object contains count, arithmetic mean, population standard deviation,
minimum, and maximum. It excludes the dummy warm-up and counts the one-shot
path once rather than counting the three copies used only to satisfy the
legacy trim calculation. The object is written only after the normal row emits
successfully. The canonical report, detail CSV, baseline, and comparison value
do not consume these fields. The diagnostics and report paths must resolve to
different files, including through path aliases, so the two writers cannot
truncate or interleave each other's output.

## CLI

| Flag | Meaning |
|---|---|
| `--filter=<glob>` | glob-pattern filter (`*`, `?`, `:` for OR, `-` prefix to exclude) |
| `--list` | print and exit |
| `--forceGpu` | bring up the runtime as GPU |
| `--directGpu` | use DirectGPU settings; requires `--forceGpu` |
| `--data=<dir>` | data folder for fixtures |
| `--report=<file>` | report path |
| `--timing-diagnostics=<file>` | optional JSONL summary of every actual timed call before trimming; must resolve to a different file from `--report` |
| `--regenerate` | rewrite baseline with current results |
| `--slop=<pct>` | tolerances written during baseline regeneration (default 10); existing baseline values govern comparisons |
| `--steps=<N>` / `--runs=<N>` | override per-benchmark counts |
| `--detail` | per-step CSV dump |
| `--verbose` | name-of-current-benchmark logging |
| `--hidden` | also include benchmarks flagged hidden |

`--filter=` syntax matches the shared harness and is intentionally
not reduced.

Two consequences of that catch everyone once, because both fail as an empty
table rather than an error:

- The glob must match the **whole** name. A bare substring is not a substring
  search — `--filter=readonly_arti_link_8192_cpu` works, `--filter=arti_link`
  matches nothing. Wrap it: `--filter=*arti_link*`.
- A GPU lane needs `--forceGpu`, and `--hidden` if the row is registered hidden —
  the GPU variants are, so `--hidden` is what makes them runnable at all. The
  harness then appends `_GPU` to the name it prints, so the reported row does not
  read back as the name you filtered on.
- `--directGpu` is **not** a third flag every GPU lane takes. It applies to
  DirectGPU lanes only — the `OutputRead.*_gpu` family. A conventional GPU lane
  must run without it: `ContactReport.persistent_pairs_512_step_read_gpu` rejects
  it, because DirectGPU disables the host simulation-state readback path that
  row's public raw getter measures (see "ContactReport measurement policy" above).
  The driver's own GPU pass in `scripts/test_benchmarks_cpp.cmake` passes
  `--forceGpu` alone for the same reason; only the DirectGPU contract steps add
  `--directGpu`.

```
ovphysx_benchmarks --filter=OutputRead.readonly_arti_link_8192_cpu --runs=3

# DirectGPU lane -- all three
ovphysx_benchmarks --filter=OutputRead.readonly_arti_link_8192_gpu --forceGpu --directGpu --hidden --runs=3

# conventional GPU lane -- no --directGpu
ovphysx_benchmarks --filter=ContactReport.persistent_pairs_512_step_read_gpu --forceGpu --hidden --runs=3
```

## Build & CI integration

- Opt-in via `OVPHYSX_BUILD_BENCHMARKS=ON`. `./build.sh --benchmarks`
  forwards it.
- Driver script `scripts/test_benchmarks_cpp.cmake` runs separate GPU, CPU, and
  single-threaded CPU passes and writes per-pass logs under
  `_build/benchmark_results/` by default. `BENCHMARK_RESULTS_DIR` gives smoke
  consumers a separate evidence directory without changing that product
  default. `BENCHMARK_HIDDEN=1` reaches every enabled pass.
  `BENCHMARK_EXPECT_ROWS=N` is fail-closed and valid only with one pass, because
  the single-threaded pass overrides the user's filter.
- Dedicated job `ovphysx-linux-x86_64-benchmarks` performs its own
  `./build.sh --benchmarks`, SDK install, regenerated twelve-row CPU L1B
  contract, the separate CPU/conventional-GPU ContactReport contract, and
  DirectGPU Cartpole control and binding contracts. Each ContactReport row and
  the Cartpole control row must publish 100 all-sample timings; the binding row
  must publish one. The ContactReport contract also proves reciprocal
  wrong-device suppression and DirectGPU refusal. The same contract run ends
  with the high-scale section: collision-free inventory union, exact hidden
  five-row list, vacuity without `--hidden`, and one positive five-row run.
  The job reuses the Linux GPU-test runner/image/setup and credentials but has
  no publish or security consumers; benchmark artifacts are not added to the
  publish-critical `ovphysx-linux` job.
- ctest label `benchmarks` is excluded from `validate_all` via
  `-LE benchmarks`, so timing-sensitive jobs never run alongside
  correctness tests.

## Python suite

`tests/python_benchmarks/` is pytest-based, driven by `pytest-benchmark`.
Scoped per review to signals the C++ harness genuinely can't
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
