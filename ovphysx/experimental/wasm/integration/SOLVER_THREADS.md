<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# CPU solver worker experiment

The default JavaScript API uses zero additional solver workers. The configurable
runtime accepts `createPhysics(stage, { solverWorkers: 0 | 2 | 4 })`, selected before
attachment. Zero executes physics on the caller. One side binary supports all
three counts; no automatic scene heuristic selects a count.

The existing native create ABI is unchanged. Its new
`_ovp_wasm_configure_solver_threads(count)` setter accepts only 0, 2 or 4 before
initialization or instance ownership. `_ovp_wasm_solver_threads()` reports the
selected count. `OVP_WASM_SOLVER_THREADS` remains the raw binding's compile-time
default so the earlier fixed-count comparison builds stay reproducible. The
package facade explicitly selects zero by default; old nonconfigurable modules
support that default and reject explicit nonzero requests.

The binding uses existing native configuration: `OVPHYSX_GLOBAL_CONFIG_NUM_THREADS`
and `/physics/physxDispatcher`. Each create explicitly selects the SDK dispatcher
for nonzero counts and restores `false` for zero, including a 4-to-0 transition. `PhysXSetup::acquireAttachmentResources()`
selects `PxDefaultCpuDispatcherCreate()` through that setting. The dispatcher
belongs to the attachment and is shared across its scenes. Destruction releases
native scene resources before releasing the dispatcher, which signals and joins
its workers. No solver, dispatcher algorithm or Carbonite scheduler is replaced.
CPU-only and synchronous instance execution remain required.

Changing NUM_THREADS alone cannot enable workers: the Carbonite dispatcher clamps
its worker count to the host scheduler's zero capacity. The host intentionally
keeps that zero-worker descriptor and its unsupported scheduling operations. The
SDK dispatcher is the existing alternative that can use Emscripten pthreads.

The first real two-worker test exposed a second native assumption:
`PhysXStepper::launch()` schedules its outer step through Carbonite whenever the
solver dispatcher has workers. `host/prepare-stepper.py` generates a scoped
source overlay for this case. Only an Emscripten build with a synchronous scene,
synchronous C API execution, the SDK dispatcher, and zero Carbonite host workers runs
the existing stepper on its caller. `run()` still uses `simulate()` followed by
`fetchResults(true)` and dispatches the actual physics tasks to the SDK workers.
The direct simulate/fetch API passes `forceAsync=true` for its event and flush
semantics even with synchronous C API execution. The overlay preserves that value
and checks the actual `OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION` setting instead.
Native builds and other scheduling modes retain the original launch branch.
The helper saves the exact original source and hashes both versions. It does not
replace the Carbonite task table or implement a second scheduler.

## Capacity and ownership

Use a matching main module with a preloaded pool of 16 workers for the initial
2/4-worker experiments. The current pool of 8 also serves TBB/population; the
thread count is not a solver-only budget. SDF cooking remains on its existing
serial path, rather than requesting 16 additional cooking workers.

Before attachment, the owner must call the `_ovp_wasm_solver_threads()` getter and
check the pinned Emscripten `runtime.PThread.unusedWorkers.length`. The local
`solver-capacity.mjs` harness requires the requested count plus four idle workers
of headroom for the bounded population host. Zero-worker builds require no pool
capacity check. Recheck after every stage open and before every reattachment;
observe live worker counts and that teardown returns workers to the pool.

This guard matters because the SDK's existing `PxThreadImpl::start()` waits for a
thread-start signal after `pthread_create()`. If the fixed Emscripten pool is
exhausted, that wait can hang. A larger configured pool alone is insufficient as
a shipping contract. The package owner guard and capacity-failure tests must be
present before enabling a threaded default. Raw C binding callers own this
preflight responsibility. The getter reports configured additional solver
workers, not total process threads. The recipe retains an eight-worker default
pool and permits explicit pool sizes. Pool12 passed raw native selection but had
only six idle workers after a default simulation in the package worker, so its
four-worker request correctly rejected. Pool16 passed the public 0-to-4-to-0-to-2
sequence. Preload cost and workload benefit must be measured before changing the
default pool. The receipt distinguishes `solverConfiguration.supportedWorkers`
and `nativeDefaultWorkers` from each attachment's actual requested/getter count.

A separate main-module lifetime issue was found during repeated-owner testing:
releasing the last TBB `global_control` between owners restores the machine's
hardware concurrency, and background TBB workers can exhaust the pool before the
next attachment. The isolated thread qualification bridge retains a one-thread
control between owners and constructs its replacement before releasing the old
control. Idle population-thread configuration remains allowed when no stage is
active. This bridge override is recorded separately from the canonical build;
it does not silently modify the stage recipe. Solver startup/join checks passed
with this bounded lifetime. Full same-module schema/provider/render lifecycle
qualification is a separate requirement, not established by physics tests alone.

## Validation and comparison

Run under an external timeout so worker-start/teardown regressions are observable:

```sh
timeout 90s /tmp/ovstage-wasm-emsdk/node/24.19.0_64bit/bin/node \
  ovphysx/experimental/wasm/integration/test-runtime-solver-threads.mjs \
  /path/to/runtime/ovstage.mjs /path/to/runtime/ovphysx.wasm 4 /tmp/solver-4.json
```

On a failed assertion or rejected initialization, the harness terminates only
the pthread pools it created. The external timeout still catches native hangs.

The focused harness rejects insufficient or unavailable owner pool state before
native attachment, observes actual worker allocation/return, and runs the real
publication, contact, articulation and serial SDF-cooking contracts. Publication
checks fresh physics positions against stage transforms/bounds, signed and nested
scales, hierarchy folding, rejected ordinals, and owner recreation.

After correctness, compare 0/2/4 on the same pyramid input and a bounded generated
10k-box scene with the same solver settings, timestep, warmup and measurement
window. Separate attachment latency, physics step, output publication and stage
readback. Record the preloaded pool size and compile flags. Thread counts can
change solver scheduling and floating-point trajectories; require finite output,
contacts, constraints and pose/publication consistency rather than identical
cross-run trajectories. Node checks are prerequisites; repeat on the actual
isolated browser worker before shipping.

The browser harness hosts only the selected runtime and test fixtures on an
ephemeral local port, then runs the same contracts in a dedicated Chrome worker.
It records artifact hashes, actual worker startup/join, and responsive UI ticks:

```sh
timeout 90s node \
  ovphysx/experimental/wasm/integration/test-runtime-solver-threads-browser.mjs \
  /path/to/runtime 4 /path/to/playwright-core/index.mjs /tmp/solver-4-browser.json
```

Neither harness starts or restarts the standalone editor service. Both remain
physics qualification, rather than a full renderer/provider lifecycle test.

For an exception-mode comparison with a known raw-module lifetime limitation,
append `single-owner` to the browser command. Each publication or feature scenario
then receives its own dedicated worker, native module and USD stage. The receipt
explicitly excludes module/worker reuse, while publication still checks native
error recovery and detach/reattach on the same stage. The default `full` mode
retains the existing repeated-owner assertions; `single-owner` is not evidence
that a lifetime problem was fixed.


## Runtime-selectable count contract

```sh
timeout 90s node \
  ovphysx/experimental/wasm/integration/test-runtime-solver-options.mjs \
  /path/to/runtime/ovstage.mjs /path/to/runtime/ovphysx.wasm /tmp/solver-options.json
```

The contract uses one USD stage and one side module for 0-to-4-to-0-to-2 attachment
cycles. Each record includes the native getter, capacity, real running/idle worker
counts before attach/after attach/after detach, and fresh pose/scale/bound checks.
Invalid and live reconfiguration must reject without changing the selection;
failed attachment cleanup must allow a new selection. No raw provider/owner
reuse claim follows from this test.

The browser harness accepts `runtime-options` after its report path for the same
native contract. `OVPHYSX_BROWSER=firefox` selects the small WebDriver adapter;
`GECKODRIVER` and `FIREFOX_BIN` may override the installed executables. It uses an
owned temporary profile and default WASM settings. The default engine remains
Chrome. Public facade tests in the standalone worktree additionally exercise the
option through worker RPC and preserve the authored-input reattachment gate.
