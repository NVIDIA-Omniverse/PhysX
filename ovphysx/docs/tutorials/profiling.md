<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Profiling: See Where Your Simulation Spends Its Time

This tutorial shows how to make an ovphysx workload publish its profile zones, so you can see which ovphysx call and which PhysX simulation phase each span of time belongs to.

Two consumers are selectable. **NVTX** feeds [NVIDIA Nsight Systems](https://developer.nvidia.com/nsight-systems), which correlates the zones with the CUDA activity it already records. **Trace** writes a chrome trace JSON with no external tool at all, which you can open in [Perfetto](https://ui.perfetto.dev) or parse from a script.

The instrumentation is compiled into release builds and the shipped wheel, so profiling never requires a custom build. No backend is selected by default, and while none is bound a zone costs an atomic load and a branch.

This page covers only what is specific to ovphysx. For Nsight recording options, the timeline UI, report scripts, and GPU metrics, refer to the [Nsight Systems User Guide](https://docs.nvidia.com/nsight-systems/UserGuide/index.html).

## Prerequisites

- Install ovphysx and confirm native libraries load.
- This tutorial uses `links_chain_sample.usda`, which ships with every package under
  `ovphysx/samples/data/` in the wheel, `<sdk-root>/samples/data/` in the C/C++ SDK,
  and `tests/data/` in a repository checkout.
- For the NVTX backend, install Nsight Systems to record and view a capture. Selecting it without Nsight is harmless, just uninteresting.

## Selecting a Backend

Set the backend **before** instance creation. The environment variable overrides the global setter:

| Python | C | Environment |
|---|---|---|
| `set_global_config(GlobalConfigKey.PROFILER_BACKEND, ...)` | `ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, ...)` | `OVPHYSX_PROFILER=none\|nvtx\|trace` |
| `set_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH, ...)` | `ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH, ...)` | `OVPHYSX_PROFILER_OUTPUT=<file>` |

The backend is bound during physics engine startup. Prefer the environment variable when you are not changing application source; prefer the setter when the process decides at runtime. `get_global_config(GlobalConfigKey.PROFILER_BACKEND)` / `ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, &value)` reports the effective selection.

The global setter is startup-only: after the first instance is created, changing
`PROFILER_BACKEND` returns an error. An instance created after one that selected
a backend inherits it. `OVPHYSX_PROFILER` remains authoritative when the backend
is resolved; use a separate process for a different programmatic selection.

Because this is a create-time decision, profile a slice of a long run by scoping the *collection* with Nsight's own delay, duration, and capture-range options rather than by toggling the instrumentation.

## Code

### Python

```{literalinclude} ../../tests/python_samples/profiling.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C

Set the backend before `ovphysx_create_instance()`:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

int main(void)
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_result_t result = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND,
        ovphysx_config_value_int64(OVPHYSX_PROFILER_BACKEND_TRACE));
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;
    result = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH,
        ovphysx_config_value_string(OVPHYSX_LITERAL("my_run.json")));
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_instance_handle_t handle = 0;
    result = ovphysx_create_instance(&handle);
    if (result.status == OVPHYSX_API_SUCCESS) {
        ovphysx_config_value_t backend = ovphysx_config_value_int64(0);
        result = ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, &backend);
        ovphysx_destroy_instance(handle);
    }
    ovphysx_shutdown();
    return result.status == OVPHYSX_API_SUCCESS ? 0 : 1;
}
```

The C++ benchmark harness accepts `--profiler=<backend>` and `--profiler-output=<file>`, which are sugar for the environment variables.

## The Trace Backend

```bash
OVPHYSX_PROFILER=trace OVPHYSX_PROFILER_OUTPUT=run.json python my_workload.py
```

The file is written when the **last** ovphysx instance in the process shuts down, because that is when the profiler is released. It will not exist while a simulation is still alive, and a process that never destroys its instances never produces one. With no output path set, the trace lands in the working directory as `ct-profile <date> <time>.json`.

The output is plain JSON, not gzip, so a script can read it directly. Traces grow quickly: a 120-step run of the sample scene produces roughly 20 MB, so budget accordingly on a long run and compress the file yourself if you need to keep it.

## Recording an NVTX Capture

Include the `nvtx` trace domain alongside `cuda`:

```bash
OVPHYSX_PROFILER=nvtx nsys profile -t nvtx,cuda -o my_capture python my_workload.py
```

## What You See

Three groups of zones appear, whichever backend you chose:

| Group | Contents |
|---|---|
| `ovphysx_*` | The API calls: `ovphysx_step`, `ovphysx_step_sync`, `ovphysx_wait_task`, `ovphysx_wait_all`, `ovphysx_attach_ovstage`, `ovphysx_clone`, and the tensor binding create / read / write calls |
| Runtime zones | The host-side work between those: USD parsing and stage load, collision cooking, the PhysX task names as they are dispatched, and the per-step orchestration around the SDK call |
| PhysX SDK zones | The SDK's own zones for both CPU and GPU work |

That middle group is what makes a slow `ovphysx_attach_ovstage` readable rather than merely large: it breaks the attach down into parsing, cooking and scene construction.

Every zone follows the same route, so under the NVTX backend they all land in NVTX's default domain, unprefixed.

An `nsys stats --report nvtx_pushpop_sum` excerpt from a stage-attach and stepping workload, abbreviated:

```text
 Time (%)  Total Time (ns)  Instances    Avg (ns)     Range
     59,0      18317627149      59545      307626,0   :Running fiber
      8,0       2504407895        687     3645426,0   :ovphysx_wait_task
      6,0       2048330234      41527       49325,0   :Executing task
      2,0        642633130         25    25705325,0   :ovphysx_attach_ovstage
      2,0        623135765         24    25963990,0   :Physics Load Stage
      1,0        552981192        321     1722682,0   :PxgGpuTask
      1,0        533954612         23    23215417,0   :UsdPhysics:processScannedDescs
      1,0        425303983        321     1324934,0   :GpuDynamics.Solve
      1,0        304712212        621      490679,0   :Basic.updateBroadPhase
```

Two things to read past. `Running fiber` and `Executing task` are Carbonite's task-system zones, not physics work, and they dominate any summary sorted by total time because they wrap everything else — start from the named physics rows instead. And such a summary reports *inclusive* time, so nested zones are counted at every level; it is a localization aid, not a budget.

The host-side zones are not free to record. Several of the busiest ones build their names per prim, so a capture of a large stage attach measures a slower attach than an unprofiled run. Compare captures with captures, and take absolute attach times from a run with profiling off.

Two things commonly surprise people reading a first capture:

- `ovphysx_step` looks almost free. It only enqueues the step; the simulation cost appears under `ovphysx_wait_task`. That is the asynchronous execution model, not a measurement error.
- The first capture of a scene looks nothing like the second. Cooking zones only do work on a cold cooked-collider cache, so a warm run collapses them to almost nothing.

## What No Capture Contains

The PhysX SDK's twelve **cross-thread** phases are not reported, under any backend:

`Basic.simulate`, `Basic.collision`, `Basic.narrowPhase`, `Basic.broadPhase`, `Basic.postBroadPhase`, `Basic.rigidBodySolver`, `Basic.dynamics`, `Basic.fetchResults`, `Basic.processCallbacks`, `Basic.sceneQueriesUpdate`, `Basic.fetchQueries`, and `Sim.ccdBroadPhaseComplete`.

These may close on a different thread than they opened on. The Carbonite profiler interface models a zone as a per-thread begin/end pair and cannot express that — its one cross-thread primitive draws an arrow between two points rather than a range with a duration — so the runtime discards them rather than begin a zone on one thread and end it on another.

The outermost phase is bracketed by zones you do have: `Basic.simulate` spans the same work as `ovphysx_step` through `ovphysx_wait_task`. For a breakdown within the step, the SDK's thread-local zones are all present — `Sim.`, `GpuDynamics.*`, `Basic.updateBroadPhase` and the rest — so what is missing is the top-level phase grouping rather than the detail underneath it.

## Attributing Memory Traffic

Host-device transfer volume and time are already in any Nsight capture recorded with `-t cuda`, and the `cuda_gpu_mem_size_sum` and `cuda_gpu_mem_time_sum` reports break them down by operation. What the zones add is attribution: each burst lines up with the `ovphysx_read_tensor_binding` or `ovphysx_write_tensor_binding` call that caused it. This one is NVTX-only — a chrome trace has no CUDA activity to correlate against.

Read the transfer count alongside the volume. A workload reading state back through tensor bindings typically issues many small copies rather than a few large ones, and a run dominated by sub-kilobyte transfers is limited by per-copy overhead rather than by bandwidth. In that regime, reducing the number of copies — batching reads, or avoiding readback entirely with DirectGPU — pays off where a faster link does not.

Two related questions belong to other tools: how saturated device DRAM is comes from Nsight Systems' GPU Metrics rows, which need [performance-counter access](https://developer.nvidia.com/ERR_NVGPUCTRPERM), and which kernel is bandwidth-bound is a question for [Nsight Compute](https://docs.nvidia.com/nsight-compute/). Use a Systems capture to find the kernel, then Compute to explain it.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Capture or trace is empty | No backend selected, or selected after instance creation | Set `OVPHYSX_PROFILER` before the process starts, or call `set_global_config` / `ovphysx_set_global_config` before create |
| No trace file appeared | The process still holds a live instance, or never destroyed one | The file is written at last-instance shutdown; destroy every instance |
| A capture came back in the wrong format | Another instance already bound a different backend | The first selection wins process-wide; check the ovphysx log at `WARN` for the line naming the bound backend |
| API and runtime zones appear but SDK ones do not | ovphysx built against a PhysX `release` package, where the SDK's profile zones are compiled out | Use the shipped build; the packaged PhysX is the `checked` config, which keeps its zones |
| A simulation phase seems absent | The SDK's twelve cross-thread phases are not reported under any backend | Use `ovphysx_step` through `ovphysx_wait_task` for the outermost span, and the SDK's thread-local zones for the detail |
| No zones at all, but the log says a backend bound | Something in the process registered its own Carbonite profiler backend first | Check the ovphysx log at `INFO` for the line naming which backend bound; the host's backend owns the process profiler |
| No GPU zones | The scene ran on CPU | Confirm the scene uses GPU dynamics and that a CUDA device is available |
| Nsight shows CUDA but no NVTX rows | `nvtx` not in the trace selection | Include it: `-t nvtx,cuda` |

## Result

After this tutorial you can publish any ovphysx workload's API calls, host-side stage load and cooking work, and PhysX simulation phases to either a chrome trace or an Nsight Systems capture, without rebuilding the library.
