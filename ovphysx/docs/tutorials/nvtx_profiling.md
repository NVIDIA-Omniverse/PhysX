<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# NVTX Profiling in Nsight Systems

This tutorial shows how to make an ovphysx workload annotate itself for [NVIDIA Nsight Systems](https://developer.nvidia.com/nsight-systems). A capture then shows which ovphysx call and which PhysX simulation phase each span of time belongs to, correlated with the CUDA activity Nsight already records.

The instrumentation is compiled into release builds and the shipped wheel, so profiling never requires a custom build. Emission is off by default.

This page covers only what is specific to ovphysx. For recording options, the timeline UI, report scripts, and GPU metrics, refer to the [Nsight Systems User Guide](https://docs.nvidia.com/nsight-systems/UserGuide/index.html).

## Prerequisites

- Install ovphysx and confirm native libraries load.
- Prepare a USD file with physics objects.
- Install Nsight Systems to record and view a capture. Enabling NVTX without
  Nsight Systems installed is harmless and produces no useful capture UI.

## Required Config

Emission is controlled by one typed config field, or equivalently by an environment variable:

**NVTX emission config field and environment variable**

| Config field (Python) | C builder | Environment variable |
|---|---|---|
| `nvtx_enabled` | `ovphysx_config_entry_nvtx_enabled()` | `OVPHYSX_NVTX=1` |

It must be configured **before** the PhysX instance is created, because the PhysX profiler callback is installed during physics engine startup. Pass it through `PhysXConfig` (Python) or `config_entries` in `ovphysx_create_args` (C/C++), or set the environment variable before the process starts.

Prefer the environment variable when profiling an application whose source you are not changing; prefer the config field when the process itself decides whether it is being profiled. Both resolve to the same process-wide `/physics/nvtxEnabled` setting, and the effective state is readable with `physx.get_config_bool(ConfigBool.NVTX_ENABLED)` or `ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, &value)`.

That setting is sticky for the life of the process: an instance created after one that enabled NVTX inherits the enabled state unless it is explicitly turned off with `OVPHYSX_NVTX=0` or a false config entry.

Because this is a create-time decision, profile a slice of a long run by scoping the *collection* with Nsight's own delay, duration, and capture-range options rather than by toggling the instrumentation.

## Code

### Python

This sample enables NVTX at instance creation, then attaches a scene and steps
it so the capture has annotated work to show:

```{literalinclude} ../../tests/python_samples/nvtx_profiling.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C

Enabling it is a single config entry on the create args. Add these lines to your
existing instance-creation code:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

ovphysx_config_entry_t config[] = {
    ovphysx_config_entry_nvtx_enabled(true),
};

ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
args.config_entries = config;
args.config_entry_count = 1;

ovphysx_handle_t handle = 0;
ovphysx_result_t result = ovphysx_create_instance(&args, &handle);

/* Confirm what the process actually resolved. */
bool nvtx_on = false;
ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, &nvtx_on);
```

The C++ benchmark harness accepts `--nvtx`, which is sugar for the environment variable.

## Recording a Capture

Include the `nvtx` trace domain alongside `cuda`, and substitute your own script
for `my_workload.py`:

```bash
OVPHYSX_NVTX=1 nsys profile -t nvtx,cuda -o my_capture python my_workload.py
```

## Timeline Contents

Two NVTX domains appear:

**NVTX domains in an ovphysx capture**

| Domain | Contents |
|---|---|
| `ovphysx` | The API calls: `ovphysx_step`, `ovphysx_step_sync`, `ovphysx_step_n_sync`, `ovphysx_wait_op`, `ovphysx_attach_ovstage`, `ovphysx_clone`, and the tensor binding create / read / write calls |
| `PhysX` | The PhysX SDK profile zones nested inside them, for both CPU and GPU work |

The `PhysX` domain spans both NVTX range kinds, which Nsight reports separately. Most zones are thread-local push/pop ranges (`nvtx_pushpop_sum`). The high-level simulation phases — `Basic.simulate`, `Basic.collision`, `Basic.rigidBodySolver` and their siblings — are cross-thread ranges (`nvtx_startend_sum`), because they can close on a different thread than they opened on. Checking only one of the two report kinds is a common reason to conclude a phase is missing when it is not.

Two things commonly surprise people reading a first capture:

- `ovphysx_step` looks almost free. It only enqueues the step; the simulation cost appears under `ovphysx_wait_op`. That is the asynchronous execution model, not a measurement error.
- In a test or benchmark harness, `ovphysx_attach_ovstage` can dominate, because such harnesses attach a stage per case. A real application attaches once and steps many times.

## Attributing Memory Traffic

Host-device transfer volume and time are already in any capture recorded with `-t cuda`, and the `cuda_gpu_mem_size_sum` and `cuda_gpu_mem_time_sum` reports break them down by operation. What the NVTX ranges add is attribution: each burst lines up with the `ovphysx_read_tensor_binding` or `ovphysx_write_tensor_binding` call that caused it.

Read the transfer count alongside the volume. A workload reading state back through tensor bindings typically issues many small copies rather than a few large ones, and a run dominated by sub-kilobyte transfers is limited by per-copy overhead rather than by bandwidth. In that regime, reducing the number of copies — batching reads, or avoiding readback entirely with DirectGPU — pays off where a faster link does not.

Two related questions belong to other tools: how saturated device DRAM is comes from Nsight Systems' GPU Metrics rows, which need [performance-counter access](https://developer.nvidia.com/ERR_NVGPUCTRPERM), and which kernel is bandwidth-bound is a question for [Nsight Compute](https://docs.nvidia.com/nsight-compute/). Use a Systems capture to find the kernel, then Compute to explain it.

## Troubleshooting

These symptoms cover the enablement and trace-selection mistakes that leave NVTX
data out of a capture:

**NVTX capture symptoms and fixes**

| Symptom | Cause | Fix |
|---|---|---|
| Capture contains no NVTX data | Not enabled, or enabled after instance creation | Set `OVPHYSX_NVTX=1` before the process starts, or pass `nvtx_enabled` at init |
| `ovphysx` ranges appear but `PhysX` ones do not | ovphysx built against a PhysX `release` package, where the SDK's profile zones are compiled out | Use the shipped build; the packaged PhysX is the `checked` config, which keeps its zones |
| A simulation phase seems absent | The cross-thread phases are reported separately from the push/pop zones | Check `nvtx_startend_sum` as well as `nvtx_pushpop_sum` |
| No GPU zones | The scene ran on CPU | Confirm the scene uses GPU dynamics and that a CUDA device is available |
| Nsight shows CUDA but no NVTX rows | `nvtx` not in the trace selection | Include it: `-t nvtx,cuda` |

## Result

After this tutorial you can annotate any ovphysx workload and read its ovphysx and PhysX phases in Nsight Systems alongside CUDA activity, without rebuilding the library.
