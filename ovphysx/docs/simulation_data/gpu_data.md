<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Data on the GPU

When ovphysx simulates on a GPU, your program can get the results in CPU memory or keep them in
GPU memory. This page shows where read results are and how to pass them to NumPy or PyTorch without
copying them. It also shows when to keep them on the GPU and how to replay a whole GPU loop with a
CUDA graph.

![Two rows compare the modes over three steps. In the default mode, each step's positions are copied from GPU memory to CPU memory, and the read returns an array whose device is cpu, which NumPy uses. In DirectGPU mode, there is no copy to CPU memory: the read returns an array in GPU memory, whose device is cuda:0, which PyTorch uses.](images/guide_gpu_data_memory.png)

## Where Results Are

[CPU and GPU](../getting_started/concepts.md#cpu-and-gpu) lists the three modes. They differ in
where your program finds the results:

- In the **default mode** with an NVIDIA GPU, physics runs on the GPU, and ovphysx copies most
  results to CPU memory after every step, ready for NumPy.
- In **DirectGPU mode**, simulated values such as positions stay in GPU memory, ready for code
  that runs on the GPU, such as a PyTorch network.
- On the **CPU**, physics and results both stay in CPU memory.

The copy takes time every step. DirectGPU avoids it.

## Check Where an Array Is

Every array has a `device`: `cpu` for CPU memory, `cuda:0` for the first GPU. Some arrays stay in
GPU memory even in the default mode, such as the `points` of soft bodies. In DirectGPU mode, one
read can return arrays in both places. Check each array's `device`, in reads and writes alike,
before you use it.
[Device Residency and Interop](../read_write/device.md#where-the-data-lives-cpu-vs-gpu) lists
where each attribute is.

## Pass Arrays On Without Copying

Reads return [NVIDIA Warp arrays](../getting_started/concepts.md#using-the-results), which other libraries can use without a copy when they work with
memory on that device:

| To Use an Array In | Call | Copies the Data? |
| --- | --- | --- |
| NumPy | `array.numpy()` | No for an array in CPU memory. An array in GPU memory is copied to CPU memory. |
| PyTorch | `warp.to_torch(array)` | No. The tensor uses the same memory, on the CPU or the GPU. |
| Another library that supports [DLPack](https://dmlc.github.io/dlpack/latest/), the standard for sharing arrays | That library's `from_dlpack(array)` | No. |

When nothing is copied, the result shares the read's memory: finish with it, or copy it, before
the read ends, as [Using the Results](../getting_started/concepts.md#using-the-results) explains.

## Turn On DirectGPU

Use DirectGPU when your program works on the results on the GPU every step, as a PyTorch training
loop does. It has no dedicated call yet: `create_physx` in the
[CUDA graph excerpt](#replay-a-loop-with-a-cuda-graph) turns it on. The switch applies to
the whole process and is read when a scene is attached, so turn it on before you attach.

## What Changes in DirectGPU Mode

- **Step first.** Before the first step, reads of bodies and articulations return no data, and
  writes of values in GPU memory fail.
- **GPU work needs synchronization.** Before PyTorch or your own GPU code uses a read array, or
  fills a write array, synchronize as
  [CUDA Synchronization](../read_write/device.md#cuda-synchronization) shows.
- **Objects never sleep.** DirectGPU turns [sleeping](../building_scenes/physics_scene.md#settings-you-can-change) off, so resting objects are still simulated.
- **Some features are missing.**
  [Device Residency and Interop](../read_write/device.md#where-the-data-lives-cpu-vs-gpu) lists
  them.

## Replay a Loop With a CUDA Graph

A training loop repeats the same GPU work at every step: it reads the state, runs the policy
network, and writes the actions. Launched one by one, every read, kernel, and write costs CPU
time, and the GPU waits between them. A **CUDA graph** records that GPU work once and replays it
with one launch, between physics steps:

![Two schematic timelines of two control steps on the GPU. In the first, the program launches the read, four policy kernels, and the write one at a time, with a launch and wait before each, then the physics step. In the second, one CUDA graph launch replays the read, the policy, and the write back to back, so the same work finishes sooner. Block lengths are not measured.](images/guide_gpu_data_cuda_graph.png)

You need an NVIDIA driver for CUDA 12.5 or newer and a `PhysX` object in DirectGPU mode on GPU 0,
so that physics and Warp share one CUDA context. `create_physx` in this excerpt makes one
through `PhysXConfig`. Python can warn that the `config` argument is deprecated. A small Warp
kernel stands in for the policy and holds 1 kg
bodies up:

```{literalinclude} code/gpu_data_cuda_graph.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

These rules matter most:

- **What you can capture:** reads and writes of values in GPU memory, and your own GPU work.
  Values in CPU memory, contact data, and physics steps stay outside the graph.
- **Before the capture:** keep a [query](../reference/glossary.md#reading-and-writing), read once with the same attributes and close that read,
  open the write and fetch its [group](../reference/glossary.md#reading-and-writing), load your kernels, and call `wait_all`.
- **While you replay:** keep the query, the captured read, and the write open. Each replay must
  end before the next step, and each step before the next replay.
- **Capture again** after objects are added or removed, a rigid body is enabled or disabled, or
  the whole scene is reset or replaced. An old graph can write to the wrong bodies without any
  error.
- **Articulations:** if the graph reads center-of-mass values, call
  `update_articulations_kinematic` on the stream you replay on after you write joint positions or
  root poses outside the graph. An episode reset in a training loop is a typical case.

[Capturing a Read in a CUDA Graph](../read_write/device.md#capturing-a-read-in-a-cuda-graph) and
[Capturing a Write Step](../read_write/device.md#capturing-a-write-step-in-a-cuda-graph) list
every rule.

## More Detail

- [Device Residency and Interop](../read_write/device.md) lists where every value lives and covers
  DLPack ownership, CUDA synchronization, and CUDA graphs.
- [Step-First Precondition](../read_write/data_model.md#step-first-precondition-directgpu)
  explains reads and writes before the first step.
- [Fast Simulations](../common_tasks/fast_simulations.md) has more ways to save time.
- Next: [Changing the Scene](../common_tasks/changing_the_scene.md) adds, removes, and changes
  objects while the simulation runs.
