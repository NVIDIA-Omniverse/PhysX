---
name: tensor-bindings-gpu
description: Read and write physics simulation data on GPU using CUDA device pointers and DLPack tensors. Use when you need GPU-to-GPU tensor exchange (no CPU staging) for high-throughput RL or robotics workloads.
compatibility: "ovphysx >=0.5.1 GPU mode with CUDA device memory; C examples require the SDK headers, libraries, CUDA runtime, and an NVIDIA GPU; Python examples require PyTorch with CUDA."
allowed-tools: Read Shell
metadata:
  version: "0.1.0"
  author: NVIDIA Omniverse Physics
  tags: "ovphysx, physics, tensor-bindings, gpu, cuda"
---

# Tensor Bindings: GPU Read and Write

GPU tensor bindings use GPU-mode PhysX with DLPack CUDA tensors.
DirectGPU (`/physics/suppressReadback`) is a separate setting: keep it enabled for fastest tensor-pipeline workloads and disabled for workflows that need contact modification.
ovstage is used to populate the scene and resolve prim path patterns, but the tensor data path does not go through ovstage.

## When to Use

Use this skill when a caller needs GPU-to-GPU tensor exchange through CUDA device pointers, PyTorch CUDA tensors, or DLPack without CPU staging.

## Instructions

1. Read the full C or Python sample before adapting this pattern because CUDA memory lifetime, DLPack shape storage, and device ordinal handling matter.
2. Allocate CUDA device memory before wrapping it in `DLTensor`, derive `cuda_device_id` from `active_cuda_gpus`, and keep shape storage valid until the synchronous read or write returns.
3. Use Shell to compile and run the full sample or a local integration test after adapting the scene path and tensor type.

## C sample (CUDA)

The GPU tensor bindings sample uses `cuda_runtime.h` for device memory allocation and wraps those device pointers in DLPack `DLTensor` structs for:
- `ovphysx_read_tensor_binding()`
- `ovphysx_write_tensor_binding()`

Full sample:
- `samples/c_samples/tensor_bindings_gpu_c/main.c` (SDK)
- Source checkout: `tests/c_samples/tensor_bindings_gpu_c/main.c`

Minimal DLPack wrapper pattern:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/dlpack/dlpack.h>
#include <stddef.h>
#include <stdint.h>

static DLTensor make_cuda_tensor_f32_2d(
    // CUDA device memory, for example from cudaMalloc(); caller frees it.
    void* device_buffer,
    int64_t rows,
    int64_t columns,
    // Caller-owned storage; tensor.shape points here until the tensor is consumed.
    int64_t shape_storage[2],
    // First CUDA ordinal from the active_cuda_gpus create-args string.
    int32_t cuda_device_id)
{
    shape_storage[0] = rows;
    shape_storage[1] = columns;

    DLTensor tensor = {
        .data = device_buffer,
        .device = { kDLCUDA, cuda_device_id },
        .ndim = 2,
        .dtype = { kDLFloat, 32, 1 },
        .shape = shape_storage,
        .strides = NULL,
        .byte_offset = 0
    };
    return tensor;
}

static int read_and_write_gpu_tensor(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    // CUDA device memory, for example from cudaMalloc(); caller frees it.
    void* device_buffer,
    int64_t count,
    int64_t components,
    // Derive from the active_cuda_gpus string used to create the instance.
    int32_t cuda_device_id)
{
    int64_t shape_storage[2];
    DLTensor tensor = make_cuda_tensor_f32_2d(
        device_buffer,
        count,
        components,
        shape_storage,
        cuda_device_id);

    // Check every result; GPU I/O failures surface through the return status.
    if (ovphysx_read_tensor_binding(handle, binding, &tensor).status != OVPHYSX_API_SUCCESS) {
        return 0;
    }
    // NULL means no index tensor: apply a full, unindexed write.
    if (ovphysx_write_tensor_binding(handle, binding, &tensor, NULL).status != OVPHYSX_API_SUCCESS) {
        return 0;
    }
    return 1;
}
```

Use the first CUDA ordinal in `active_cuda_gpus` as the `cuda_device_id`.
Default (empty) means GPU 0; `"1"` means GPU 1; `"0,1"` means primary ordinal 0.
Read this from the same `ovphysx_create_args.active_cuda_gpus` string you used when creating the instance; the full C sample shows the parsing code.
This helper wraps an existing CUDA allocation; it does not allocate memory.
`device_buffer`, `shape_storage`, and optional `strides` storage must stay valid until the synchronous read or write call returns.
The snippet uses `strides = NULL` for C-contiguous tensors, matching the ovphysx C samples.

## Python with PyTorch

```python
import torch
from ovphysx import PhysX, PhysXConfig
from ovphysx.types import TensorType
import ovstage

physx = PhysX(
    config=PhysXConfig(
        carbonite_overrides={"/physics/suppressReadback": True},
    ),
)
stage = ovstage.Stage("ovphysx-gpu-tensors")
ovstage.population.open_usd(stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
physx.attach_ovstage(stage, read_ordinal=1)

binding = physx.create_tensor_binding(
    pattern="/World/envs/env*/box",
    tensor_type=TensorType.RIGID_BODY_POSE,
)

# Write poses from a PyTorch CUDA tensor via DLPack (e.g. an RL env reset).
# The pose layout is [px, py, pz, qx, qy, qz, qw]; qw = 1 is identity rotation.
reset_poses = torch.zeros(binding.shape, dtype=torch.float32, device="cuda")
reset_poses[..., 6] = 1.0
binding.write(reset_poses)

# step_sync steps and waits in one call
physx.step_sync(1.0 / 60.0)

# Read simulated poses back into a CUDA tensor (no CPU staging)
output = torch.zeros(binding.shape, dtype=torch.float32, device="cuda")
binding.read(output)

binding.destroy()
physx.detach_ovstage()
stage.destroy()
physx.release()
```

## Important notes

- The physics-only `domains` mask above is fine for this skill's non-instanced sample USD. For arbitrary content prefer `ALL` -- see `docs/ovstage_integration.md` ("Population domains").
- GPU dynamics are enabled by authoring `physxScene:enableGPUDynamics=true` in the USD stage (PhysX reads this). To prevent any GPU usage in ovphysx, call `PhysX.set_cpu_mode(True)` before creating any `PhysX` instance. Process-wide CPU-only mode rejects CUDA DLPack tensors before accessing CUDA.
- DLPack interop works with PyTorch and other objects implementing the `__dlpack__` protocol.
- Masked writes are supported on GPU as well via `ovphysx_write_tensor_binding_masked()` / `binding.write(..., mask=...)`.
- Create bindings once outside simulation loops and reuse them; binding creation allocates native TensorAPI resources.

## References

- Docs: `docs/tutorials/tensor_bindings.md` (tensor bindings tutorial, shared with the CPU skill); `docs/developer_guide.md` for GPU/DirectGPU specifics
- C sample: `samples/c_samples/tensor_bindings_gpu_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_gpu_c/main.c`)
