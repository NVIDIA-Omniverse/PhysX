// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "ovphysx/ovphysx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef OVPHYSX_HAS_CUDA
#include <cuda_runtime.h>
#endif

// Helper to check and report errors
static int check_result(ovphysx_result_t result, const char* context)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "ERROR in %s: ", context);
        if (result.error.ptr && result.error.length > 0)
        {
            fprintf(stderr, "%.*s\n", (int)result.error.length, result.error.ptr);
            ovphysx_destroy_error(result.error);
        }
        else
        {
            fprintf(stderr, "status=%d\n", (int)result.status);
        }
        return 0;
    }
    return 1;
}

// Helper to wait for async ops
static int wait_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index, const char* context)
{
    ovphysx_op_wait_result_t wait_result = { 0 };
    ovphysx_result_t result = ovphysx_wait_op(handle, op_index, 10ull * 1000ull * 1000ull * 1000ull, &wait_result);
    
    if (wait_result.num_errors > 0)
    {
        fprintf(stderr, "ERROR in %s: async operation failed\n", context);
        ovphysx_destroy_errors(wait_result.errors, wait_result.num_errors);
        return 0;
    }
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "ERROR in %s: wait failed (status=%d)\n", context, (int)result.status);
        if (result.error.ptr)
            ovphysx_destroy_error(result.error);
        return 0;
    }
    return 1;
}

int main(void)
{
#ifndef OVPHYSX_HAS_CUDA
    printf("CUDA toolkit not found at build time. Skipping GPU sample.\n");
    return 0;
#else
    printf("=== Tensor Binding API Sample ===\n\n");

    // 1. Create instance with GPU mode (default)
    // GPU mode automatically enables DirectGPU API required for TensorBinding
    ovphysx_handle_t handle = 0;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    // args.device = OVPHYSX_DEVICE_GPU;  // default, enables TensorBinding support
    //
    // Optional: explicitly select which GPU PhysX should use.
    // This sets the Carbonite setting `/physics/cudaDevice` BEFORE PhysX plugins load.
    //
    // Example (uncomment to use):
    //   args.device = OVPHYSX_DEVICE_GPU;
    //   args.gpu_index = 1; // run PhysX on CUDA device 1
    //
    // Note: if you want to override via settings instead (higher precedence), you can pass:
    //   args.settings_keys / args.settings_values with key "/physics/cudaDevice".

    ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    if (!check_result(result, "create_instance"))
        return 1;

    printf("Instance created (GPU mode).\n");

    // 2. Load USD scene
    ovphysx_usd_handle_t usd_handle = 0;
    ovphysx_enqueue_result_t add_result = ovphysx_add_usd(
        handle,
        OVPHYSX_LITERAL(OVPHYSX_TEST_DATA "/links_chain_sample_gpu.usda"),
        OVPHYSX_LITERAL(""),  // empty prefix
        &usd_handle);

    if (add_result.status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "Failed to load USD scene\n");
        if (add_result.error.ptr && add_result.error.length > 0)
        {
            fprintf(stderr, "ERROR in add_usd enqueue: %.*s\n", (int)add_result.error.length, add_result.error.ptr);
            ovphysx_destroy_error(add_result.error);
        }
        ovphysx_destroy_instance(handle);
        return 1;
    }
    if (add_result.error.ptr)
        ovphysx_destroy_error(add_result.error);

    if (!wait_op(handle, add_result.op_index, "add_usd"))
    {
        fprintf(stderr, "Failed to load USD scene\n");
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("USD scene loaded.\n");

    // 3. Create tensor bindings
    //    - Rigid body poses for articulation links
    //    - Articulation DOF positions (read actual state)
    //    - Articulation DOF position targets (write control targets)
    //    - Articulation DOF velocities (read actual velocities)

    // 3a. Rigid body pose binding for link transforms
    ovphysx_tensor_binding_handle_t rb_binding = 0;
    ovphysx_tensor_binding_desc_t rb_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation/articulationLink*"),
        .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
    };

    result = ovphysx_create_tensor_binding(handle, &rb_desc, &rb_binding);
    if (!check_result(result, "create rigid body binding"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 3b. DOF position binding (read joint positions)
    ovphysx_tensor_binding_handle_t dof_pos_binding = 0;
    ovphysx_tensor_binding_desc_t dof_pos_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation"),
        .tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32
    };

    result = ovphysx_create_tensor_binding(handle, &dof_pos_desc, &dof_pos_binding);
    if (!check_result(result, "create DOF position binding"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 3c. DOF position target binding (write control targets)
    ovphysx_tensor_binding_handle_t dof_target_binding = 0;
    ovphysx_tensor_binding_desc_t dof_target_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation"),
        .tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32
    };

    result = ovphysx_create_tensor_binding(handle, &dof_target_desc, &dof_target_binding);
    if (!check_result(result, "create DOF target binding"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 3d. DOF velocity binding (read joint velocities)
    ovphysx_tensor_binding_handle_t dof_vel_binding = 0;
    ovphysx_tensor_binding_desc_t dof_vel_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation"),
        .tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32
    };

    result = ovphysx_create_tensor_binding(handle, &dof_vel_desc, &dof_vel_binding);
    if (!check_result(result, "create DOF velocity binding"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("Tensor bindings created.\n");

    // 4. Query binding specs and allocate GPU tensors
    ovphysx_tensor_spec_t rb_spec, dof_spec;
    
    result = ovphysx_get_tensor_binding_spec(handle, rb_binding, &rb_spec);
    if (!check_result(result, "get_tensor_binding_spec (rb)"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    result = ovphysx_get_tensor_binding_spec(handle, dof_pos_binding, &dof_spec);
    if (!check_result(result, "get_tensor_binding_spec (dof)"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("\nBinding specs:\n");
    printf("  Rigid bodies: shape=[%lld, %lld], ndim=%d\n",
           (long long)rb_spec.shape[0], (long long)rb_spec.shape[1], rb_spec.ndim);
    printf("  Articulation DOFs: shape=[%lld, %lld], ndim=%d\n",
           (long long)dof_spec.shape[0], (long long)dof_spec.shape[1], dof_spec.ndim);

    // Allocate GPU buffers
    const size_t rb_count = (size_t)rb_spec.shape[0];
    const size_t rb_components = (size_t)rb_spec.shape[1];
    const size_t dof_count = (size_t)dof_spec.shape[0];
    const size_t dof_components = (size_t)dof_spec.shape[1];

    float* rb_device = NULL;
    float* dof_pos_device = NULL;
    float* dof_target_device = NULL;
    float* dof_vel_device = NULL;

    cudaMalloc((void**)&rb_device, rb_count * rb_components * sizeof(float));
    cudaMalloc((void**)&dof_pos_device, dof_count * dof_components * sizeof(float));
    cudaMalloc((void**)&dof_target_device, dof_count * dof_components * sizeof(float));
    cudaMalloc((void**)&dof_vel_device, dof_count * dof_components * sizeof(float));

    if (!rb_device || !dof_pos_device || !dof_target_device || !dof_vel_device) {
        printf("CUDA allocation failed\n");
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // Create DLTensor wrappers - use args.gpu_index to match PhysX device
    int64_t rb_shape[2] = { (int64_t)rb_count, (int64_t)rb_components };
    int64_t dof_shape[2] = { (int64_t)dof_count, (int64_t)dof_components };
    int32_t device_id = args.gpu_index;

    DLTensor rb_tensor = {
        .data = rb_device,
        .device = { kDLCUDA, device_id },
        .ndim = 2,
        .dtype = { kDLFloat, 32, 1 },
        .shape = rb_shape,
        .strides = NULL,
        .byte_offset = 0
    };

    DLTensor dof_pos_tensor = {
        .data = dof_pos_device,
        .device = { kDLCUDA, device_id },
        .ndim = 2,
        .dtype = { kDLFloat, 32, 1 },
        .shape = dof_shape,
        .strides = NULL,
        .byte_offset = 0
    };

    DLTensor dof_target_tensor = {
        .data = dof_target_device,
        .device = { kDLCUDA, device_id },
        .ndim = 2,
        .dtype = { kDLFloat, 32, 1 },
        .shape = dof_shape,
        .strides = NULL,
        .byte_offset = 0
    };

    DLTensor dof_vel_tensor = {
        .data = dof_vel_device,
        .device = { kDLCUDA, device_id },
        .ndim = 2,
        .dtype = { kDLFloat, 32, 1 },
        .shape = dof_shape,
        .strides = NULL,
        .byte_offset = 0
    };

    printf("GPU tensors allocated.\n");

    // 5. OPTIONAL: Explicit GPU warmup
    // GPU tensor reads trigger automatic warmup on first access. However, you can
    // call ovphysx_warmup_gpu() explicitly to control when the warmup latency occurs.
    // This is useful if you want to avoid a latency spike on the first tensor read.
    printf("\nPerforming explicit GPU warmup (optional - happens automatically on first read)...\n");
    
    result = ovphysx_warmup_gpu(handle);
    if (!check_result(result, "GPU warmup"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 6. Read initial state
    printf("\n=== Initial State ===\n");

    result = ovphysx_read_tensor_binding(handle, rb_binding, &rb_tensor);
    if (!check_result(result, "read rigid body poses"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    result = ovphysx_read_tensor_binding(handle, dof_pos_binding, &dof_pos_tensor);
    if (!check_result(result, "read DOF positions"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // Copy to host and print
    float* host_transforms = malloc(rb_count * rb_components * sizeof(float));
    float* host_dof_pos = malloc(dof_count * dof_components * sizeof(float));
    
    cudaMemcpy(host_transforms, rb_device, rb_count * rb_components * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(host_dof_pos, dof_pos_device, dof_count * dof_components * sizeof(float), cudaMemcpyDeviceToHost);

    printf("\nLink transforms (first 3):\n");
    for (size_t i = 0; i < rb_count && i < 3; i++)
    {
        float* t = &host_transforms[i * rb_components];
        printf("  Link %zu: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n",
               i, t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
    }

    printf("\nDOF positions (first articulation):\n");
    printf("  ");
    for (size_t d = 0; d < dof_components && d < 8; d++)
    {
        printf("%.3f ", host_dof_pos[d]);
    }
    if (dof_components > 8) printf("...");
    printf("\n");

    // 7. Set DOF targets and simulate
    printf("\n=== Setting DOF position targets to 0.3 rad ===\n");

    float* host_targets = malloc(dof_count * dof_components * sizeof(float));
    for (size_t i = 0; i < dof_count * dof_components; i++)
        host_targets[i] = 0.3f;  // Set all targets to 0.3 radians
    
    cudaMemcpy(dof_target_device, host_targets, dof_count * dof_components * sizeof(float), cudaMemcpyHostToDevice);

    result = ovphysx_write_tensor_binding(handle, dof_target_binding, &dof_target_tensor, NULL);
    if (!check_result(result, "write DOF targets"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 8. Simulation loop
    printf("Running 120 simulation steps...\n");
    for (int i = 0; i < 120; i++)
    {
        ovphysx_enqueue_result_t step_result = ovphysx_step(handle, 1.0f / 60.0f, (float)i / 60.0f);
        if (step_result.status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "ERROR in step enqueue (status=%d)\n", (int)step_result.status);
            if (step_result.error.ptr && step_result.error.length > 0)
            {
                fprintf(stderr, "  %.*s\n", (int)step_result.error.length, step_result.error.ptr);
                ovphysx_destroy_error(step_result.error);
            }
            ovphysx_destroy_instance(handle);
            return 1;
        }
        if (step_result.error.ptr)
            ovphysx_destroy_error(step_result.error);
        if (!wait_op(handle, step_result.op_index, "step"))
        {
            ovphysx_destroy_instance(handle);
            return 1;
        }
    }

    // 9. Read final state
    printf("\n=== Final State (after 120 steps) ===\n");

    result = ovphysx_read_tensor_binding(handle, rb_binding, &rb_tensor);
    if (!check_result(result, "read final rigid body poses"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    result = ovphysx_read_tensor_binding(handle, dof_pos_binding, &dof_pos_tensor);
    if (!check_result(result, "read final DOF positions"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    result = ovphysx_read_tensor_binding(handle, dof_vel_binding, &dof_vel_tensor);
    if (!check_result(result, "read final DOF velocities"))
    {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    float* host_dof_vel = malloc(dof_count * dof_components * sizeof(float));
    
    cudaMemcpy(host_transforms, rb_device, rb_count * rb_components * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(host_dof_pos, dof_pos_device, dof_count * dof_components * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(host_dof_vel, dof_vel_device, dof_count * dof_components * sizeof(float), cudaMemcpyDeviceToHost);

    printf("\nLink transforms (first 3):\n");
    for (size_t i = 0; i < rb_count && i < 3; i++)
    {
        float* t = &host_transforms[i * rb_components];
        printf("  Link %zu: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n",
               i, t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
    }

    printf("\nDOF positions (first articulation):\n");
    printf("  ");
    for (size_t d = 0; d < dof_components && d < 8; d++)
    {
        printf("%.3f ", host_dof_pos[d]);
    }
    if (dof_components > 8) printf("...");
    printf("\n");

    printf("\nDOF velocities (first articulation):\n");
    printf("  ");
    for (size_t d = 0; d < dof_components && d < 8; d++)
    {
        printf("%.3f ", host_dof_vel[d]);
    }
    if (dof_components > 8) printf("...");
    printf("\n");

    // Cleanup
    printf("\n=== Cleanup ===\n");

    free(host_transforms);
    free(host_dof_pos);
    free(host_dof_vel);
    free(host_targets);

    cudaFree(rb_device);
    cudaFree(dof_pos_device);
    cudaFree(dof_target_device);
    cudaFree(dof_vel_device);

    ovphysx_destroy_tensor_binding(handle, rb_binding);
    ovphysx_destroy_tensor_binding(handle, dof_pos_binding);
    ovphysx_destroy_tensor_binding(handle, dof_target_binding);
    ovphysx_destroy_tensor_binding(handle, dof_vel_binding);

    ovphysx_destroy_instance(handle);

    printf("\nTensor Binding sample completed successfully!\n");
    return 0;
#endif
}
