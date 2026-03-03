// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Compile-time check: fail compilation if C++ compiler is used
#ifdef __cplusplus
#error "This file must be compiled as C, not C++"
#endif

#define NUM_LINKS 15
#define NUM_JOINTS 14

typedef struct TensorBuffer {
    DLTensor tensor;
    void* data;
    int64_t* shape;
} TensorBuffer;

static int check_result(ovphysx_result_t result, const char* context) {
    if (result.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ERROR in %s: ", context);
        if (result.error.ptr && result.error.length > 0) {
            fprintf(stderr, "%.*s\n", (int)result.error.length, result.error.ptr);
            ovphysx_destroy_error(result.error);
        } else {
            fprintf(stderr, "status=%d\n", (int)result.status);
        }
        return 0;
    }
    return 1;
}

static void destroy_tensor(TensorBuffer* t) {
    if (!t) {
        return;
    }
    free(t->data);
    free(t->shape);
    t->data = NULL;
    t->shape = NULL;
}

static TensorBuffer make_tensor_f32_2d(size_t rows, size_t cols) {
    TensorBuffer t;
    memset(&t, 0, sizeof(TensorBuffer));
    t.data = calloc(rows * cols, sizeof(float));
    t.shape = (int64_t*)malloc(sizeof(int64_t) * 2);
    t.shape[0] = (int64_t)rows;
    t.shape[1] = (int64_t)cols;

    t.tensor.data = t.data;
    t.tensor.ndim = 2;
    t.tensor.shape = t.shape;
    t.tensor.strides = NULL;
    t.tensor.byte_offset = 0;
    t.tensor.dtype.code = kDLFloat;
    t.tensor.dtype.bits = 32;
    t.tensor.dtype.lanes = 1;
    t.tensor.device.device_type = kDLCPU;
    t.tensor.device.device_id = 0;
    return t;
}

static TensorBuffer make_tensor_f32_3d(size_t dim0, size_t dim1, size_t dim2) {
    TensorBuffer t;
    memset(&t, 0, sizeof(TensorBuffer));
    t.data = calloc(dim0 * dim1 * dim2, sizeof(float));
    t.shape = (int64_t*)malloc(sizeof(int64_t) * 3);
    t.shape[0] = (int64_t)dim0;
    t.shape[1] = (int64_t)dim1;
    t.shape[2] = (int64_t)dim2;

    t.tensor.data = t.data;
    t.tensor.ndim = 3;
    t.tensor.shape = t.shape;
    t.tensor.strides = NULL;
    t.tensor.byte_offset = 0;
    t.tensor.dtype.code = kDLFloat;
    t.tensor.dtype.bits = 32;
    t.tensor.dtype.lanes = 1;
    t.tensor.device.device_type = kDLCPU;
    t.tensor.device.device_id = 0;
    return t;
}

static int wait_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index, const char* context) {
    ovphysx_op_wait_result_t wait_result = {0};
    ovphysx_result_t result = ovphysx_wait_op(handle, op_index, 10ULL * 1000 * 1000 * 1000, &wait_result);

    if (wait_result.num_errors > 0) {
        fprintf(stderr, "ERROR in %s: async operation failed\n", context);
        ovphysx_destroy_errors(wait_result.errors, wait_result.num_errors);
        return 0;
    }
    if (result.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ERROR in %s: wait failed (status=%d)\n", context, (int)result.status);
        if (result.error.ptr)
            ovphysx_destroy_error(result.error);
        return 0;
    }
    return 1;
}

int main(void) {
    printf("=== ovphysx Articulation Control (C API - Tensor Binding) ===\n");

    // 1. Create instance
    ovphysx_handle_t handle = 0;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.device = OVPHYSX_DEVICE_CPU;

    ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    if (!check_result(result, "create_instance")) {
        return 1;
    }

    printf("Instance created.\n");

    // 2. Load USD scene
    ovphysx_usd_handle_t usd_handle = 0;
    ovphysx_enqueue_result_t add_result = ovphysx_add_usd(
        handle,
        OVPHYSX_LITERAL(OVPHYSX_TEST_DATA "/links_chain_sample.usda"),
        OVPHYSX_LITERAL(""),
        &usd_handle);

    if (add_result.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to load USD scene\n");
        if (add_result.error.ptr && add_result.error.length > 0) {
            fprintf(stderr, "ERROR in add_usd enqueue: %.*s\n", (int)add_result.error.length, add_result.error.ptr);
            ovphysx_destroy_error(add_result.error);
        }
        ovphysx_destroy_instance(handle);
        return 1;
    }

    if (!wait_op(handle, add_result.op_index, "add_usd")) {
        fprintf(stderr, "Failed to load USD scene\n");
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("USD scene loaded.\n");

    // 3. Create tensor bindings
    // 3a. DOF velocity target binding (write control targets)
    ovphysx_tensor_binding_handle_t dof_target_binding = 0;
    ovphysx_tensor_binding_desc_t dof_target_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation"),
        .tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32
    };

    result = ovphysx_create_tensor_binding(handle, &dof_target_desc, &dof_target_binding);
    if (!check_result(result, "create DOF target binding")) {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 3b. Articulation link pose binding
    ovphysx_tensor_binding_handle_t link_pose_binding = 0;
    ovphysx_tensor_binding_desc_t link_pose_desc = {
        .pattern = OVPHYSX_LITERAL("/World/articulation"),
        .tensor_type = OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32
    };

    result = ovphysx_create_tensor_binding(handle, &link_pose_desc, &link_pose_binding);
    if (!check_result(result, "create articulation link pose binding")) {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("Tensor bindings created.\n");

    // 4. Query binding specs and allocate tensors
    ovphysx_tensor_spec_t dof_spec, link_pose_spec;

    result = ovphysx_get_tensor_binding_spec(handle, dof_target_binding, &dof_spec);
    if (!check_result(result, "get_tensor_binding_spec (dof target)")) {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    result = ovphysx_get_tensor_binding_spec(handle, link_pose_binding, &link_pose_spec);
    if (!check_result(result, "get_tensor_binding_spec (link pose)")) {
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("\nBinding specs:\n");
    printf("  Articulation DOFs: shape=[%lld, %lld], ndim=%d\n",
           (long long)dof_spec.shape[0], (long long)dof_spec.shape[1], dof_spec.ndim);
    printf("  Articulation link poses: shape=[%lld, %lld, %lld], ndim=%d\n",
           (long long)link_pose_spec.shape[0],
           (long long)link_pose_spec.shape[1],
           (long long)link_pose_spec.shape[2],
           link_pose_spec.ndim);

    // Allocate CPU tensors
    const size_t dof_count = (size_t)dof_spec.shape[0];
    const size_t dof_components = (size_t)dof_spec.shape[1];
    const size_t link_pose_batch = (size_t)link_pose_spec.shape[0];
    const size_t link_count = (size_t)link_pose_spec.shape[1];
    const size_t link_pose_components = (size_t)link_pose_spec.shape[2];

    TensorBuffer dof_target_tensor = make_tensor_f32_2d(dof_count, dof_components);
    TensorBuffer link_pose_tensor = make_tensor_f32_3d(link_pose_batch, link_count, link_pose_components);

    // 5. Set initial DOF velocity targets and simulate
    printf("\n=== Setting initial DOF velocity targets ===\n");

    // Initialize all targets to 0.0
    float* dof_target_data = (float*)dof_target_tensor.data;
    for (size_t i = 0; i < dof_count * dof_components; i++) {
        dof_target_data[i] = 0.0f;
    }
    printf("\n=== Writing initial DOF velocity targets ===\n");

    result = ovphysx_write_tensor_binding(handle, dof_target_binding, &dof_target_tensor.tensor, NULL);
    if (!check_result(result, "write initial DOF targets")) {
        destroy_tensor(&dof_target_tensor);
        destroy_tensor(&link_pose_tensor);
        ovphysx_destroy_instance(handle);
        return 1;
    }

    // 6. Simulation loop
    const float dt = 1.0f / 60.0f;
    float sim_time = 0.0f;
    const size_t link_index_to_print = (link_count > 0) ? (link_count - 1) : 0;

    printf("Running 120 simulation steps...\n");
    for (int step = 0; step < 120; ++step) {
        // Update DOF targets every 50 steps
        if (step % 50 == 0) {
            // Alternate between positive and negative target velocities
            float target_vel = ((step / 50) % 2 == 0) ? 50.0f : -50.0f;
            for (size_t i = 0; i < dof_count * dof_components; ++i) {
                // Alternate direction for each DOF
                dof_target_data[i] = (i % 2 == 0) ? target_vel : -target_vel;
            }

            result = ovphysx_write_tensor_binding(handle, dof_target_binding, &dof_target_tensor.tensor, NULL);
            if (!check_result(result, "write DOF targets")) {
                destroy_tensor(&dof_target_tensor);
                destroy_tensor(&link_pose_tensor);
                ovphysx_destroy_instance(handle);
                return 1;
            }
        }

        // Step simulation
        ovphysx_enqueue_result_t step_result = ovphysx_step(handle, dt, sim_time);
        if (step_result.status != OVPHYSX_API_SUCCESS) {
            fprintf(stderr, "ERROR in step enqueue (status=%d)\n", (int)step_result.status);
            if (step_result.error.ptr && step_result.error.length > 0) {
                fprintf(stderr, "  %.*s\n", (int)step_result.error.length, step_result.error.ptr);
                ovphysx_destroy_error(step_result.error);
            }
            destroy_tensor(&dof_target_tensor);
            destroy_tensor(&link_pose_tensor);
            ovphysx_destroy_instance(handle);
            return 1;
        }
        if (step_result.error.ptr)
            ovphysx_destroy_error(step_result.error);
        if (!wait_op(handle, step_result.op_index, "step")) {
            destroy_tensor(&dof_target_tensor);
            destroy_tensor(&link_pose_tensor);
            ovphysx_destroy_instance(handle);
            return 1;
        }
        sim_time += dt;

        // Read and print state every 30 steps
        if (step % 30 == 0) {
            // Read articulation link poses
            result = ovphysx_read_tensor_binding(handle, link_pose_binding, &link_pose_tensor.tensor);
            if (!check_result(result, "read articulation link poses")) {
                destroy_tensor(&dof_target_tensor);
                destroy_tensor(&link_pose_tensor);
                ovphysx_destroy_instance(handle);
                return 1;
            }

            const float* link_pose_data = (const float*)link_pose_tensor.data;

            size_t articulation_index = 0;
            size_t link_pose_offset = (articulation_index * link_count + link_index_to_print) * link_pose_components;
            printf("Step %3d | Link %zu pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n",
                   step,
                   link_index_to_print,
                   link_pose_data[link_pose_offset + 0],
                   link_pose_data[link_pose_offset + 1],
                   link_pose_data[link_pose_offset + 2],
                   link_pose_data[link_pose_offset + 3],
                   link_pose_data[link_pose_offset + 4],
                   link_pose_data[link_pose_offset + 5],
                   link_pose_data[link_pose_offset + 6]);

        }
    }

    // Cleanup
    printf("\n=== Cleanup ===\n");

    destroy_tensor(&dof_target_tensor);
    destroy_tensor(&link_pose_tensor);

    ovphysx_destroy_tensor_binding(handle, dof_target_binding);
    ovphysx_destroy_tensor_binding(handle, link_pose_binding);

    ovphysx_destroy_instance(handle);

    printf("=== Articulation control sample completed successfully ===\n");
    return 0;
}
