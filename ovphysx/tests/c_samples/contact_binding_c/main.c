// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NOTE: This file is included verbatim in documentation.
// When editing, keep tutorial line ranges in sync.

// [tutorial-start]

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
#error "This file must be compiled as C, not C++"
#endif

static int check_result(ovphysx_result_t r, const char* ctx)
{
    if (r.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ERROR in %s: ", ctx);
        ovphysx_string_t err = ovphysx_get_last_error();
        if (err.ptr && err.length > 0)
            fprintf(stderr, "%.*s\n", (int)err.length, err.ptr);
        else
            fprintf(stderr, "status=%d\n", (int)r.status);
        return 0;
    }
    return 1;
}

static int check_enqueue(ovphysx_enqueue_result_t r, const char* ctx)
{
    if (r.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ERROR in %s: ", ctx);
        ovphysx_string_t err = ovphysx_get_last_error();
        if (err.ptr && err.length > 0)
            fprintf(stderr, "%.*s\n", (int)err.length, err.ptr);
        else
            fprintf(stderr, "status=%d\n", (int)r.status);
        return 0;
    }
    return 1;
}

static int wait_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index, const char* ctx)
{
    ovphysx_op_wait_result_t wait_result = {0};
    ovphysx_result_t r = ovphysx_wait_op(handle, op_index, UINT64_MAX, &wait_result);
    int has_errors = (wait_result.num_errors > 0);
    ovphysx_destroy_wait_result(&wait_result);
    if (has_errors) {
        fprintf(stderr, "ERROR in %s: async operation failed\n", ctx);
        return 0;
    }
    if (r.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ERROR in %s: wait failed (status=%d)\n", ctx, (int)r.status);
        return 0;
    }
    return 1;
}

/* Allocate a 2-D float32 DLTensor on the CPU. Caller frees data and shape. */
static DLTensor make_tensor_f32_2d(size_t rows, size_t cols, float** out_data, int64_t** out_shape)
{
    DLTensor t;
    memset(&t, 0, sizeof(DLTensor));
    *out_data  = (float*)calloc(rows * cols, sizeof(float));
    *out_shape = (int64_t*)malloc(2 * sizeof(int64_t));
    (*out_shape)[0] = (int64_t)rows;
    (*out_shape)[1] = (int64_t)cols;
    t.data         = *out_data;
    t.ndim         = 2;
    t.shape        = *out_shape;
    t.strides      = NULL;
    t.byte_offset  = 0;
    t.dtype.code   = kDLFloat;
    t.dtype.bits   = 32;
    t.dtype.lanes  = 1;
    t.device.device_type = kDLCPU;
    t.device.device_id   = 0;
    return t;
}

/* Allocate a 3-D float32 DLTensor on the CPU. Caller frees data and shape. */
static DLTensor make_tensor_f32_3d(size_t d0, size_t d1, size_t d2,
                                   float** out_data, int64_t** out_shape)
{
    DLTensor t;
    memset(&t, 0, sizeof(DLTensor));
    *out_data  = (float*)calloc(d0 * d1 * d2, sizeof(float));
    *out_shape = (int64_t*)malloc(3 * sizeof(int64_t));
    (*out_shape)[0] = (int64_t)d0;
    (*out_shape)[1] = (int64_t)d1;
    (*out_shape)[2] = (int64_t)d2;
    t.data         = *out_data;
    t.ndim         = 3;
    t.shape        = *out_shape;
    t.strides      = NULL;
    t.byte_offset  = 0;
    t.dtype.code   = kDLFloat;
    t.dtype.bits   = 32;
    t.dtype.lanes  = 1;
    t.device.device_type = kDLCPU;
    t.device.device_id   = 0;
    return t;
}

int main(void)
{
    ovphysx_result_t r;
    ovphysx_enqueue_result_t er;

    /* 1. Initialize SDK */
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.device = OVPHYSX_DEVICE_CPU;

    ovphysx_handle_t handle = 0;
    r = ovphysx_create_instance(&args, &handle);
    if (!check_result(r, "ovphysx_create_instance")) return 1;

    /* 2. Load scene */
    ovphysx_usd_handle_t usd_handle = 0;
    er = ovphysx_add_usd(handle,
                         ovphysx_cstr(OVPHYSX_TEST_DATA "/boxes_falling_on_groundplane.usda"),
                         (ovphysx_string_t){NULL, 0}, &usd_handle);
    if (!check_enqueue(er, "ovphysx_add_usd")) { ovphysx_destroy_instance(handle); return 1; }
    if (!wait_op(handle, er.op_index, "add_usd")) { ovphysx_destroy_instance(handle); return 1; }

    /* 3. Create contact binding BEFORE the first step.
     *    sensor: the falling box.  filter: the ground plane. */
    ovphysx_string_t sensors[1];
    sensors[0] = ovphysx_cstr("/World/Cube1");

    ovphysx_string_t filters[1];
    filters[0] = ovphysx_cstr("/World/GroundPlane/CollisionMesh");

    ovphysx_contact_binding_handle_t cb = 0;
    r = ovphysx_create_contact_binding(
        handle,
        sensors, 1,     /* 1 sensor pattern */
        filters, 1,     /* 1 filter pattern per sensor */
        256,            /* max raw contact pairs */
        &cb);
    if (!check_result(r, "ovphysx_create_contact_binding")) {
        ovphysx_destroy_instance(handle); return 1;
    }

    /* 4. Query matched sensor / filter counts */
    int32_t sensor_count = 0, filter_count = 0;
    r = ovphysx_get_contact_binding_spec(handle, cb, &sensor_count, &filter_count);
    if (!check_result(r, "ovphysx_get_contact_binding_spec")) {
        ovphysx_destroy_contact_binding(handle, cb);
        ovphysx_destroy_instance(handle);
        return 1;
    }
    printf("Sensors: %d  Filters per sensor: %d\n", sensor_count, filter_count);

    /* 5. Simulate until the box lands */
    for (int i = 0; i < 120; i++) {
        er = ovphysx_step(handle, 1.0f / 60.0f, 0.0f);
        if (!check_enqueue(er, "ovphysx_step")) {
            ovphysx_destroy_contact_binding(handle, cb);
            ovphysx_destroy_instance(handle);
            return 1;
        }
    }
    if (!wait_op(handle, er.op_index, "step")) {
        ovphysx_destroy_contact_binding(handle, cb);
        ovphysx_destroy_instance(handle);
        return 1;
    }

    /* 6. Read net contact forces: shape [S, 3].
     *    dt is taken automatically from the last ovphysx_step() call. */
    float* net_data   = NULL;
    int64_t* net_shp  = NULL;
    DLTensor net_tensor = make_tensor_f32_2d(
        (size_t)sensor_count, 3, &net_data, &net_shp);

    r = ovphysx_read_contact_net_forces(handle, cb, &net_tensor);
    if (!check_result(r, "ovphysx_read_contact_net_forces")) {
        free(net_data); free(net_shp);
        ovphysx_destroy_contact_binding(handle, cb);
        ovphysx_destroy_instance(handle);
        return 1;
    }
    printf("Net contact forces [%d, 3]:\n", sensor_count);
    for (int s = 0; s < sensor_count; s++) {
        printf("  sensor %d: fx=%.3f  fy=%.3f  fz=%.3f\n",
               s,
               net_data[s * 3 + 0],
               net_data[s * 3 + 1],
               net_data[s * 3 + 2]);
    }
    free(net_data); free(net_shp);

    /* 7. Read contact force matrix: shape [S, F, 3]. */
    float* mat_data   = NULL;
    int64_t* mat_shp  = NULL;
    DLTensor mat_tensor = make_tensor_f32_3d(
        (size_t)sensor_count, (size_t)filter_count, 3,
        &mat_data, &mat_shp);

    r = ovphysx_read_contact_force_matrix(handle, cb, &mat_tensor);
    if (!check_result(r, "ovphysx_read_contact_force_matrix")) {
        free(mat_data); free(mat_shp);
        ovphysx_destroy_contact_binding(handle, cb);
        ovphysx_destroy_instance(handle);
        return 1;
    }
    printf("Contact force matrix [%d, %d, 3]:\n", sensor_count, filter_count);
    for (int s = 0; s < sensor_count; s++) {
        for (int f = 0; f < filter_count; f++) {
            int base = (s * filter_count + f) * 3;
            printf("  [%d][%d]: fx=%.3f  fy=%.3f  fz=%.3f\n",
                   s, f,
                   mat_data[base + 0],
                   mat_data[base + 1],
                   mat_data[base + 2]);
        }
    }
    free(mat_data); free(mat_shp);

    /* 8. Destroy contact binding and SDK */
    ovphysx_destroy_contact_binding(handle, cb);

    // [tutorial-end]

    ovphysx_destroy_instance(handle);

    printf("[SUCCESS]\n");
    return 0;
}
