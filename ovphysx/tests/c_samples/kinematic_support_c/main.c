// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USD-KINEMATIC-SUPPORT-001
 * @covers AC-1 AC-3
 */

// Kinematic support sample: transform motion, surface velocity, and both.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>
#include <ovx/path_dictionary/path_dictionary.h>
#include "ovstage_sample.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <stdatomic.h>
#endif

#ifdef __cplusplus
#error "This file must be compiled as C, not C++"
#endif

#ifdef _WIN32
typedef volatile LONG object_change_counter_t;
#else
typedef atomic_uint object_change_counter_t;
#endif

static void object_change_counter_init(object_change_counter_t* counter)
{
#ifdef _WIN32
    InterlockedExchange(counter, 0);
#else
    atomic_init(counter, 0);
#endif
}

static void object_change_counter_increment(object_change_counter_t* counter)
{
#ifdef _WIN32
    InterlockedIncrement(counter);
#else
    atomic_fetch_add_explicit(counter, 1, memory_order_relaxed);
#endif
}

static unsigned int object_change_counter_load(object_change_counter_t* counter)
{
#ifdef _WIN32
    return (unsigned int)InterlockedCompareExchange(counter, 0, 0);
#else
    return atomic_load_explicit(counter, memory_order_relaxed);
#endif
}

typedef struct path_query_t
{
    ovstage_query_handle_t query;
    ovx_primpath_list_t path_list;
} path_query_t;

typedef struct object_change_counts_t
{
    object_change_counter_t created;
    object_change_counter_t destroyed;
} object_change_counts_t;

static void on_object_created(
    ovphysx_string_t prim_path,
    ovphysx_physx_type_t type,
    void* user_data)
{
    (void)prim_path;
    (void)type;
    object_change_counts_t* counts = (object_change_counts_t*)user_data;
    object_change_counter_increment(&counts->created);
}

static void on_object_destroyed(
    ovphysx_string_t prim_path,
    ovphysx_physx_type_t type,
    void* user_data)
{
    (void)prim_path;
    (void)type;
    object_change_counts_t* counts = (object_change_counts_t*)user_data;
    object_change_counter_increment(&counts->destroyed);
}

static int check_result(ovphysx_result_t result, const char* operation)
{
    if (result.status == OVPHYSX_API_SUCCESS)
        return 1;
    const ovphysx_string_t error = ovphysx_get_last_error();
    fprintf(stderr, "%s failed: %.*s\n", operation, (int)error.length, error.ptr ? error.ptr : "");
    return 0;
}

static int wait_step(ovphysx_handle_t handle, float dt)
{
    const ovphysx_enqueue_result_t step = ovphysx_step(handle, dt);
    if (step.status != OVPHYSX_API_SUCCESS)
        return 0;

    ovphysx_op_wait_result_t wait_result = { 0 };
    const ovphysx_result_t waited =
        ovphysx_wait_op(handle, step.op_index, OVPHYSX_TIMEOUT_INFINITE, &wait_result);
    const int ok = waited.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0;
    ovphysx_destroy_wait_result(&wait_result);
    return ok;
}

static int wait_ovstage(ovstage_instance_t* stage, ovstage_enqueue_result_t op)
{
    if (op.status != OVSTAGE_OK)
    {
        fprintf(stderr, "ovstage enqueue failed: %s\n", ovstage_get_error_string(stage, op.status));
        return 0;
    }
    if (op.op_index == OVSTAGE_INVALID_OP_ID)
        return 1;

    ovstage_op_wait_result_t wait_result;
    memset(&wait_result, 0, sizeof(wait_result));
    const ovstage_api_status_t status =
        ovstage_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait_result);
    const int ok = status == OVSTAGE_OK && wait_result.error_op_id_count == 0;
    if (!ok)
    {
        fprintf(stderr, "ovstage operation failed: %s\n", ovstage_get_error_string(stage, status));
        for (size_t i = 0; i < wait_result.error_op_id_count; ++i)
        {
            const ovx_string_t error =
                ovstage_get_last_op_error(stage, wait_result.error_op_ids[i]);
            fprintf(stderr, "  %.*s\n", (int)error.length, error.ptr ? error.ptr : "");
        }
    }
    return ovstage_release_op(stage, op.op_index) == OVSTAGE_OK && ok;
}

static ovx_string_or_token_t attribute_name(const char* name)
{
    ovx_string_or_token_t result;
    memset(&result, 0, sizeof(result));
    result.string.ptr = name;
    result.string.length = strnlen(name, 256);
    return result;
}

static int write_matrix(
    ovstage_instance_t* stage,
    ovstage_query_handle_t query,
    const char* attribute,
    ovstage_ordinal_t ordinal,
    double matrix[16])
{
    int64_t shape[1] = { 1 };
    DLTensor tensor;
    memset(&tensor, 0, sizeof(tensor));
    tensor.data = matrix;
    tensor.device.device_type = kDLCPU;
    tensor.dtype.code = kDLFloat;
    tensor.dtype.bits = 64;
    tensor.dtype.lanes = 16;
    tensor.ndim = 1;
    tensor.shape = shape;

    ovstage_write_data_t write;
    memset(&write, 0, sizeof(write));
    write.tensors = &tensor;
    write.tensor_count = 1;
    return wait_ovstage(
        stage,
        ovstage_write_attribute(
            stage, query, attribute_name(attribute), ordinal, write, OVSTAGE_PRIM_MODE_UPSERT));
}

static int write_transform(
    ovstage_instance_t* stage,
    ovstage_query_handle_t query,
    ovstage_ordinal_t ordinal,
    double x,
    double z)
{
    double matrix[16] = {
        6.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 6.0, 0.0,
        x,   0.0, z,   1.0
    };
    return write_matrix(stage, query, "omni:xform", ordinal, matrix) &&
           write_matrix(stage, query, "omni:fabric:worldMatrix", ordinal, matrix);
}

static int create_path_query(
    ovstage_instance_t* stage,
    const char* path,
    path_query_t* path_query)
{
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
        return 0;

    ovx_string_t path_string = { path, strnlen(path, 256) };
    ovx_api_result_t created =
        dictionary->vtable->create_path_list_from_strings(
            dictionary->context, &path_string, 1, &path_query->path_list);
    if (created.status != OVX_API_SUCCESS ||
        path_query->path_list == OVX_INVALID_PRIMPATH_LIST)
    {
        if (created.error.ptr)
            dictionary->vtable->release_error(dictionary->context, created.error);
        return 0;
    }

    const ovstage_api_status_t queried =
        ovstage_query_from_path_list(stage, path_query->path_list, &path_query->query);
    return queried == OVSTAGE_OK && path_query->query != OVSTAGE_INVALID_QUERY_HANDLE;
}

static int destroy_path_query(
    ovstage_instance_t* stage,
    path_query_t* path_query)
{
    int ok = 1;
    if (path_query->query != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ok = wait_ovstage(stage, ovstage_release_query(stage, path_query->query));
        path_query->query = OVSTAGE_INVALID_QUERY_HANDLE;
    }
    if (path_query->path_list != OVX_INVALID_PRIMPATH_LIST)
    {
        path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
        if (!dictionary)
            return 0;
        ovx_api_result_t released =
            dictionary->vtable->release_path_list_reference(
                dictionary->context, path_query->path_list);
        path_query->path_list = OVX_INVALID_PRIMPATH_LIST;
        if (released.error.ptr)
            dictionary->vtable->release_error(dictionary->context, released.error);
        ok = released.status == OVX_API_SUCCESS && ok;
    }
    return ok;
}

static int publish_control_ordinal(
    ovphysx_handle_t handle,
    ovstage_instance_t* stage,
    ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t floor;
    memset(&floor, 0, sizeof(floor));
    floor.ordinal = ordinal;
    floor.scope = OVSTAGE_SCOPE_ALL;
    if (!wait_ovstage(stage, ovstage_advance_write_floor(stage, &floor)))
        return 0;

    const ovstage_ordinal_range_t range = { ordinal, ordinal, true };
    return check_result(
        ovphysx_update_from_ovstage(handle, range),
        "update from ovstage");
}

static int create_binding(
    ovphysx_handle_t handle,
    const char* path,
    ovphysx_tensor_type_t type,
    ovphysx_tensor_binding_handle_t* binding)
{
    ovphysx_tensor_binding_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    desc.pattern.ptr = path;
    desc.pattern.length = strnlen(path, 256);
    desc.tensor_type = type;
    if (!check_result(ovphysx_create_tensor_binding(handle, &desc, binding), "create tensor binding"))
        return 0;

    ovphysx_tensor_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    if (!check_result(ovphysx_get_tensor_binding_spec(handle, *binding, &spec), "get tensor binding spec"))
        return 0;
    if (spec.ndim != 2 || spec.shape[0] != 1 || spec.shape[1] != 7)
    {
        fprintf(stderr, "Unexpected binding shape for %s\n", path);
        return 0;
    }
    return 1;
}

static DLTensor make_pose_tensor(float data[7], int64_t shape[2])
{
    DLTensor tensor;
    memset(&tensor, 0, sizeof(tensor));
    tensor.data = data;
    tensor.device.device_type = kDLCPU;
    tensor.ndim = 2;
    tensor.dtype.code = kDLFloat;
    tensor.dtype.bits = 32;
    tensor.dtype.lanes = 1;
    tensor.shape = shape;
    return tensor;
}

// NOTE: this sample uses the deprecated tensor-binding API. New code should use the session
// read/write API (ovphysx_read / ovphysx_write).
static int read_x(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    float* x)
{
    float data[7] = { 0 };
    int64_t shape[2] = { 1, 7 };
    DLTensor tensor = make_pose_tensor(data, shape);
    if (!check_result(ovphysx_read_tensor_binding(handle, binding, &tensor), "read rider pose"))
        return 0;
    *x = data[0];
    return 1;
}

int main(void)
{
    int exit_code = 1;
    ovphysx_handle_t handle = 0;
    ovphysx_sample_stage_attachment_t attachment;
    path_query_t target_platform = {
        OVSTAGE_INVALID_QUERY_HANDLE, OVX_INVALID_PRIMPATH_LIST
    };
    path_query_t combined_platform = {
        OVSTAGE_INVALID_QUERY_HANDLE, OVX_INVALID_PRIMPATH_LIST
    };
    object_change_counts_t object_changes;
    ovphysx_subscription_id_t object_change_subscription =
        OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_tensor_binding_handle_t target_rider = 0;
    ovphysx_tensor_binding_handle_t conveyor_rider = 0;
    ovphysx_tensor_binding_handle_t combined_rider = 0;
    memset(&attachment, 0, sizeof(attachment));
    object_change_counter_init(&object_changes.created);
    object_change_counter_init(&object_changes.destroyed);

    ovphysx_set_cpu_mode(true);
    if (!check_result(ovphysx_initialize(), "initialize"))
        return exit_code;

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    if (!check_result(ovphysx_create_instance(&args, &handle), "create instance"))
        goto cleanup;
    if (!ovphysx_sample_attach_usd_with_ovstage(
            handle, OVPHYSX_TEST_DATA "/kinematic_support.usda", &attachment))
        goto cleanup;

    ovphysx_object_change_callbacks_t callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.on_object_created = on_object_created;
    callbacks.on_object_destroyed = on_object_destroyed;
    callbacks.user_data = &object_changes;
    if (!check_result(
            ovphysx_subscribe_object_changes(
                &callbacks, &object_change_subscription),
            "subscribe to object changes"))
        goto cleanup;

    if (!create_path_query(attachment.stage, "/World/TargetPlatform", &target_platform) ||
        !create_path_query(attachment.stage, "/World/CombinedPlatform", &combined_platform) ||
        !create_binding(handle, "/World/TargetRider",
                        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, &target_rider) ||
        !create_binding(handle, "/World/ConveyorRider",
                        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, &conveyor_rider) ||
        !create_binding(handle, "/World/CombinedRider",
                        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, &combined_rider))
        goto cleanup;

    const float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 10; ++frame)
    {
        if (!wait_step(handle, dt))
            goto cleanup;
    }

    float target_start = 0.0f;
    float conveyor_start = 0.0f;
    float combined_start = 0.0f;
    if (!read_x(handle, target_rider, &target_start) ||
        !read_x(handle, conveyor_rider, &conveyor_start) ||
        !read_x(handle, combined_rider, &combined_start))
        goto cleanup;

    for (int frame = 1; frame <= 120; ++frame)
    {
        const float x = frame * dt;
        const ovstage_ordinal_t ordinal = (ovstage_ordinal_t)frame + attachment.ordinal;
        if (!write_transform(attachment.stage, target_platform.query, ordinal, x, 0.0) ||
            !write_transform(attachment.stage, combined_platform.query, ordinal, x, 16.0) ||
            !publish_control_ordinal(handle, attachment.stage, ordinal) ||
            !wait_step(handle, dt))
            goto cleanup;
    }

    float target_end = 0.0f;
    float conveyor_end = 0.0f;
    float combined_end = 0.0f;
    if (!read_x(handle, target_rider, &target_end) ||
        !read_x(handle, conveyor_rider, &conveyor_end) ||
        !read_x(handle, combined_rider, &combined_end))
        goto cleanup;

    const float target_dx = target_end - target_start;
    const float conveyor_dx = conveyor_end - conveyor_start;
    const float combined_dx = combined_end - combined_start;
    printf("Rider displacement: transform=%.3f, surface=%.3f, combined=%.3f\n",
           target_dx, conveyor_dx, combined_dx);

    const float independent_max = target_dx > conveyor_dx ? target_dx : conveyor_dx;
    const unsigned int created = object_change_counter_load(&object_changes.created);
    const unsigned int destroyed = object_change_counter_load(&object_changes.destroyed);
    printf("Physics object changes: created=%u, destroyed=%u\n", created, destroyed);
    if (created != 0 || destroyed != 0)
        fprintf(stderr,
                "Transform updates reconstructed physics objects "
                "(created=%u, destroyed=%u)\n",
                created, destroyed);
    else if (target_dx < 0.5f)
        fprintf(stderr, "ovstage transform did not carry its rider\n");
    else if (conveyor_dx < 0.5f)
        fprintf(stderr, "Surface velocity did not carry its rider\n");
    else if (combined_dx <= independent_max + 0.2f)
        fprintf(stderr, "Combined transform and surface velocity were not observably additive\n");
    else
        exit_code = 0;

cleanup:
    if (object_change_subscription != OVPHYSX_INVALID_SUBSCRIPTION_ID)
        check_result(
            ovphysx_unsubscribe_object_changes(object_change_subscription),
            "unsubscribe from object changes");
    if (attachment.stage)
    {
        destroy_path_query(attachment.stage, &target_platform);
        destroy_path_query(attachment.stage, &combined_platform);
    }
    if (target_rider)
        ovphysx_destroy_tensor_binding(handle, target_rider);
    if (conveyor_rider)
        ovphysx_destroy_tensor_binding(handle, conveyor_rider);
    if (combined_rider)
        ovphysx_destroy_tensor_binding(handle, combined_rider);
    if (handle)
    {
        ovphysx_sample_destroy_stage(handle, &attachment);
        ovphysx_destroy_instance(handle);
    }
    ovphysx_shutdown();
    return exit_code;
}
