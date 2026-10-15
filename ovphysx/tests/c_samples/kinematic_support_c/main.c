// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USD-KINEMATIC-SUPPORT-001
 * @covers AC-1 AC-3
 */

// Kinematic support sample: transform motion, surface velocity, and both.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
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

static int wait_step(ovphysx_instance_handle_t handle, float dt)
{
    const ovphysx_enqueue_result_t step = ovphysx_step(handle, dt);
    if (step.status != OVPHYSX_API_SUCCESS)
        return 0;

    ovphysx_wait_task_result_t wait_result = { 0 };
    const ovphysx_result_t waited =
        ovphysx_wait_task(handle, step, OVPHYSX_TIMEOUT_INFINITE, &wait_result);
    const int ok = waited.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0;
    ovphysx_destroy_wait_task_result(&wait_result);
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
    ovphysx_instance_handle_t handle,
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
        ovphysx_sample_complete_task(handle, ovphysx_update_from_ovstage(handle, range)),
        "update from ovstage");
}

// Reads the world x of the rider at `path` through a session read selected by the rider's path.
static int read_x(
    ovphysx_instance_handle_t handle,
    const char* path,
    float* x)
{
    ovphysx_query_selection_t selection;
    memset(&selection, 0, sizeof(selection));
    selection.pattern.ptr = path;
    selection.pattern.length = strnlen(path, 256);
    ovphysx_query_handle_t query = 0;
    if (!check_result(
            ovphysx_sample_complete_task(
                handle, ovphysx_query_selection(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection, &query, NULL)),
            "open rider query"))
        return 0;

    ovx_string_or_token_t position;
    memset(&position, 0, sizeof(position));
    position.string.ptr = OVPHYSX_ATTR_POSITION;
    position.string.length = strlen(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    const ovstage_read_group_t* group = NULL;
    int ok = check_result(
        ovphysx_sample_complete_task(handle, ovphysx_read(handle, query, &position, 1, &read, NULL)), "read rider pose");
    if (ok)
        ok = check_result(
            ovphysx_sample_complete_task(handle, ovphysx_fetch_read_next(handle, read, &group)), "fetch rider pose");
    if (ok && (!group || group->prims.count != 1 || group->data.tensor_count != 1 || !group->data.tensors[0].data))
    {
        fprintf(stderr, "Expected one rigid body at %s\n", path);
        ok = 0;
    }
    if (ok)
    {
        // The sample runs in CPU mode, so the position column is host memory.
        const DLTensor* tensor = &group->data.tensors[0];
        *x = ((const float*)((const char*)tensor->data + tensor->byte_offset))[0];
    }

    if (group)
        ok = check_result(
                 ovphysx_sample_complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)),
                 "release rider group") && ok;
    if (read)
        ok = check_result(
                 ovphysx_sample_complete_task(handle, ovphysx_release_read(handle, read)), "release rider read") && ok;
    return check_result(
               ovphysx_sample_complete_task(handle, ovphysx_release_query(handle, query)), "release rider query") && ok;
}

int main(void)
{
    int exit_code = 1;
    ovphysx_instance_handle_t handle = 0;
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
    memset(&attachment, 0, sizeof(attachment));
    object_change_counter_init(&object_changes.created);
    object_change_counter_init(&object_changes.destroyed);

    ovphysx_set_cpu_mode(true);
    if (!check_result(ovphysx_initialize(), "initialize"))
        return exit_code;

    if (!check_result(ovphysx_create_instance(&handle), "create instance"))
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
                handle, &callbacks, &object_change_subscription),
            "subscribe to object changes"))
        goto cleanup;

    if (!create_path_query(attachment.stage, "/World/TargetPlatform", &target_platform) ||
        !create_path_query(attachment.stage, "/World/CombinedPlatform", &combined_platform))
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
    if (!read_x(handle, "/World/TargetRider", &target_start) ||
        !read_x(handle, "/World/ConveyorRider", &conveyor_start) ||
        !read_x(handle, "/World/CombinedRider", &combined_start))
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
    if (!read_x(handle, "/World/TargetRider", &target_end) ||
        !read_x(handle, "/World/ConveyorRider", &conveyor_end) ||
        !read_x(handle, "/World/CombinedRider", &combined_end))
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
            ovphysx_unsubscribe_object_changes(handle, object_change_subscription),
            "unsubscribe from object changes");
    if (attachment.stage)
    {
        destroy_path_query(attachment.stage, &target_platform);
        destroy_path_query(attachment.stage, &combined_platform);
    }
    if (handle)
    {
        ovphysx_sample_destroy_stage(handle, &attachment);
        ovphysx_destroy_instance(handle);
    }
    ovphysx_shutdown();
    return exit_code;
}
